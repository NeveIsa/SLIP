
#ifndef UTIL_H
#define UTIL_H

#ifndef htons
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#endif

#ifndef ntohs
#define ntohs(x) htons(x)
#endif

#ifndef htonl
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#endif

#ifndef ntohl
#define ntohl(x) htonl(x)
#endif

#endif




//////////////// SLIP STRUCTS //////////////////////

struct IPpacket
{
  uint8_t ip_version:4;
  uint8_t ip_header_len:4;
  uint8_t ip_tos;
  uint16_t ip_len;

  uint16_t ip_id;
  uint16_t ip_flags_and_offset;
  
  uint8_t ip_ttl;
  uint8_t ip_proto;
  uint16_t ip_hdr_cksum;

  uint32_t ip_src;
  uint32_t ip_dst;

  char data[];
}
__attribute__((packed));

typedef struct IPpacket IPpacket_t; 


///////////////// SLIP LINK LEVEL //////////////////////

class slip
{
  #define SERIALIFACE HardwareSerial
  #define DEBUGGER Serial
  public:
    #define END       0xC0
    #define ESC       0xDB
    #define ESC_END   0xDC
    #define ESC_ESC   0xDD

    #define PROTO_ICMP   1
    #define PROTO_TCP    6 
    #define PROTO_UDP    17


    uint32_t selfIP;
    SERIALIFACE *IFACE;
    uint8_t rPacket[128];
    uint16_t len_of_last_valid_packet;
    IPpacket_t *ipPacket;

    slip(SERIALIFACE &s, long baud);
    void init(uint32_t ip);
 
    uint16_t iphdr_checksum();
    uint16_t readPacket();
    uint16_t writePacket();

    //debug
    void IPrepr(uint32_t ip,char *buff);
    void dump_header();
    
};


slip::slip(SERIALIFACE &s, long  baud)
{
  IFACE = &s;
  IFACE->begin(baud);
}

void slip::init(uint32_t ip)
{
  selfIP = ip;
  
  //parse by setting pointer to IPHeader structure
  ipPacket = (IPpacket_t*)rPacket;

  len_of_last_valid_packet=0; //reset
}


uint16_t slip::readPacket()
{
  static uint16_t index=0;
  uint16_t tmp;
 

  //while(!IFACE->available());
  //delay(50);
  
  while(IFACE->available())
  {
    uint8_t rcv = IFACE->read();

    //Serial.println(rcv);
    
    if(rcv==ESC)
    {
      
      while(!IFACE->available()); //wait for next byte
      char rcv2 = IFACE->read();

      if(rcv2 == ESC_ESC) rPacket[index++]=ESC;
      else if(rcv2 == ESC_END) rPacket[index++]=END;
    }
    
    else if(rcv==END) 
    {
      
      tmp = index;
      index=0; //reinit

      // SANITY CHECKS
      
      //check for minimum IPv4 header size, else discard
      if (tmp < 20) 
      {
        len_of_last_valid_packet=0;
        return 0;
      }
      
      // check if the dst IP matches mine  
      if (((IPpacket_t*)rPacket)->ip_dst != htonl(selfIP)) 
      {
        len_of_last_valid_packet=0;
        return 0;
      }

      len_of_last_valid_packet = tmp;
      return tmp;
    }
    else
    {
      rPacket[index++] = rcv;
    }
  }

  return 0;
}

uint16_t slip::writePacket()
{
  uint16_t len = ipPacket->ip_len;

  ipPacket->ip_ttl = 64;
  
  for(uint16_t i=0;i<len;i++)
  {
    IFACE->write(rPacket[i]);
  }
}


//////////////////////// SLIP IP LEVEL  ////////////////////

uint16_t slip::iphdr_checksum()
{
  uint16_t *p = (uint16_t*)ipPacket;

  uint32_t cksum=0;

  //save current cksum
  uint16_t current_cksum=ipPacket->ip_hdr_cksum;

  //set to zero for cksum calculation
  ipPacket->ip_hdr_cksum=0;
  
  //header is 20 bytes, hence we need 10 16bit words
  for(uint8_t i=0;i<10;i++)
  {
    cksum+=p[i];
  }

  while(cksum> 0xffff)
  {
    cksum += (uint16_t)(cksum >> 16);
  }

  //restore original cksum
  ipPacket->ip_hdr_cksum = current_cksum;
  
  return cksum;
}
  


//////////////// SLIP DEBUG ////////////////////////

void slip::IPrepr(uint32_t ip, char* buf)
{  
  sprintf(buf,"%d.%d.%d.%d",(uint8_t)(ip>>24 ) , (uint8_t)((ip >> 16)) , (uint8_t)((ip >> 8)) , (uint8_t)(ip));
}

void slip::dump_header()
{
  if(!len_of_last_valid_packet) DEBUGGER.println("Last IP packet is invalid...");

  char buff[20];

  // wall
  DEBUGGER.println("--------------------------------");
  
  //src
  IPrepr(ntohl(ipPacket->ip_src),buff);
  DEBUGGER.print("SRC: "); DEBUGGER.println(buff);

   //dst
  IPrepr(ntohl(ipPacket->ip_dst),buff);
  DEBUGGER.print("DST: "); DEBUGGER.println(buff);

  //len
  DEBUGGER.print("LEN: ");
  DEBUGGER.println(ntohs(ipPacket->ip_len));

  //proto
  DEBUGGER.print("PROTO: ");
  DEBUGGER.print(ipPacket->ip_proto);
  if(ipPacket->ip_proto == PROTO_ICMP)
    DEBUGGER.println(" -> ICMP");
  else if(ipPacket->ip_proto == PROTO_TCP)
    DEBUGGER.println(" -> TCP");
  else if(ipPacket->ip_proto == PROTO_UDP)
    DEBUGGER.println(" -> UDP");
  else
    DEBUGGER.println(" -> Protocol Not Supported Yet");
  
  

  // wall
  DEBUGGER.println("--------------------------------");
  
}




////////////////

slip network = slip(Serial1,9600);
///////////////

void setup() {
  // put your setup code here, to run once

  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  Serial.begin(115200);
  //pinMode(13,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int z=network.readPacket();
  
  if(z)
  {
    network.dump_header();
  }
  
  
}
