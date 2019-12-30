
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

    #define SLIP_DEBUG 1


    uint32_t selfIP;
    SERIALIFACE *IFACE;
    uint8_t rPacket[128];
    uint16_t len_of_last_valid_packet;
    IPpacket_t *ipPacket;

    slip(SERIALIFACE &s, long baud);
    void init(uint32_t ip);

    // low level
    uint16_t iphdr_checksum();
    uint16_t readPacket();
    uint16_t writePacket();


    //mid level
    void handleICMP();
    void handleUDP();
    uint8_t stateMachine();

    //high level
    #define UDP_CB_PORTS_MAX 5
    uint16_t udpCBports[UDP_CB_PORTS_MAX];
    void (*udpCB[UDP_CB_PORTS_MAX])();
    void udpCBregister(uint16_t port, void (*cb)());
    
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

  //reset the udpCBports
  for(int i=0;i<UDP_CB_PORTS_MAX;i++) udpCBports[i]=0;
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
  uint16_t len = ntohs(ipPacket->ip_len);

  ipPacket->ip_ttl = 64;


  //END - flushes any garbage on the wire due to noise
  IFACE->write(END);


  
  for(uint16_t i=0;i<len;i++)
  {
    if(rPacket[i]==END)
    {
      IFACE->write(ESC);
      IFACE->write(ESC_END);
    }
    else if(rPacket[i]==ESC)
    {
      IFACE->write(ESC);
      IFACE->write(ESC_ESC);
    }
    else
    {
      IFACE->write((char)rPacket[i]);
    }
  }


  //END the SLIP frame
  IFACE->write(END);
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
    cksum+=ntohs(p[i]);
    if(cksum>0xffff) cksum -= 0xffff;
  }


  //restore original cksum
  ipPacket->ip_hdr_cksum = current_cksum;

  cksum = ~cksum;
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
  DEBUGGER.println("\n--------------------------------IP");
  
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
  DEBUGGER.println("--------------------------------IP");
  
}


////////////////////////////// SLIP NETWORK STATEMACHINE /////////////////////////


//////////// ICMP ////////////

struct ICMPpacket
{
  uint8_t type;
  uint8_t code;
  uint16_t cksum;

  // there are some other header fields but we will ignore them
  uint32_t rest_of_header;

  char data[];  
  
}
__attribute__((packed));

typedef struct ICMPpacket ICMPpacket_t;

void slip::handleICMP()
{
  // only ICMP ECHO (ping) requests are handled - 

  ICMPpacket_t *icmpPacket = (ICMPpacket_t*)ipPacket->data;

  #if SLIP_DEBUG
   // wall
  DEBUGGER.println("--------------------------------ICMP");

  DEBUGGER.print("TYPE: ");
  DEBUGGER.println(icmpPacket->type);
  
  DEBUGGER.print("CODE: ");
  DEBUGGER.println(icmpPacket->code);

  
   // wall
  DEBUGGER.println("--------------------------------ICMP");
  #endif
   
  //check if echo request - type-8, code-0
  if(icmpPacket->type == 8 && icmpPacket->code==0)
  {
    #if SLIP_DEBUG
    DEBUGGER.println("Replying to echo (ping) request...");
    #endif
    
      
   //convert to echo reply - type-0, code-0
   icmpPacket->type = 0; 

   //ICMP checksum needs to be updated - we add 0x800 as we reduced the type from 8 to 0.
   // and since checksum is calculated using words (16bits), we may observe that the icmp-type field is
   // the first 8 bits of the first 16bit word in the ICMP header. Hence we reduced the first word by 0x800
   // hence the checksum which is (0xffff - sum of words) must increase by 0x800
   icmpPacket->cksum = htons(ntohs(icmpPacket->cksum) + 0x800);
   

   //exchange src IP and dest IP
   uint32_t tempIP = ipPacket->ip_src;
   ipPacket->ip_src = ipPacket->ip_dst;
   ipPacket->ip_dst    = tempIP;

   //send the ping reply packet
   writePacket();

  }
  
}

//////////// ICMP ////////////


//////////// UDP /////////////

struct UDPpacket
{
  uint16_t port_src;
  uint16_t port_dst;
  uint16_t len;
  uint16_t cksum;
}
__attribute__((packed));

typedef struct UDPpacket UDPpacket_t;

void slip::handleUDP()
{
    // UDP checksum is optional, hence we do not verify/use checksum while receiving/sending UDP packets

  UDPpacket_t *udpPacket = (UDPpacket_t*)ipPacket->data;

  #if SLIP_DEBUG
   // wall
  DEBUGGER.println("--------------------------------UDP");

  DEBUGGER.print("SRC: ");
  DEBUGGER.println(ntohs(udpPacket->port_src));
  
  DEBUGGER.print("DEST: ");
  DEBUGGER.println(ntohs(udpPacket->port_dst));

  uint8_t udpdatalen = ntohs(udpPacket->len) - 8;
  DEBUGGER.print("UDP DATA LEN: ");
  DEBUGGER.println(udpdatalen); //UDP header is 8 bytes

  DEBUGGER.println("UDP DATA >>>");
  uint8_t *udpdata = (uint8_t *)(udpPacket)+8;
  for(uint8_t i=0;i<udpdatalen;i++) DEBUGGER.write(udpdata[i]);
  DEBUGGER.println("<<< UDP DATA");

   // wall
  DEBUGGER.println("--------------------------------UDP");
  #endif


  //check if callback ports are non-empty
  for(uint8_t i=0;i<UDP_CB_PORTS_MAX;i++)
  {
    
    uint16_t port = ntohs(udpPacket->port_dst);
    
    if(port == udpCBports[i]) 
    {
      
      udpCB[i](); 
    }
  }

  
}

void slip::udpCBregister(uint16_t port, void (*cb)())
{
  uint8_t i;
  for(i=0;i< UDP_CB_PORTS_MAX && udpCBports[i]!=0; i++);

  udpCBports[i] = port;
  udpCB[i] = cb;
}
//////////// UDP /////////////

uint8_t slip::stateMachine()
{
  int len_of_packet=readPacket();
  if(len_of_packet ==0 ) return 0;

  
  #if SLIP_DEBUG
  dump_header();
  #endif

  switch (ipPacket->ip_proto)
  {
    case PROTO_ICMP:
    handleICMP();
    break;

    case PROTO_TCP:
    #if SLIP_DEBUG
    DEBUGGER.println("TCP is not supported yet...");
    break;
    #endif

    case PROTO_UDP:
    handleUDP();
    break;

    default:
    #if SLIP_DEBUG
    DEBUGGER.print("PROTO: ");
    DEBUGGER.print(ipPacket->ip_proto);
    DEBUGGER.println(" -> This Protocol is not supported yet...");
    #endif
    
    break;
     
  }
  
  
}



////////////////

slip network = slip(Serial3,9600);
///////////////

void udp_handler_5000()
{
  Serial.println("---> 5000");
}
void udp_handler_6000()
{
  Serial.println("---> 6000");
}

void setup() {
  // put your setup code here, to run once

  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  Serial.begin(115200);
  //pinMode(13,OUTPUT);


  network.udpCBregister(5000,udp_handler_5000);
  network.udpCBregister(6000,udp_handler_6000);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  network.stateMachine();
  //network.writePacket();
  
 
  
  
}
