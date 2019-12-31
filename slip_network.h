
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
    uint8_t rPacket[255];
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
    void handleTCP();
    uint8_t stateMachine();

    //high level
    #define UDP_CB_PORTS_MAX 5
    uint16_t udpCBports[UDP_CB_PORTS_MAX];
    void (*udpCB[UDP_CB_PORTS_MAX])();
    void udpCBregister(uint16_t port, void (*cb)());
    
    uint16_t tcpCBport;
    uint8_t (*tcpCB)(uint8_t *rx, uint8_t rxlen, uint8_t *tx);
    void tcpCBregister(uint16_t port,uint8_t (*cb)(uint8_t *rx, uint8_t rxlen, uint8_t *tx));
    uint16_t tcpChecksum();


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

  //Serial.println(IFACE->available());
  
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
      
      
      // check if the dst doesn't match mine
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
  return htons(cksum);
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

//////////// TCP ////////////

#define ACK   0x10
#define FIN   0x01
#define SYN   0x02
#define RST   0x04

struct TCPpacket 
{
  uint16_t port_src;
  uint16_t port_dst;
  
  uint32_t seqno;
  uint32_t ackno;

  uint8_t reserved:4;     // dont know why it is mentioned as header_len and then reserved while the sequence reserved, header_len works out
  uint8_t header_len:4;   // also called data offset - in 32bit words
  
  uint8_t flags;
  
  uint16_t  window_size;

  uint16_t cksum;
  uint16_t urgent_pointer;

  uint8_t data[]; //may contain optional header data
}
__attribute__((packed));

typedef struct TCPpacket TCPpacket_t;

uint16_t slip::tcpChecksum()
{
  TCPpacket_t *tcpPacket = (TCPpacket_t*)ipPacket->data;
  
  uint16_t got_cksum = tcpPacket->cksum;

  //set to zero before calc.
  tcpPacket->cksum=0;

  uint16_t tcp_packet_len = ntohs(ipPacket->ip_len) - 20; //IP header is in general 20 bytes

  uint32_t cksum=0;


  //pseudo header
 

  //ip_src
  uint16_t src_l = ipPacket->ip_src & 0xffff;
  uint16_t src_h = (ipPacket->ip_src>>16) & 0xffff;
  cksum+=ntohs(src_l);
  if(cksum>0xffff) cksum-=0xffff;
  cksum+=ntohs(src_h);
  if(cksum>0xffff) cksum-=0xffff;

  //ip_dst
  uint16_t dst_l = ipPacket->ip_dst & 0xffff;
  uint16_t dst_h = (ipPacket->ip_dst>>16) & 0xffff;
  cksum+=ntohs(dst_l);
  if(cksum>0xffff) cksum-=0xffff;
  cksum+=ntohs(dst_h);
  if(cksum>0xffff) cksum-=0xffff;

   // tcp - protocol -> 6
  cksum+=0x0006;
  if(cksum>0xffff) cksum-=0xffff;
  
  //tcp length
  cksum+=tcp_packet_len; 
  if(cksum>0xffff) cksum-=0xffff;

  
  
  uint16_t *words = (uint16_t*)tcpPacket;
  for(uint16_t i=0; i < tcp_packet_len/2; i++)
  {
   cksum += ntohs(words[i]);
   if(cksum > 0xffff) cksum -= 0xffff;
  }

  if(tcp_packet_len & 1) //check if odd
  {
    uint16_t lastword = 0;
    uint8_t *lastbyte = (uint8_t*)tcpPacket;

    // https://www.netfor2.com/tcpsum.htm -> check this for padding odd bytes in TCP for cksum calc.
    lastword = lastbyte[tcp_packet_len - 1];
    cksum+=ntohs(lastword);

    if(cksum>0xffff) cksum-=0xffff;
  }



  

  //restore original cksum
  tcpPacket->cksum = got_cksum;
  
  cksum = ~cksum;
  return ntohs(cksum);
  
}


void slip::tcpCBregister(uint16_t port, uint8_t (*cb)(uint8_t *rx, uint8_t rxlen, uint8_t *tx))
{
    tcpCBport = port;
    tcpCB = cb;
}




void slip::handleTCP()
{
  #define MAX_RCV_WINDOW           190
  
  #define TCP_STATE_IDLE           0
  #define TCP_STATE_SENT_SYN       1
  #define TCP_STATE_SYN_ACKED      2
  #define TCP_STATE_SENT_FIN       3
  
  static uint8_t TCP_SERVER_STATE = TCP_STATE_IDLE;
  
 // static uint32_t lastackno = 0;
  static uint32_t myseqno = 100UL;

  static uint16_t other_host_port = 0;
  static long tcp_state_last_idle = millis();

  //if not idle for more than 5seconds, reset TCP STATE
  if( (TCP_SERVER_STATE!=TCP_STATE_IDLE) && (millis() - tcp_state_last_idle  > 5000) ) 
  {
    DEBUGGER.println("Resetting Stale Connection...");
    //reset TCP STATE
    TCP_SERVER_STATE = TCP_STATE_IDLE;
    myseqno=100UL;
    other_host_port=0;
    
  }
  
  else if(TCP_SERVER_STATE==TCP_STATE_IDLE)
  {
    tcp_state_last_idle = millis();
  }
  
  

  //flag to send packet back after modification
  uint8_t SEND_PACKET_BACK_FLAG=0;
  
	TCPpacket_t* tcpPacket = (TCPpacket_t*)ipPacket->data;

      /* check we calculate correct tcpcksum
      Serial.println("TCPcksum");
      Serial.println(tcpPacket->cksum);
      Serial.println(tcpChecksum());*/


  // CHECK IF GOT RESET FLAG
  if(tcpPacket->flags & RST)
  {
    DEBUGGER.println("RST");
    //reset the connection
    TCP_SERVER_STATE = TCP_STATE_IDLE;
    myseqno=100UL;
    other_host_port=0;
    
    tcp_state_last_idle = 0; //this will reset the connection aswell

    return;
  }


  

  if((tcpPacket->flags & FIN) &&  other_host_port != tcpPacket->port_src) return;
  if(other_host_port && other_host_port != tcpPacket->port_src) return;

  //Serial.println(ntohs(tcpPacket->port_dst));
  
  if(ntohs(tcpPacket->port_dst)==tcpCBport)
  {
    // TCP state machine

    //Serial.println(tcpPacket->flags);


    // NEW SYN?
    if( TCP_SERVER_STATE==TCP_STATE_IDLE && (tcpPacket->flags & SYN)) 
    {
      DEBUGGER.println("SYN");
      //modify and send the same packet back
      
      tcpPacket->flags |= ACK; //set ack

      tcpPacket->ackno = htonl(ntohl(tcpPacket->seqno) + 1); //set ack no to the next expected seqno
      

      tcpPacket->seqno = htonl(myseqno); //set your sending side seqno. - note > acking to the syn is count as 1 byte of data
      
      if(myseqno==100UL)myseqno++; //do this only once, initial myseqno is 100
       
      //we cannot handle tcp segments with data larger than 80 bytes.
      tcpPacket->window_size = htons(MAX_RCV_WINDOW);


      //exchange ports
      other_host_port = tcpPacket->port_src;
      tcpPacket->port_src = tcpPacket->port_dst;
      tcpPacket->port_dst = other_host_port;

      //set checksum
      //tcpPacket->cksum = tcpChecksum();


       TCP_SERVER_STATE = TCP_STATE_SENT_SYN;

       SEND_PACKET_BACK_FLAG=1; //write back packet
      
    }

    
    //ACK for SYN?
    else if( TCP_SERVER_STATE==TCP_STATE_SENT_SYN && (tcpPacket->flags & ACK) )
    {
      DEBUGGER.println("ACK");

      TCP_SERVER_STATE = TCP_STATE_SYN_ACKED;
      
    }

     
    // DATA?
    else if( TCP_SERVER_STATE==TCP_STATE_SYN_ACKED )
    {
      //IP header is 20 bytes each in general - TCP header length field gives length in 32bit words
      uint8_t tcp_hdr_len_in_bytes = tcpPacket->header_len*32/8;
      uint8_t tcpdatalen = ntohs(ipPacket->ip_len) - 20 - tcp_hdr_len_in_bytes; 
      
      uint8_t *tcpdata = (uint8_t*)tcpPacket + tcp_hdr_len_in_bytes ; // skip tcp headers

      #if SLIP_DEBUG
      DEBUGGER.println("TCPDATA >>>");
      for(uint8_t i=0;i<tcpdatalen;i++)DEBUGGER.write(tcpdata[i]);
      DEBUGGER.println("<<< TCPDATA");
      #endif

      // If the packet has no data + ACK + !FIN, drop the packet.
      if(tcpdatalen==0 && (tcpPacket->flags & ACK) && !(tcpPacket->flags & FIN) ) return;

      //create ACK and send back
      
      //tcpPacket->ackno = htonl(ntohl(tcpPacket->seqno) + 3); //set ack no to the next expected seqno.
      //tcpPacket->ackno = ntohl(100UL);
      //Serial.println(ntohl(tcpPacket->ackno));
      //tcpPacket->flags = ACK; //set ACK flag
      //tcpPacket->seqno = ntohl(100UL); 
      
      
      
      TCPpacket_t *ackPacket = (TCPpacket_t*)calloc(sizeof(TCPpacket_t),1);
      

      if(tcpPacket->flags & FIN)
      {
        ackPacket->flags = ACK | FIN;
        ackPacket->ackno = htonl(ntohl(tcpPacket->seqno) + tcpdatalen + 1); // +1 for FIN is counted as one data byte
        TCP_SERVER_STATE=TCP_STATE_SENT_FIN;
      }
      else
      {
        ackPacket->flags = ACK;
        ackPacket->ackno = htonl(ntohl(tcpPacket->seqno) + tcpdatalen);
      }
      
      ackPacket->seqno = ntohl(myseqno); //this is only as long as we have 

      ackPacket->header_len = 5; //5*32/8 = 20 bytes

      //exchange ports
      ackPacket->port_dst = tcpPacket->port_src;
      ackPacket->port_src = tcpPacket->port_dst;
      

      //we cannot handle tcp segments with data larger than 80 bytes.
      ackPacket->window_size = htons(MAX_RCV_WINDOW);

      uint8_t *txdata = (uint8_t*) calloc(190,1); //can send maximum 180 bytes -> Note - IP+TCP header max is 60 -> 60+190 = 250 < uint8_t range = 255 
      uint8_t txlen=0;
      
      if(TCP_SERVER_STATE != TCP_STATE_SENT_FIN)
      {
        
        //TCP CALLBACK
        txlen = tcpCB(tcpdata,tcpdatalen,txdata); //returns data to  xmit in txdata
        
       
      }
     

      //copy ack packet
      memcpy(tcpPacket,ackPacket,sizeof(TCPpacket_t));
       
      // copy the tx data into tcp(ack) packet - note the tcp header is 20
      memcpy((uint8_t*)tcpPacket+20, txdata, txlen);


      // FREE the memory claimed
      free(ackPacket);
      free(txdata);
  
      
      
      //update myseqno
      myseqno += txlen;

    
      
      //set ackPacket total length in IP header to 40 (20IP + 20TCP) + txlen
      ipPacket->ip_len = htons(40 + txlen);

      //update ipv4 checksum
      ipPacket->ip_hdr_cksum = iphdr_checksum();

       
      SEND_PACKET_BACK_FLAG=1; //write back packet
      
      
    }

    else if(TCP_SERVER_STATE==TCP_STATE_SENT_FIN)
    {
      if( (tcpPacket->flags | ACK) && ntohl(tcpPacket->ackno)==myseqno+1 ) 
      {
        TCP_SERVER_STATE=TCP_STATE_IDLE;

         // these needs to be initialized as well
        other_host_port = 0;
        myseqno=100UL;
        
      }
    }


    if(SEND_PACKET_BACK_FLAG)
    {
      
      // exchange IP - 
      // no need to update IPcksum as IPcksum is calculated only over IPheader and since we are exchanging, IPcksum doesn't change.
      uint32_t tempIP = ipPacket->ip_src;
      ipPacket->ip_src = ipPacket->ip_dst;
      ipPacket->ip_dst = tempIP;


      //set tcp checksum
      tcpPacket->cksum = tcpChecksum();


       //send the packet
       writePacket();  
    }
    
   
  }
  else
  {
    //close the connection or else TCP will flood with retransmission
    
  }
 
}

//////////// TCP ////////////

uint8_t slip::stateMachine()
{
  int len_of_packet=readPacket();
  if(len_of_packet ==0 ) return 0;


  //parse by setting pointer to IPHeader structure
  ipPacket = (IPpacket_t*)rPacket;

  
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
    //DEBUGGER.println("TCP is not supported yet...");
    handleTCP();
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
