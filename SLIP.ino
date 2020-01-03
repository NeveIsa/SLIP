
#define LEDPIN 13
#include "slip_network.h"

//create the SLIP network
slip network = slip(Serial2, 115200);

  
 /*
 * 
 * callback for udp_server - this is an echo server,
 * any data received will be echoed back
 * 
  */
uint8_t udp_echo_server(uint8_t *rx, uint8_t rxlen, uint8_t *tx)
{

  /*
   * rx -> pointer to the received UDP payload data
   * rxlen -> number of bytes of UDP payload data received | max ~= 220 bytes
   * tx -> data to transmit
   * 
   * return -> number of bytes of data to be transmitted = length of tx
   * 
   */
  
  //echo server - copy rx to tx and return length of data received which also equals the length of data to be transmitted
  memcpy(tx,rx,rxlen);
  return rxlen; 
}

uint8_t udp_handler_6000(uint8_t *rx, uint8_t rxlen, uint8_t *tx)
{
  /*
   * 
   * UDP handler on port 6000
   * Reply with a msg -> "Received: n bytes" where n=rxlen
   * 
   */

   String txdata = String("Received: " + String(rxlen) + " bytes\n\n"); 
   uint8_t txlen = txdata.length();
   uint8_t *txbytes =  (uint8_t*)malloc(txlen);
   
   txdata.toCharArray((char*)txbytes,txlen);
   memcpy((char*)tx,txbytes,txlen);

   return txlen;
   
  
  
}


/*
 * BASIC TCP Server 
 * Replied with HTTP response with content set to analogRead(A0);
 * 
 * Also toggles the LED everytime the an HTTP Request is made
 */

uint8_t tcp_server(uint8_t *rx, uint8_t rxlen, uint8_t *tx)
{
  
  //for(uint8_t i=0;i<rxlen;i++) Serial.write(rx[i]);

  uint8_t val = analogRead(A0);
  String senddata = "HTTP/1.1 200 OK\r\nContent-Length: "+ String(String(val).length()+2) +"\r\n\r\n"+String(val)+"\r\n";
  int senddatalen = senddata.length();
  
  char csenddata[senddatalen+1];
  senddata.toCharArray(csenddata,senddatalen+1); //toCharArray - 2nd argument is buffer lenght, hence must pass 1 more than string size for toCharArray to add null terminator

  //for(uint8_t i=0;i<senddatalen;i++) {Serial.print(i);Serial.print(" -> ");Serial.print((uint8_t)csenddata[i]); Serial.print(" ");Serial.println((uint8_t)senddata[i]);}
  
  memcpy(tx,csenddata,senddatalen);

  Serial.println(csenddata);
  static int LED_STATE=HIGH;
  LED_STATE = LED_STATE==HIGH?LOW:HIGH;
  digitalWrite(LEDPIN,LED_STATE);
  
  return senddatalen;
}



/*
 * Callback for TCP Client
 */
void tcpRXcb(uint8_t *rxdata, uint8_t rxlen)
{
  for(uint8_t i=0;i<rxlen;i++)Serial.write(rxdata[i]);
}



void setup() {

  /*
   * Set this device IP to 10.10.10.1
   */
  
  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  // INITIALIZE DEBUGGER if DEBUGGER is SET

  #if SLIP_DEBUG
  DEBUGGER.begin(115200);
  #endif
  
  pinMode(LEDPIN,OUTPUT);



  //seed Pseudo random number generator
  randomSeed(analogRead(0));

}

void loop() {
  


 /*
  * |---------------------------- SETUP -------------------------------| 
  * 
  * 
  * git clone https://github.com/contiki-os/contiki.git
  * cd contiki/tools
  * make
  * sudo ./tunslip -B 115200 -s /dev/ttyUSB0 10.10.10.1 255.255.255.0
  * sudo ifconfig tun0 10.10.10.1/24
  * 
  * 
  * |------------------------------------------------------------------|
  * 
  */

  
 
  uint32_t destIP = (10UL<<24 | 10UL << 16 | 10UL << 8 | 1);



  /*
   * 
   * for UDP Client test, run -
   * nc -lup 9000
   * 
   * params:
   * 
   * 1 -> uint32_t destination IP
   * 2 -> uint16_t destination Port
   * 3 -> uint16_t source Port
   * 4 -> uint8_t* txdata
   * 5 -> uint8_t  txlen
   * 6 -> uint8_t* rxdata = data received
   * 7 -> uint8_t  rx_timeout in seconds [ 0 to send txdata and return immediately]
   * 
   * returns -> rxlen = length of data received
   * 
   */
  uint8_t rx[10];
  uint8_t rxlen=network.udpClient(destIP, 9000, 2000, (uint8_t*)"Hare Krishna Hare Rama", 22, rx,5);
  
  //print rxed data
  for(uint8_t i=0;i<rxlen;i++)Serial.write(rx[i]);
  Serial.println();
  

  /*
   * // for TCP client test run - 
   * python3 -m http.server 9000
   * 
   * params:
   * 
   * 1 -> uint32_t destination IP
   * 2 -> uint16_t destination Port
   * 3 -> uint16_t source Port
   * 4 -> uint8_t* txdata
   * 5 -> uint8_t  txlen
   * 6 -> callback function -> void cb(uint8_t *rxdata, uint8_t rxlen)
   * 7 -> uint8_t  rx_timeout in seconds [ 0 to send txdata and return immediately]
   * 
   */
   
  uint16_t random_src_port = random(1500, 10000);
  #define DATA "GET / HTTP/1.1\r\n\r\n"
  rxlen=network.tcpClient(destIP, 9000,random_src_port, (uint8_t*)DATA, String(DATA).length(), tcpRXcb ,5);


   /*
   * TCP Server
   * Can register only one TCP Server on one TCP port
   * 
   * Test -> 
   * curl 10.10.10.2:7000
   */
   network.tcpCBregister(7000, tcp_server);

   
  /*
   * UDP Servers
   * Can register upto 5 handlers for 5 UDP ports
   * 
   * Test ->
   * nc -u 10.10.10.2 5000
   * nc -u 10.10.10.2 6000
   */
  network.udpCBregister(5000, udp_echo_server);
  network.udpCBregister(6000, udp_handler_6000);




  /*
   * UDP and TCP Servers are handled inside the network.StateMachine()
   */
  while(1)network.stateMachine();



}
