
#define LEDPIN 13


#include "slip_network.h"


slip network = slip(Serial2, 115200);


uint8_t udp_echo_server(uint8_t *rx, uint8_t rxlen, uint8_t *tx)
{
  
  //echo
  memcpy(tx,rx,rxlen);
  return rxlen; 
}
/*void udp_handler_6000()
{
  Serial.println("---> 6000");
}*/


uint8_t tcp_handler(uint8_t *rx, uint8_t rxlen, uint8_t *tx)
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




void tcpRXcb(uint8_t *rxdata, uint8_t rxlen)
{
  for(uint8_t i=0;i<rxlen;i++)Serial.write(rxdata[i]);
}

void setup() {
  // put your setup code here, to run once

  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  Serial.begin(115200);
  pinMode(LEDPIN,OUTPUT);


  network.udpCBregister(5000, udp_echo_server);
  
  network.tcpCBregister(7000, tcp_handler);


  //seed Pseudo random number generator
  randomSeed(analogRead(0));

}

void loop() {
  // put your main code here, to run repeatedly:


  
  //network.writePacket();
  delay(100);
  uint8_t rx[10];
  uint32_t destIP = (10UL<<24 | 10UL << 16 | 10UL << 8 | 1);

  //destIP = 34UL <<24 | 193UL<<16 | 212UL<<8 | 251;
  //destIP = 54UL<<24 | 172UL<<16 | 95UL<<8 | 6;
  
  //uint8_t rxlen=network.udpClient(destIP, 9000, 2000, (uint8_t*)"Hare Krishna Hare Rama", 22, rx,0);

  //for(uint8_t i=0;i<rxlen;i++)Serial.write(rx[i]);
  //Serial.println();

  uint16_t random_src_port = random(1500, 10000);

  #define DATA "GET / HTTP/1.1\r\n\r\n"
  uint8_t rxlen=network.tcpClient(destIP, 9000, analogRead(A0), (uint8_t*)DATA, String(DATA).length(), tcpRXcb ,5);

  while(1)network.stateMachine();



}
