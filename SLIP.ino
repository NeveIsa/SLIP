
#define LEDPIN 13


#include "slip_network.h"


slip network = slip(Serial2, 115200);


void udp_handler_5000()
{
  Serial.println("---> 5000");
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
  int senddatalen = 41+String(val).length();
  
  char csenddata[senddatalen+1];
  senddata.toCharArray(csenddata,senddatalen);
  memcpy(tx,csenddata,senddatalen);

  Serial.println(csenddata);
  static int LED_STATE=HIGH;
  LED_STATE = LED_STATE==HIGH?LOW:HIGH;
  digitalWrite(LEDPIN,LED_STATE);
  
  return senddatalen;
}

void setup() {
  // put your setup code here, to run once

  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  Serial.begin(115200);
  pinMode(LEDPIN,OUTPUT);


  network.udpCBregister(5000, udp_handler_5000);
  //network.udpCBregister(6000, udp_handler_6000);

  network.tcpCBregister(7000, tcp_handler);

}

void loop() {
  // put your main code here, to run repeatedly:


  network.stateMachine();
  //network.writePacket();



}
