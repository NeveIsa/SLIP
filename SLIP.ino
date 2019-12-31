
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
  for(uint8_t i=0;i<rxlen;i++) Serial.write(rx[i]);
  memcpy(tx,rx,rxlen);
  return rxlen;
}

void setup() {
  // put your setup code here, to run once

  uint32_t x = 10;
  network.init(x << 24 | x << 16 | x << 8 | 2);

  Serial.begin(115200);
  //pinMode(13,OUTPUT);


  network.udpCBregister(5000, udp_handler_5000);
  //network.udpCBregister(6000, udp_handler_6000);

  network.tcpCBregister(7000, tcp_handler);

}

void loop() {
  // put your main code here, to run repeatedly:


  network.stateMachine();
  //network.writePacket();



}
