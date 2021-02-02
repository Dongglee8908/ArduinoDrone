#ifndef WIRELESS_H
#define WIRELESS_H
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
RF24 Radio(8, 7);

bool Drive = false;
uint8_t pipe[6] = "N4VeR";
int16_t rcControl[4] = {0,};
uint32_t lastReceiveTime, controlTime;

struct Packet
{
  uint16_t J1PotX = 0;
  uint16_t J1PotY = 0;
  uint16_t J2PotX = 0;
  uint16_t J2PotY = 0;
  uint16_t AngleX = 0;
  uint16_t AngleY = 0;
  byte Button1 = 0;
  byte Button2 = 0;
  byte Button3 = 0;
  byte Button4 = 0;
} RFData;

void initRF()
{
  Radio.begin();
  Radio.openReadingPipe(0, pipe);
  Radio.setPALevel(RF24_PA_LOW);
  Radio.startListening();
}

void UpdateRF()
{
  uint32_t currentMillis = millis();
  if(Radio.available())
  {
    lastReceiveTime = currentMillis;
    Radio.read(&RFData, sizeof(RFData));
    if(Drive)
    {
      rcControl[0] = constrain(RFData.J1PotY / 2.1, 0, 500);
      rcControl[1] = constrain((int16_t)(RFData.J2PotX >> 6) - 8, -7, 7);
      rcControl[2] = constrain((int16_t)(RFData.J2PotY >> 6) - 7, -7, 7);
      if(currentMillis - controlTime > 50)
      {
        controlTime = currentMillis;
        if(rcControl[0] > 50)
        {
          rcControl[3] += (RFData.J1PotX > 600) ? 2:0;
          rcControl[3] -= (RFData.J1PotX < 400) ? 2:0;
        }
      }
    }
  }
}

#endif
