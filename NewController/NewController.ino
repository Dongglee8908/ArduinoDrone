#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Joystick
#define Joy1_PotX A0
#define Joy1_PotY A1
#define Joy1_Btn 9
#define Joy2_PotX A2
#define Joy2_PotY A3
#define Joy2_Btn 10
// Button
#define Joy_Btn1 2
#define Joy_Btn2 3
#define Joy_Btn3 4
#define Joy_Btn4 5

RF24 radio(8, 7); // CE, CSN

const uint8_t code[6] = "N4VeR";
float roll = 0, pitch = 0, acc[3];
char curButton[4] = {0,}, lastButton[4] = {0,};

struct Packet
{
  uint16_t J1PotX = 0;
  uint16_t J1PotY = 0;
  uint16_t J2PotX = 0;
  uint16_t J2PotY = 0;
  int16_t AngleX = 0;
  int16_t AngleY = 0;
  byte Button1 = 0;
  byte Button2 = 0;
  byte Button3 = 0;
  byte Button4 = 0;
} RFData;

void setup()
{
  Serial.begin(115200);
  // nrf24l01 set
  radio.begin();
  radio.openWritingPipe(code);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  // pinmode
  pinMode(Joy1_PotX, INPUT);
  pinMode(Joy1_PotY, INPUT);
  pinMode(Joy2_PotX, INPUT);
  pinMode(Joy2_PotY, INPUT);
  pinMode(Joy_Btn1, INPUT);
  pinMode(Joy_Btn2, INPUT);
  pinMode(Joy_Btn3, INPUT);
  pinMode(Joy_Btn4, INPUT);
  // mpu6050
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(); delay(100);
}

void loop()
{
  // Joystick
  RFData.J1PotX = analogRead(Joy1_PotX);
  RFData.J1PotY = analogRead(Joy1_PotY);
  RFData.J2PotX = analogRead(Joy2_PotX);
  RFData.J2PotY = analogRead(Joy2_PotY);
  // Select Button
  curButton[0] = digitalRead(Joy_Btn1);
  curButton[1] = digitalRead(Joy_Btn2);
  curButton[2] = digitalRead(Joy_Btn3);
  curButton[3] = digitalRead(Joy_Btn4);
  if(curButton[0] && curButton[0] != lastButton[0]) RFData.Button1 = 1;
  else RFData.Button1 = 0;
  if(curButton[1] && curButton[1] != lastButton[1]) RFData.Button2 = 1;
  else RFData.Button2 = 0;
  if(curButton[2] && curButton[2] != lastButton[2]) RFData.Button3 = 1;
  else RFData.Button3 = 0;
  if(curButton[3] && curButton[3] != lastButton[3]) RFData.Button4 = 1;
  else RFData.Button4 = 0;
  lastButton[0] = curButton[0];
  lastButton[1] = curButton[1];
  lastButton[2] = curButton[2];
  lastButton[3] = curButton[3];
  // Angle
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  acc[0] = Wire.read() << 8 | Wire.read();
  acc[1] = Wire.read() << 8 | Wire.read();
  acc[2] = Wire.read() << 8 | Wire.read();
  roll = roll * 0.9 + atan(-acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * RAD_TO_DEG * 0.1;
  pitch = pitch * 0.9 + atan(-acc[1] / sqrt(acc[0]*acc[0] + acc[2]*acc[2])) * RAD_TO_DEG * 0.1;
  RFData.AngleX = roll;
  RFData.AngleY = pitch;
  // Send Data
  radio.write(&RFData, sizeof(RFData));
}
