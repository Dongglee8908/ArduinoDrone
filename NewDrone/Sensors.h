#ifndef SENSORS_H
#define SENSORS_H
#include <Wire.h>
#define ROLL  0
#define PITCH 1
#define YAW   2

/* Wire */
void initWire()
{
  // Start I2C
  Wire.begin();
  // Set Clock
  Wire.setClock(400000); delay(5);
}

/* Buzzer */
bool bzState = false;
uint8_t BZ_PIN = 4;
uint16_t onTime = 0;
uint32_t bzTime = 0;

void initBZ(uint8_t pin = 4)
{
  pinMode(BZ_PIN = pin, OUTPUT);
}

void onBuzzer(uint16_t t)
{
  bzTime = millis();
  bzState = true;
  onTime = t;
}

void UpdateBZ()
{
  if(bzState && millis() - bzTime > onTime) bzState = false;
  digitalWrite(BZ_PIN, bzState);
}

/* MPU6050 */
float accSmooth[3]={0,}, LPF_Angle[3]={0,}, AngleTrim[3]={16.05,12.75,0}, dt=0;
float gyroData[3]={0,}, gyroBase[3]={0,}, gyroAngle[3]={0,}, gyroRate[3]={0,};
uint32_t dtMicros = 0;

void initAcc()
{
  // Wake up the mpu6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Reg 107
  Wire.write(0x00); 
  Wire.endTransmission(true); delay(5);
  // set gyro scale (2000dps)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Reg 27
  Wire.write(0xF8); // 0b11111000
  Wire.endTransmission(true);
  // set acc scale (2g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Reg 28
  Wire.write(0xE0); // 0b11100000
  Wire.endTransmission(true);
}

void initGyro()
{
  gyroBase[0] = gyroBase[1] = gyroBase[2] = 0;
  for(uint8_t i = 0; i < 20; i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    gyroBase[0] += Wire.read() << 8 | Wire.read(); // gx
    gyroBase[1] += Wire.read() << 8 | Wire.read(); // gy
    gyroBase[2] += Wire.read() << 8 | Wire.read(); // gz
    delay(3);
  }
  gyroBase[0] /= 20;
  gyroBase[1] /= 20;
  gyroBase[2] /= 20;
}

void UpdateRAW()
{
  uint8_t axis;
  int16_t acc[3], gyro[3], temp;
  uint32_t currentMicros = micros(), currentMillis = millis();
  float vector_sum, deg0, deg1, deg2;
  dt = (currentMicros - dtMicros) * 0.000001F; dtMicros = currentMicros;
  // read data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(0x68, 14, true); currentMicros = micros();
  while(Wire.available() < 14 && micros() - currentMicros < 20);
  acc[0] = (Wire.read() << 8) | Wire.read();
  acc[1] = (Wire.read() << 8) | Wire.read();
  acc[2] = (Wire.read() << 8) | Wire.read();
  temp = (Wire.read() << 8) | Wire.read();
  gyro[1] = +(((Wire.read() << 8) | Wire.read()) - gyroBase[0]);
  gyro[0] = -(((Wire.read() << 8) | Wire.read()) - gyroBase[1]);
  gyro[2] = -(((Wire.read() << 8) | Wire.read()) - gyroBase[2]);
  // calc smooth
  for(axis = 0; axis < 3; axis++) accSmooth[axis] = accSmooth[axis] * 0.996 + acc[axis] * 0.004;
  // calc vector&degree
  vector_sum = sqrt(accSmooth[0]*accSmooth[0] + accSmooth[1]*accSmooth[1] + accSmooth[2]*accSmooth[2]);
  for(axis = 0; axis < 3; axis++)
  {
    // Gyro LPF
    gyroData[axis] = gyroData[axis] * 0.8  + gyro[axis] * 0.2 / 16.4;
    gyroRate[axis] = gyroData[axis] * dt;
    gyroAngle[axis] += gyroRate[axis];
  }
  LPF_Angle[0] = (LPF_Angle[0] + gyroRate[0]) * 0.8 + atan(accSmooth[0] / vector_sum) * RAD_TO_DEG * 0.2;
  LPF_Angle[1] = (LPF_Angle[1] + gyroRate[1]) * 0.8 + atan(accSmooth[1] / vector_sum) * RAD_TO_DEG * 0.2;
}

#endif
