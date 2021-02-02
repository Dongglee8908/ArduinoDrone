#ifndef MOTOR_H
#define MOTOR_H
#include "Servo.h"
// FRONT
//  0 3
//   X
//  1 2

const uint8_t PWM_PIN[4] = {5, 6, 9, 10};
Servo motor[4];

// 1000:2000
void writeMotor(uint8_t n, uint16_t c)
{
  motor[n].writeMicroseconds(c);
}

void writeMotors(float throttle, float roll, float pitch, float yaw)
{
  writeMotor(0, constrain(throttle + roll - pitch + yaw, 0, 1000) + 1000);
  writeMotor(1, constrain(throttle + roll + pitch - yaw, 0, 1000) + 1000);
  writeMotor(2, constrain(throttle + -roll + pitch + yaw, 0, 1000) + 1000);
  writeMotor(3, constrain(throttle + -roll - pitch - yaw, 0, 1000) + 1000);
}

void writeMotorsAll(uint16_t c)
{
  writeMotor(0, c);
  writeMotor(1, c);
  writeMotor(2, c);
  writeMotor(3, c);
}

void initMotors()
{
  for(uint8_t i = 0; i < 4; i++) motor[i].attach(PWM_PIN[i], 1000, 2000);
  writeMotorsAll(1000);
  delay(100);
}

#endif
