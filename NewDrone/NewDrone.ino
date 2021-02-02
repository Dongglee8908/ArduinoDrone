#include "Wireless.h"
#include "Sensors.h"
#include "Motor.h"
// Drive & PID
float errorGyroI[3]={0,}, delta1[3]={0,}, delta2[3]={0,}, lastError[3]={0,};
const float rate_gain[3] = {
  8.3, // 8.0
  8.3, // 8.0
  4.0
};
const float pid_gain[3][3] = {
{1.3, 0.8, 38}, // 1.3 0.8 38 (/ 128)
{1.3, 0.8, 38}, // 1.3 0.8 38 (/ 128)
{4.0, 0.0, 6.0} // 4.0 0.0 6.0
};
const float max_control[3] = {
  200,
  200,
  100
};
uint32_t pdTime;
// Others
uint8_t Calibrate = 0, Start = 0;
uint32_t caliTime = 0, startTime = 0, connectTime = 0;

void setup() {
  Serial.begin(115200);
  initMotors();
  initWire();
  initAcc();
  initGyro();
  initRF();
  initBZ(4);
  onBuzzer(100);
}

void loop() {
  // Update
  UpdateRF();
  UpdateBZ();
  UpdateRAW();
  // Controller
  uint8_t axis = 0;
  if(Drive)
  {
    // PID Control
    float PTerm, ITerm, DTerm, errorRate, errorGyro, delta, Control[4];
    for(axis = 0; axis < 3; axis++)
    {
      // Correct the gyro value
      if(axis != YAW) gyroAngle[axis] += (LPF_Angle[axis] - AngleTrim[axis] - gyroAngle[axis]) / 1000;
      // Calculate the Error
      errorGyro = rcControl[axis+1] - gyroAngle[axis];
      errorRate = errorGyro - gyroRate[axis] * rate_gain[axis];
      // PTerm
      PTerm = errorRate * pid_gain[axis][0];
      // ITerm
      errorGyroI[axis] += errorRate * pid_gain[axis][1] / 256;
      errorGyroI[axis] = constrain(errorGyroI[axis], -max_control[axis], max_control[axis]);
      ITerm = errorGyroI[axis];
      // DTerm
      delta = errorRate - lastError[axis];
      lastError[axis] = errorRate;
      DTerm = (delta + delta1[axis] + delta2[axis]) * pid_gain[axis][2];
      delta2[axis] = delta1[axis];
      delta1[axis] = delta;
      // PID
      Control[axis] = PTerm + ITerm + DTerm;
      Control[axis] = constrain(Control[axis], -200, 200);
    }
    if (rcControl[0] > 50) writeMotors(rcControl[0], Control[0], Control[1], Control[2]);
    else
    {
      for(axis = 0; axis < 3; axis++) errorGyroI[axis] = 0;
      writeMotorsAll(1000);
    }
    // Drive Down
    uint32_t currentMillis = millis();
    if(RFData.J1PotY < 5 && RFData.J1PotX < 5 && rcControl[0] == 0)
    {
      if(Start == 0)
      {
        Start = 1;
        onBuzzer(50);
        startTime = currentMillis;
      }
      else if(Start == 1 && currentMillis - startTime > 2000)
      {
        Start = 0;
        for(axis = 0; axis < 4; axis++) rcControl[axis] = 0;
        Drive = false;
        onBuzzer(500);
        startTime = currentMillis;
      }
    }
    else if(Start) Start = 0;
  }
  else
  {
    uint32_t currentMillis = millis();
    // Start (Joy DOWN&IN)
    if(RFData.J1PotX > 1010 && RFData.J1PotY < 10 && RFData.J2PotX < 10 && RFData.J2PotY < 10)
    {
      if(Start == 0)
      {
        onBuzzer(50);
        Start = 1;
        startTime = currentMillis;
      }
      else if(Start == 1 && currentMillis - startTime > 1000)
      {
        Start = 2;
        onBuzzer(500);
        startTime = currentMillis;
      }
    }
    else if(Start == 2)
    {
      Start = 0;
      Drive = true;
      for(axis = 0; axis < 4; axis++) rcControl[axis] = 0;
    }
    else if(Start) Start = 0;
    // Calibration (Joy DOWN&OUT)
    if(RFData.J1PotX < 10 && RFData.J1PotY < 10 && RFData.J2PotX > 1010 && RFData.J2PotY < 10)
    {
      if(Calibrate == 0)
      {
        onBuzzer(50);
        Calibrate = 1;
        caliTime = currentMillis;
      }
      else if(Calibrate == 1 && currentMillis - caliTime > 1500)
      {
        onBuzzer(500);
        Calibrate = 2;
        caliTime = currentMillis;
        initGyro();
        for(axis = 0; axis < 3; axis++)
        {
          AngleTrim[axis] = LPF_Angle[axis];
          gyroAngle[axis] = 0;
        }
      }
    }
    else if(Calibrate) Calibrate = 0;
    writeMotorsAll(1000);
  }
  // Timeout the wireless rf24
  if(millis() - lastReceiveTime > 2000)
  {
    onBuzzer(100);
    if(Drive && millis() - connectTime > 50)
    {
      connectTime = millis();
      if(rcControl[0] > 0) rcControl[0] -= 1;
      else if(rcControl[0] == 0) Drive = false;
    }
  }
}
