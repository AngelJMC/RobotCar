#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "Arduino.h"
#include "Servo.h"
#include "Encoder.h"
#include "L298N.h"
#include "PID_v1.h"

#define SERVO_INPUT 11

#define _SENS_COEF 6
//Define Variables we'll be connecting to

typedef struct{
  double setPointMotor;
  double input;
  double output;
  double Kp;
  double Ki;
  double Kd;
}pidParameter_t;


typedef struct{
  uint8_t servoVal;
  int16_t speedVal;
}refValue_t;


void updateSpeed(pidParameter_t *cval, Encoder * encoderMtr, PID *pidMtr,
  L298N *driverMtr, refValue_t ref);
void updateSteering(Servo * servoSteering, refValue_t ref);
void printInfoPID(const pidParameter_t *cval);
void initControl(Servo * servoSteering, PID *pidMtr);

#endif //#define __CONTROL_H__
