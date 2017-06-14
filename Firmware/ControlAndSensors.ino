#include "Arduino.h"
//#include <PID_v1.h>
#include "PID_v1.h"
//Conection

//Servo - direction

//Analog pot ->  A0
//l298N ENB  -> D7
//l298N IN3  -> D6
//l298N IN4  -> D3
#define SERVO_POT A0
#define MAX_VALUE_SERVO 620
#define MIN_VALUE_SERVO 260
#define ENB  7
#define IN3  6
#define IN4  3


// Motor DC
//l298N ENA  -> D12
//l298N IN1  -> D11
//l298N IN2  -> D8
#define ENA  12
#define IN1  11
#define IN2  8

//Define Variables we'll be connecting to
double Setpoint, Input, Output, Output_min, Output_max;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID PID_Direction(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);


   Setpoint = 100;

  //turn the PID on
  Output_min = -255;
  Output_max = 255;
  PID_Direction.SetOutputLimits(Output_min,Output_max);
  PID_Direction.SetMode(AUTOMATIC);

}

void loop() {

  delay(500);
  Serial.println(analogRead(SERVO_POT));

  //digitalWrite(IN3, LOW);
  Input = analogRead(SERVO_POT);
  PID_Direction.Compute();

  /*digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB,200);
  delay(500);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB,200);
  delay(500); */

  /*digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite(ENA,55);
  delay(2000);
  analogWrite(ENA,105);
  delay(2000);
  analogWrite(ENA,255);
  delay(2000);
  analogWrite(ENA,10);
  delay(2000);

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite(ENA,50);
  delay(2000);
  analogWrite(ENA,200);
  delay(2000);*/

}

