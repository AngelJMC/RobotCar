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
double Setpoint_Servo,Setpoint_Motor, Input, Output, Output_min, Output_max,Output_Motor;

//Specify the links and initial tuning parameters
double Kp=0.4, Ki=0.2, Kd=0.01;
PID PID_Direction(&Input, &Output, &Setpoint_Servo, Kp, Ki, Kd, DIRECT);

String BufferSerialInput = "";         // a string to hold incoming data
String rcvNumberString = "";
boolean FlagBufferInput = false;  // whether the string is complete


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);


   Setpoint_Servo = 450;
   Setpoint_Motor = 0;

  //turn the PID on
  Output_min = -255;
  Output_max = 255;
  PID_Direction.SetOutputLimits(Output_min,Output_max);
  PID_Direction.SetMode(AUTOMATIC);

  // reserve 20 bytes for the inputString:
  BufferSerialInput.reserve(20);

}

void loop() {

  //delay(500);

	if (FlagBufferInput) {
	    Serial.println(BufferSerialInput);
	    //Process the received string
	    if(BufferSerialInput[0]=='#'){
	    	if(BufferSerialInput[1]=='S'){
	    		//read servo reference
	    		rcvNumberString = BufferSerialInput.substring(3,BufferSerialInput.length()-1);
	    		Setpoint_Servo = rcvNumberString.toDouble();
	    		//Serial.println(Setpoint_Servo);
	    	}
	    	else if(BufferSerialInput[1]=='M'){
	    		rcvNumberString = BufferSerialInput.substring(3,BufferSerialInput.length()-1);
	    		Setpoint_Motor = rcvNumberString.toDouble();
	    		//Serial.println(Setpoint_Motor);
	    	}
	    }

	    // clear the string:
	    BufferSerialInput = "";
	    FlagBufferInput = false;
	}


	//ControlServo();
	//ControlMotor();



}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    BufferSerialInput += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      FlagBufferInput = true;
    }
  }
}

void ControlServo(){
	Input = analogRead(SERVO_POT);
	if(PID_Direction.Compute()){
	  Serial.print(Setpoint_Servo);
	  Serial.print("  -  ");
	  Serial.println(Input);

	  //If PID if computed
	  if(Output>=0){
		  digitalWrite(IN3, HIGH);
		  digitalWrite(IN4, LOW);
	  }
	  else{
		  digitalWrite(IN3, LOW);
		  digitalWrite(IN4, HIGH);

	  }

	  analogWrite(ENB,abs(Output));
	  //ControlMotor();
	}
}

void ControlMotor(){

//If PID if computed
	Output_Motor = Setpoint_Motor;
	Serial.println(Setpoint_Motor);

	  if(Output_Motor>=0){
		  digitalWrite(IN1, HIGH);
		  digitalWrite(IN2, LOW);
	  }
	  else{
		  digitalWrite(IN1, LOW);
		  digitalWrite(IN2, HIGH);

	  }

	  analogWrite(ENA,abs(Output_Motor));

}
