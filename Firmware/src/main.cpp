#include "Arduino.h"
#include "Encoder.h"
#include "L298N.h"
#include "Servo.h"
#include "PID_v1.h"
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h> // ArduPilot Mega RC Library



//pin definition
#define _EN  8
#define _IN1 7
#define _IN2 6

#define _INT1 3
#define _INT2 2

#define SERVO_INPUT 11

#define _SENS_COEF 6
//Define Variables we'll be connecting to
double _setPointMotor = 0, _input = 0, _output = 0;
//Specify the links and initial tuning parameters
double Kp=1.2, Ki=0.7, Kd=0.001;

Arduino_Mega_ISR_Registry isr_registry;
APM_PPMDecoder APM_RC;

Servo servoSteering ;  // create servo object to control a servo
PID pidMtr(&_input, &_output, &_setPointMotor, Kp, Ki, Kd, DIRECT);
L298N driverMtr(_EN, _IN1, _IN2);
Encoder encoderMtr(_INT1, _INT2);

uint32_t _lastMs = 0;
bool _canMove = 1;
long _oldPosition;


uint8_t test = 1;
int16_t _SetpointMotor=0;
uint32_t _delayMs=100;

uint8_t servoVal;
int16_t speedVal;


String BufferSerialInput = "";         // a string to hold incoming data
String rcvNumberString = "";
boolean FlagBufferInput = false;  // whether the string is complete


int32_t getSpeed( void );
void updateSpeed(uint32_t delay, double reference);
void serialEvent();

void setup() {
  isr_registry.init();
  APM_RC.Init(&isr_registry);          // APM Radio initialization
  servoSteering.attach(SERVO_INPUT);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
  pidMtr.SetMode(AUTOMATIC);
  pidMtr.SetOutputLimits(-255, 255);
  // reserve 20 bytes for the inputString:
  BufferSerialInput.reserve(20);

}

void loop() {


	if (FlagBufferInput) {
	    Serial.println(BufferSerialInput);
	    //Process the received string
	    if(BufferSerialInput[0]=='#'){
	    	if(BufferSerialInput[1]=='S'){
	    		//read servo reference
	    		rcvNumberString = BufferSerialInput.substring(3,BufferSerialInput.length()-1);
	    		//Setpoint_Servo = rcvNumberString.toDouble();
	    		//Serial.println(Setpoint_Servo);
	    	}
	    	else if(BufferSerialInput[1]=='M'){
	    		rcvNumberString = BufferSerialInput.substring(3,BufferSerialInput.length()-1);
	    		speedVal = rcvNumberString.toDouble();
	    		Serial.println(speedVal);
	    	}
	    }

	    // clear the string:
	    BufferSerialInput = "";
	    FlagBufferInput = false;
	}

   if (APM_RC.GetState() == 1) {
      //  Serial.print("CH:");
      //  for(int i = 0; i < 4; i++) {
      //      Serial.print(APM_RC.InputCh(i));                    // Print channel values
      //      Serial.print(",");
      //  }
      //  Serial.println();

      servoVal = map(APM_RC.InputCh(0), 980, 2035, 180, 105);     // scale it to use it with the servo (value between 0 and 180)
      servoSteering.write(servoVal);                  // sets the servo position according to the scaled value
      speedVal = map(APM_RC.InputCh(1), 920, 2015, -255, 255);     // scale it to use it with the servo (value between 0 and 180)
   }

   updateSpeed(_delayMs, speedVal );

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

int32_t getSpeed( void ){

    int32_t newPosition = encoderMtr.read();
    int32_t diffCount = 0;

    if (newPosition != _oldPosition) {
      diffCount = newPosition - _oldPosition;
      _oldPosition = newPosition;
    }
return diffCount* _SENS_COEF;
}

void printInfoPID(){
  Serial.print(" encod: ");
  Serial.print(_oldPosition);
  Serial.print(" refe: ");
  Serial.print(_setPointMotor);
  Serial.print("  inp:");
  Serial.print(_input);
  Serial.print("  out:");
  Serial.print(_output);
  Serial.print("  control:");
  Serial.println(abs(_output));
}

void updateSpeed(uint32_t delay, double reference){

  if (((millis() - _lastMs) >= delay)) {
    _lastMs = millis();
    _setPointMotor = reference;

    if(reference > -10 && reference <10){
       _setPointMotor = 0;
    }
    else{
      _setPointMotor = reference;
    }

    _input = getSpeed();
    pidMtr.Compute();
    driverMtr.setSpeed(abs(_output));

    if(_output >=0){
      driverMtr.forward(); //forward
    }
    else{
      driverMtr.backward(); //BACKWARD
    }

    printInfoPID();
  }
}
