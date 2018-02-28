#include "Arduino.h"
#include "Encoder.h"
#include "L298N.h"
#include "Servo.h"
#include "PID_v1.h"
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h> // ArduPilot Mega RC Library
#include "control.h"
#include "cli.h"
#include "rc_command.h"

#define _EN  8
#define _IN1 7
#define _IN2 6

#define _INT1 3
#define _INT2 2


typedef struct{
  uint32_t motor = 0;
  uint32_t steering = 0;
}delaysMs_t;

pidParameter_t _cval={
  .setPointMotor = 0,
  .input = 0,
  .output = 0,
  .Kp = 1.2,
  .Ki = 0.7,
  .Kd = 0.001,
};

refValue_t refVal;

cliHdlr_t _cliHdlr={
  .BufferSerialInput = "",
  .rcvNumberString = "",
  .FlagBufferInput = false,  // whether the string is complete
};

delaysMs_t lastMs;

Arduino_Mega_ISR_Registry _isr_registry;
APM_PPMDecoder _rcPPM;
Servo servoSteering ;  // create servo object to control a servo
PID pidMtr(&_cval.input, &_cval.output , &_cval.setPointMotor, _cval.Kp, _cval.Ki, _cval.Kd, DIRECT);
L298N driverMtr(_EN, _IN1, _IN2);
Encoder encoderMtr(_INT1, _INT2);

void serialEvent();

void setup() {
  initRC(&_rcPPM, &_isr_registry);
  Serial.begin(115200);
  initControl(&servoSteering,&pidMtr);
  _cliHdlr.BufferSerialInput.reserve(20);   // reserve 20 bytes for the inputString:

}

void loop() {

  if (_cliHdlr.FlagBufferInput) {
    updateCLI( &_cliHdlr, &refVal);
  }

  if (_rcPPM.GetState() == true ) {
    updateRC( &_rcPPM, &refVal);
  }

  if ((millis() - lastMs.motor) >= 100) {
    lastMs.motor = millis();
    updateSpeed(&_cval, &encoderMtr, &pidMtr, &driverMtr, refVal);
    printInfoPID(&_cval);
  }

  if ((millis() - lastMs.steering) >= 10){
    lastMs.steering = millis();
    updateSteering( &servoSteering, refVal);
  }
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
    _cliHdlr.BufferSerialInput += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      _cliHdlr.FlagBufferInput = true;
    }
  }
}
