
#include "motorControl.h"


motorControl::motorControl( uint8_t test ){
  _oldPosition = 0;
  int16_t Output_min = -255;
  int16_t Output_max = 255;
  pidMtr.SetOutputLimits(Output_min,Output_max);
  pidMtr.SetMode(AUTOMATIC);
}


void motorControl::updateSpeed(uint32_t delay, int16_t reference){
  uint8_t pwmVal;

  if (((millis() - _lastMs) > delay)) {
    // _setPointMotor = reference;
     _input = this -> getSpeed();
    // pidMtr.Compute();
    // pwmVal = abs(_output);
    // driverMtr.setSpeed( pwmVal);
    Serial.print("refe: ");
    Serial.println(reference);
    // Serial.print("vel:");
    // Serial.println(_input);

if(reference >=0){
    //if(_output >=0){
      driverMtr.forward(); //forward
      driverMtr.setSpeed(abs( reference));
    }
    else{
      driverMtr.backward(); //BACKWARD
      driverMtr.setSpeed( abs(reference));
    }


    _lastMs = millis();

  }
}


int32_t motorControl::getSpeed( void ){

    // int32_t newPosition = encoderMtr.read();
     int32_t newPosition = 0;
    int32_t diffCount = 0;

    Serial.print("vel:");
    Serial.println(newPosition);

    if (newPosition != _oldPosition) {
      diffCount = newPosition - _oldPosition;
      _oldPosition = newPosition;
    }
return diffCount;
}
