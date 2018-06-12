
#include "motorControl.h"

#define _SENS_COEF    6


void motorControl::initialize( void )
{

  _initialised = true;
  _pidParam.oldPosition = 0;
  _pidParam.setPointMotor = 0;
  _pidParam.input = 0;
  SetOutputLimits(_OUTPUT_MIN, _OUTPUT_MAX );
  SetMode(AUTOMATIC);
}


void motorControl::updateSpeed( int16_t speedVal ){
    if( speedVal > -10 && speedVal <10){
       _pidParam.setPointMotor = 0;
    }
    else{
      _pidParam.setPointMotor = speedVal;
    }

    _pidParam.input = _getSpeed( );
    Compute();
    setSpeed(abs(_pidParam.output));

    if(_pidParam.output >=0){
      forward(); //forward
    }
    else{
      backward(); //BACKWARD
    }
}


void motorControl::printInfoPID( void)
{
    Serial.print(" encod: ");
    Serial.print(_pidParam.oldPosition);
    Serial.print(" refe: ");
    Serial.print(_pidParam.setPointMotor);
    Serial.print("  inp:");
    Serial.print(_pidParam.input);
    Serial.print("  out:");
    Serial.print(_pidParam.output);
    Serial.print("  control:");
    Serial.println(abs(_pidParam.output));
}



int32_t motorControl::_getSpeed( void )
{
    // int32_t newPosition = encoderMtr.read();
    int32_t newPosition = read();
    int32_t diffCount = 0;

    if (newPosition != _pidParam.oldPosition) {
      diffCount = newPosition - _pidParam.oldPosition;
      _pidParam.oldPosition = newPosition;
    }
return diffCount* _SENS_COEF;
}
