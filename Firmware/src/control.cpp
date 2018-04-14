#include "control.h"


static int32_t _oldPosition = 0 ;

static int32_t _getSpeed( Encoder * encoderMtr);


static int32_t _getSpeed( Encoder * encoderMtr)
{
    int32_t newPosition = encoderMtr->read();
    int32_t diffCount = 0;

    if (newPosition != _oldPosition) {
      diffCount = newPosition - _oldPosition;
      _oldPosition = newPosition;
    }
return diffCount* _SENS_COEF;
}

void printInfoPID(const pidParameter_t *cval){
    // Serial.begin(115200);
  Serial.print(" encod: ");
  Serial.print(_oldPosition);
  Serial.print(" refe: ");
  Serial.print(cval->setPointMotor);
  Serial.print("  inp:");
  Serial.print(cval->input);
  Serial.print("  out:");
  Serial.print(cval->output);
  Serial.print("  control:");
  Serial.println(abs(cval->output));
}

void initControl(Servo * servoSteering, PID *pidMtr)
{
  servoSteering->attach(SERVO_INPUT);  // attaches the servo on pin 9 to the servo object
  pidMtr->SetMode(AUTOMATIC);
  pidMtr->SetOutputLimits(-255, 255);
}




void updateSpeed(pidParameter_t *cval, Encoder * encoderMtr, PID *pidMtr,
  L298N *driverMtr, refValue_t ref)
{
    if(ref.speedVal > -10 && ref.speedVal <10){
       cval->setPointMotor = 0;
    }
    else{
      cval->setPointMotor = ref.speedVal;
    }

    cval->input = _getSpeed(encoderMtr);
    pidMtr->Compute();
    driverMtr->setSpeed(abs(cval->output));

    if(cval->output >=0){
      driverMtr->forward(); //forward
    }
    else{
      driverMtr->backward(); //BACKWARD
    }
}

void updateSteering(Servo * servoSteering, refValue_t ref)
{
  servoSteering->write(ref.servoVal);
}
