
#include "motorControl.h"


motorControl::motorControl( uint8_t test ){
  _oldPosition = 0;
}

void motorControl::updateSpeed( double theSpeed){

    // long theSpeed = 100;

    //change the initial speed
    motor.setSpeed(theSpeed);
    delay(3000);
    //print the motor satus in the serial monitor
    motor.forward();
    delay(3000);
    //stop running
    motor.stop();
    delay(3000);
    //change the initial speed
    motor.setSpeed(theSpeed);
    //tell the motor to go back (may depend by your wiring)
    motor.backward();
    delay(3000);
    //stop running
    motor.stop();


}

void motorControl::getSpeed( void ){
    long newPosition = enc.read();

    if (newPosition != _oldPosition) {
      _oldPosition = newPosition;
      Serial.println(newPosition);
    }
}
