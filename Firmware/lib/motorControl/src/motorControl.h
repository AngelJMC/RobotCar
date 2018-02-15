//Servo - direction
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#ifndef motorControl_h
#define motorControl_h

#include "Encoder.h"
#include "L298N.h"
#include "PID_v1.h"

//pin definition
#define _EN  8
#define _IN1 7
#define _IN2 6

#define _INT1 2
#define _INT2 3

//Define Variables we'll be connecting to
static double _setPointMotor = 0, _input = 0, _output = 0;
//Specify the links and initial tuning parameters
static double Kp=0.4, Ki=0.2, Kd=0.01;
static PID pidMtr(&_input, &_output, &_setPointMotor, Kp, Ki, Kd, DIRECT);
static L298N driverMtr(_EN, _IN1, _IN2);
//Encoder encoderMtr(_INT1, _INT2);

//create a motor instance

class motorControl: public Encoder{

   public:
     motorControl( uint8_t test );
     void updateSpeed(uint32_t delay, int16_t reference);

   private:

     uint32_t _lastMs = 0;
     bool _canMove = 1;
     long _oldPosition;
     int32_t getSpeed( void );
};


#endif
