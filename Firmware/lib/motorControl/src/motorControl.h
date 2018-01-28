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

//pin definition
#define _EN  9
#define _IN1 8
#define _IN2 7

#define _INT1 5
#define _INT2 6

static L298N motor(_EN, _IN1, _IN2);
static Encoder enc(_INT1, _INT2);
//create a motor instance

class motorControl{
   public:
     motorControl( uint8_t test );
     void updateSpeed( double theSpeed );
   private:
     long _oldPosition;
     void getSpeed( void );
};


#endif
