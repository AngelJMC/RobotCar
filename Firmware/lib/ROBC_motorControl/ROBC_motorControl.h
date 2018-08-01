

#ifndef ROBC_motorControl_h
#define ROBC_motorControl_h


#include "Encoder.h"
#include "L298N.h"
#include "PID_v1.h"

//pin definition
#define _EN  8
#define _IN1 7
#define _IN2 6

#define _INT1 2
#define _INT2 3

#define _OUTPUT_MIN   -100
#define _OUTPUT_MAX   100

struct pidData_s {

  double  setPointMotor;
  double  input;
  double  output;
  long    oldPosition;

};


class motorControl: public L298N, public Encoder, public PID{

public:

  motorControl(double Kp, double Ki, double Kd):

      L298N(_EN, _IN1, _IN2),
      Encoder(_INT2, _INT1),
      PID(&_pidParam.input, &_pidParam.output , &_pidParam.setPointMotor,
        Kp, Ki, Kd, DIRECT){};


  motorControl( uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2,
                double Kp, double Ki, double Kd, uint8_t pin1, uint8_t pin2):

      L298N(pinEnable, pinIN1, pinIN2),
      Encoder( pin1, pin2),
      PID( &_pidParam.input , &_pidParam.output, &_pidParam.setPointMotor,
          Kp, Ki, Kd, DIRECT){};


  motorControl( uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2,
                double* input, double* output, double* setPointMotor, double Kp,
                double Ki, double Kd, int type, uint8_t pin1, uint8_t pin2):

      L298N(pinEnable, pinIN1, pinIN2),
      Encoder( pin1, pin2),
      PID( input , output, setPointMotor, Kp, Ki, Kd, type){};

  void updateSpeed( int16_t speedVal );
  void printInfoPID( void );
  void initialize( void );
  long getCountsEncoder( void );
  double getOutputPID( void );
  double getInputPID( void );


private:
   pidData_s               _pidParam;
   // ensure we can't initialize twice
   bool                    _initialised;

   int32_t  _getSpeed( void );

};




#endif
