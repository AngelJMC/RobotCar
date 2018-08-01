

#ifndef ROBC_irsensors_h
#define ROBC_irsensors_h

typedef enum{
  AN0 = 0,
  AN1,
  AN2,
  AN3,
  AN4,
  AN5,
  AN6,
  AN7,
  AN8,
  AN9,
}analog_inputs_t;

class ROBC_IRsensors
{
public:

  ROBC_IRsensors( void);
  void init(const int, const analog_inputs_t *);
  void update( void );
  void print( void );
  int getIR( uint8_t ir);

private:
  typedef struct{
    analog_inputs_t irInputs;
    int ir_val;
  }ir_t;

  int     _nSensors;
  ir_t   * _irData;

};




#endif
