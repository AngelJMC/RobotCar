#include "Arduino.h"
#include <ROBC_IRsensors.h>

ROBC_IRsensors::ROBC_IRsensors( void)
{
  _irData = NULL;
}

void ROBC_IRsensors::init(const int irN, const analog_inputs_t *irInputs)
{

  _nSensors = irN;
  _irData = new ir_t[_nSensors];

for(int ir=0; ir<irN; ++ir){
  _irData[ir].irInputs = irInputs[ir];

}
}


void ROBC_IRsensors::update( void )
{
  for( int ir=0; ir<_nSensors; ++ir){
      _irData[ir].ir_val = analogRead( _irData[ir].irInputs);

  }

}


void ROBC_IRsensors::print( void )
{
  for( int ir=0; ir<_nSensors; ++ir){
    Serial.print(_irData[ir].ir_val);
    Serial.print(", ");
  }
  Serial.print("\r\n");
}
