#ifndef __CLI_H__
#define __CLI_H__

#include "Arduino.h"
#include "control.h"

typedef struct{
  String BufferSerialInput;         // a string to hold incoming data
  String rcvNumberString;
  boolean FlagBufferInput;  // whether the string is complete
}cliHdlr_t;





void updateCLI( cliHdlr_t *cli, refValue_t *ref);

#endif //#define __CLI_H__
