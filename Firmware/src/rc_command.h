#ifndef __RC_COMMAND_H__
#define __RC_COMMAND_H__

#include "Arduino.h"
#include "Arduino_Mega_ISR_Registry.h"
#include "APM_RC.h" // ArduPilot Mega RC Library

typedef struct{
  uint8_t servoVal;
  int16_t speedVal;
}refValue_t;

void initRC(APM_PPMDecoder *rc, Arduino_Mega_ISR_Registry *isr_registry);
void updateRC( APM_PPMDecoder *rc, refValue_t *ref);


#endif //#define __RC_COMMAND_H__
