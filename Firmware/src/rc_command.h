#ifndef __RC_COMMAND_H__
#define __RC_COMMAND_H__

#include "Arduino.h"
#include "control.h"
#include "Arduino_Mega_ISR_Registry.h"
#include "APM_RC.h" // ArduPilot Mega RC Library

void initRC(APM_PPMDecoder *rc, Arduino_Mega_ISR_Registry *isr_registry);
void updateRC( APM_PPMDecoder *rc, refValue_t *ref);


#endif //#define __RC_COMMAND_H__
