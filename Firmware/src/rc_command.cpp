#include "rc_command.h"


void initRC(APM_PPMDecoder *rc, Arduino_Mega_ISR_Registry *isr_registry){
  isr_registry->init();
  rc->Init(isr_registry);          // APM Radio initialization
}

void updateRC( APM_PPMDecoder *rc, refValue_t *ref){

     //  Serial.print("CH:");
     //  for(int i = 0; i < 4; i++) {
     //      Serial.print(APM_RC.InputCh(i));                    // Print channel values
     //      Serial.print(",");
     //  }
     //  Serial.println();

     ref->servoVal = map(rc->InputCh(0), 980, 2035, 180, 105);     // scale it to use it with the servo (value between 0 and 180)
     ref->speedVal = map(rc->InputCh(1), 920, 2015, -255, 255);     // scale it to use it with the servo (value between 0 and 180)
}
