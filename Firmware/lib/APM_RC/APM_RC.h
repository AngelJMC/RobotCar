#ifndef __APM_RC_H__
#define __APM_RC_H__


#include <inttypes.h>


// Radio channels
#define NUM_CHANNELS 8
#define MIN_CHANNELS 5      // for ppm sum we allow less than 8 channels to make up a valid packet


class Arduino_Mega_ISR_Registry;

class APM_RC_Class

{

public:

    virtual void            Init( Arduino_Mega_ISR_Registry * isr_reg ) = 0;
    virtual uint16_t        InputCh(uint8_t ch) = 0;
    virtual uint8_t         GetState() = 0;
    virtual void            clearOverride(void) = 0;

    // get the time of the last radio update (_last_update modified by interrupt, so reading of variable must be interrupt safe)
    virtual uint32_t        get_last_update() {

        uint32_t _tmp = _last_update;
        while( _tmp != _last_update ) _tmp = _last_update;

        return _tmp;
    };

protected:
    uint16_t                _map_speed(uint16_t speed_hz) {
        return 2000000UL / speed_hz;
    }
    static volatile uint32_t         _last_update; // Modified by interrupt

};


#include "APM_PPMDecoder.h"

#endif //#define __APM_RC_H__
