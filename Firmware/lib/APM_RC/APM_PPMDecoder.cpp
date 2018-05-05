/*
 *       APM_PPMDecoder.cpp - Radio Control Library for Ardupilot Mega 2.0. Arduino
 *       Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       RC Input : PPM signal on IC4 pin
 *
 *       Methods:
 *               Init() : Initialization of interrupt an Timer
 *               InputCh(ch) : Read a channel input value.  ch=0..7
 *               GetState() : Returns the state of the input. 1 => New radio frame to process
 *                            Automatically resets when we call InputCh to read channels
 *
 */
#include "APM_PPMDecoder.h"

#include <avr/interrupt.h>
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(DESKTOP_BUILD)
 # error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint16_t APM_PPMDecoder::_PWM_RAW[NUM_CHANNELS] = {2400,2400,2400,2400,2400,2400,2400,2400};
volatile uint8_t APM_PPMDecoder::_radio_status=0;

/****************************************************
*   Input Capture Interrupt ICP5 => PPM signal read
****************************************************/
void APM_PPMDecoder::_timer5_capt_cb(void)
{
    static uint16_t prev_icr;
    static uint8_t frame_idx;
    uint16_t icr;
    uint16_t pwidth;

    icr = ICR5;
    // Calculate pulse width assuming timer overflow TOP = 40000
    if ( icr < prev_icr ) {
        pwidth = ( icr + 40000 ) - prev_icr;
    } else {
        pwidth = icr - prev_icr;
    }

    // Was it a sync pulse? If so, reset frame.
    if ( pwidth > 8000 ) {
        // pass through values if at least a minimum number of channels received
        if( frame_idx >= MIN_CHANNELS ) {
            _radio_status = 1;
            _last_update = millis();
        }
        frame_idx=0;
    } else {
        // Save pulse into _PWM_RAW array.
        if ( frame_idx < NUM_CHANNELS ) {
            _PWM_RAW[ frame_idx++ ] = pwidth;
        }
    }
    // Save icr for next call.
    prev_icr = icr;
}


// Constructors ////////////////////////////////////////////////////////////////

APM_PPMDecoder::APM_PPMDecoder()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_PPMDecoder::Init( Arduino_Mega_ISR_Registry * isr_reg )
{
    //--------------- TIMER5: PPM INPUT  ---------------
    // Init PPM input on Timer 5
    pinMode(48, INPUT); // PPM Input (PL1/ICP5)
    // WGM: 1 1 1 1. Fast PWM, TOP is OCR5A
    // COM all disabled.
    // CS51: prescale by 8 => 0.5us tick
    // ICES5: Input Capture on rising edge
    TCCR5A =((1<<WGM50)|(1<<WGM51));
    // Input Capture rising edge
    TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5));
    OCR5A = 40000; // 0.5us tick => 50hz freq. The input capture routine
                   // assumes this 40000 for TOP.

    isr_reg->register_signal( ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb );
    // Enable Input Capture interrupt
    TIMSK5 |= (1<<ICIE5);
}


uint16_t APM_PPMDecoder::InputCh(unsigned char ch)
{
    uint16_t result;

    if (_HIL_override[ch] != 0) {
        return _HIL_override[ch];
    }

    // We need to block ICP5 interrupts during the read of 16 bit PWM values
    uint8_t _timsk5 = TIMSK5;
    TIMSK5 &= ~(1<<ICIE5);

    // value
    result = _PWM_RAW[ch];

    // Enable ICP5 interrupt if previously active
    TIMSK5 = _timsk5;

    // Because timer runs at 0.5us we need to do value/2
    result >>= 1;

    // Limit values to a valid range
    result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
    _radio_status = 0;     // Radio channel read
    return result;
}

unsigned char APM_PPMDecoder::GetState(void)
{
    return(_radio_status);
}


// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_PPMDecoder::setHIL(int16_t v[NUM_CHANNELS])
{
    uint8_t sum = 0;
    for (unsigned char i=0; i<NUM_CHANNELS; i++) {
        if (v[i] != -1) {
            _HIL_override[i] = v[i];
        }
        if (_HIL_override[i] != 0) {
            sum++;
        }
    }
    _radio_status = 1;
    _last_update = millis();
    if (sum == 0) {
        return 0;
    } else {
        return 1;
    }
}

void APM_PPMDecoder::clearOverride(void)
{
    for (unsigned char i=0; i<NUM_CHANNELS; i++) {
        _HIL_override[i] = 0;
    }
}

#endif
