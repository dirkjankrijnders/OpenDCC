//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      keyboard.c
// author:    Wolfgang Kufer (kw)
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-05-08 V0.1 kw:started
// 
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: keyboard, just a dump port input
//
// content:   A DCC-Decoder for ATmega8515 and other AVR
//
//             1. General defines and variable definitions
//             2. Servo Engine
//                a) Defines
//                b) predefined Curves
//                c) data structures to handle servo movement
//                d) runtime engine
//             3. Init, Timertask
//
//------------------------------------------------------------------------

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/pgmspace.h>        // put var to program memory
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>

#include "config.h"             // uses timerval
#include "hardware.h"


#ifndef KEYBOARD_ENABLED 
  #define KEYBOARD_ENABLED   TRUE
#endif
#if (KEYBOARD_ENABLED == TRUE)

#define SIMULATION  0               // 0: real application
                                    // 1: test init routine
                                    // 2: test timing engine
                                    // 3: test action
#if (SIMULATION != 0)
   #warning SIMULATION is on!
#endif


//---------------------------------------------------------------------
// Timing Definitions:
// (all values given in us)

#define TICK_PERIOD       20000L    // 20ms tick for Timing Engine
                                    

//------------------------------------------------------------------------------
// internal, but static:

#ifndef KEY_PORT
#define KEY_PORT           PINA         // This is the port where we read bits
#endif

#ifndef KEY_MASK
#define KEY_MASK           0xFF         // the keyboard task runs only on
                                        // those ports defined in the mask
                                        // Ports not in the mask are ignored
#endif

#ifndef KEY_OFFSET
#define KEY_OFFSET         0            // shift keystroke result with this number
#endif

#define KEY_ACTIVE_TO_GND  TRUE         // TRUE: detect a keystroke if pin is grounded
                                        // FALSE: detect a keystroke if high is applied

unsigned char key_state;                // static - save the previous state of keys.

unsigned char debounce;                 // static - save a flag for "current in debounce"

signed char last_key_time;              // last timerval

//------------------------------------------------------------------------------
// code:

void init_keyboard(void)
  {
    debounce = 0;
    #if (KEY_ACTIVE_TO_GND == TRUE)
        key_state = ~KEY_PORT;  
    #else
        key_state = KEY_PORT;  
    #endif

    last_key_time = timerval;                    // remember time 
  }


//---------------------------------------------------------------------------
// keyboard(void)
// returns number corresponding to keystroke
//    0: Port (KEY_PORT), bit 0
//    1: Port (KEY_PORT), bit 1
//    ....
//    7: Port (KEY_PORT), bit 7
// 0xff: no keystroke detected
//
unsigned char keyboard_poll(void)
  {
    unsigned char keys, xor;

    if ((char)(timerval - last_key_time) < 1)  return(0xFF);

    last_key_time++;

    #if (KEY_ACTIVE_TO_GND == TRUE)
        keys = ~KEY_PORT;  
    #else
        keys = KEY_PORT;  
    #endif
    xor = keys ^ key_state;
    xor = xor & KEY_MASK;

    if (xor)
      {                         // something has changed
        unsigned char i, mask;

        mask = 1;
        for (i=0; i<8; i++)
          {
            if (xor & mask)
              {
                if (debounce & mask)
                  { // debounce over, now change state and report
                    debounce &= ~mask;
                    key_state ^= mask;

                    if (keys & mask) return(i);
                  }
                else
                  { 
                    debounce |= mask;
                  }
              }
            mask = mask << 1;           
          }
      }
    else
      {
        debounce = 0;
      }
    return(0xFF);
  }


unsigned char keyboard(void)
  {
    #if (KEY_OFFSET == 0)
        return(keyboard_poll());
    #else 
        unsigned mykey;
        mykey=keyboard_poll();
        if (mykey == 0xFF) return 0xFF;
        else return(mykey + KEY_OFFSET);
    #endif
  }

//-------------------------------------------------------------------
//
// Simulationen
//
//
#if (SIMULATION != 0)

unsigned char code;

void keytest(void)
  {
    init_keyboard();

    while(1)
      {
        code = keyboard();
        PORTB = code;
        code = keyboard();
        PORTB = code;
        code = keyboard();
        PORTB = code;
        timerval++;
      }
  }
#endif

#endif // (KEYBOARD_ENABLED == TRUE)




