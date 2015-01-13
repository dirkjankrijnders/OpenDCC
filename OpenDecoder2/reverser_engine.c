//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2010 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      reverser_engine.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2010-09-14 V0.01 kw start
//
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: timing engine for outputs
//                  including feedback reading and local led flashing
//
// content:   A DCC-Decoder for ATmega8515 and other AVR
//
//             1. Defines and variable definitions
//             2. Init
//             3. ISR
//             4. Predefined values
//             5. action - performing the Command from the DCC call
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

#include "config.h"
#include "myeeprom.h"            // wrapper for eeprom
#include "hardware.h"
#include "dcc_receiver.h"
#include "main.h"
#include "port_engine.h"

#define SIMULATION  0            // 0: real application
                                 // 1: test receive routine
                                 // 2: test timing engine
                                 // 3: test action

//=============================================================================
// 1. Definitions
//=============================================================================
//
// Output:
// Reverser Engine does the control of the outputs only if REVERSER_ENABLED is true
//
#ifndef REVERSER_ENABLED
  #warning REVERSER_ENABLED not defined, now enabled by default (should be done in config.h)
  define REVERSER_ENABLED  TRUE
#endif

#if (REVERSER_ENABLED == TRUE)

#if (PORT_ENABLED != TRUE)
  #error we need PORT_ENABLED, we rely on port_engine
#endif

unsigned char logical_output;  // here we save the current 'logical' position of turnouts

unsigned char k1_trigger1_normal;  // trigger mask (read from eeprom)
unsigned char k1_trigger2_normal;
unsigned char k1_trigger1_invers;
unsigned char k1_trigger2_invers;
unsigned char k2_trigger1_normal;
unsigned char k2_trigger2_normal;
unsigned char k2_trigger1_invers;
unsigned char k2_trigger2_invers;


//------------------------------------------------------------------------------
// helper function
static void output(unsigned char port_no, unsigned char state)
  {
    if (state == 0)
      {                              
        OUTPUT_PORT &= (~(1<<port_no));
      }
    else
      {
        OUTPUT_PORT |= (1<<port_no);
      }
  }
                 
static inline void disable_timer_interrupt(void)   __attribute__((always_inline));
void disable_timer_interrupt(void)
  {
      TIMSK &= ~(1<<TOIE1);        // Timer1 Overflow
  }

static inline void enable_timer_interrupt(void)   __attribute__((always_inline));
void enable_timer_interrupt(void) 
  {
      TIMSK |= (1<<TOIE1);        // Timer1 Overflow
  }


//------------------------------------------------------------------------------
// Support routines to control relays and LED

static void  trigger_k1_normal(void)
  {
     disable_timer_interrupt(); 
     // OUT7: pulse, OUT6 off, OUT3 on, OUT2 off
     out_pwm[7].rest = 3; // pulse_duration; 
     output(PB7,1);        
     out_pwm[6].rest = 0;
     output(PB6,0);        
     out_pwm[3].rest = 0;
     output(PB3,1);        
     out_pwm[2].rest = 0;
     output(PB2,0);        
     enable_timer_interrupt(); 
  }

static void  trigger_k1_invers(void)
  {
     disable_timer_interrupt(); 
     // OUT7 off, OUT6: pulse, OUT3 off, OUT2 on
     out_pwm[7].rest = 0;
     output(PB7,0);        
     out_pwm[6].rest = 3; // pulse_duration; 
     output(PB6,1);        
     out_pwm[3].rest = 0;
     output(PB3,0);        
     out_pwm[2].rest = 0;
     output(PB2,1);        
     enable_timer_interrupt(); 
  }

static void  trigger_k2_normal(void)
  {
     disable_timer_interrupt(); 
     // OUT5: pulse, OUT4 off, OUT0 on, OUT1 off
     out_pwm[5].rest = 3; // pulse_duration; 
     output(PB5,1);        
     out_pwm[4].rest = 0;
     output(PB4,0);        
     out_pwm[0].rest = 0;
     output(PB0,1);        
     out_pwm[1].rest = 0;
     output(PB1,0);        
     enable_timer_interrupt(); 
  }

static void trigger_k2_invers(void)
  {
     disable_timer_interrupt(); 
     // OUT5 off, OUT4: pulse, OUT0 off, OUT1 on
     out_pwm[5].rest = 0;
     output(PB5,0);        
     out_pwm[4].rest = 3; // pulse_duration; 
     output(PB4,1);        
     out_pwm[0].rest = 0;
     output(PB0,0);        
     out_pwm[1].rest = 0;
     output(PB1,1);        
     enable_timer_interrupt(); 
  }

static void check_relais_k1(unsigned char test)
  {
    if ((test & k1_trigger1_normal) == test)
      {
         trigger_k1_normal();
         return;
      }
    if ((test & k1_trigger2_normal) == test)
      {
         trigger_k1_normal();
         return;
      }
    if ((test & k1_trigger1_invers) == test)
      {
         trigger_k1_invers();
         return;
      }
    if ((test & k1_trigger2_invers) == test)
      {
         trigger_k1_invers();
         return;
      }
  }

static void check_relais_k2(unsigned char test)
  {
    if ((test & k2_trigger1_normal) == test)
      {
         trigger_k2_normal();
         return;
      }
    if ((test & k2_trigger2_normal) == test)
      {
         trigger_k2_normal();
         return;
      }
    if ((test & k2_trigger1_invers) == test)
      {
         trigger_k2_invers();
         return;
      }
    if ((test & k2_trigger2_invers) == test)
      {
         trigger_k2_invers();
         return;
      }
  }


void reverser_action(unsigned int Command, unsigned char Activate)
  {
    unsigned char myCommand;
    unsigned char temp;
    myCommand = Command & 0b00000111;
    
    if (Command > 7) return;                    // not our Address
        
    temp = logical_output;
    
    if (Activate)                               //  activate coil?
      {
        switch(myCommand)
          {
            default: break;
            case 0:
                temp |= 0x01;
                temp &= ~0x02;
                break;
            case 1:
                temp |= 0x02;
                temp &= ~0x01;
                break;
            case 2:
                temp |= 0x04;
                temp &= ~0x08;
                break;
            case 3:
                temp |= 0x08;
                temp &= ~0x04;
                break;
            case 4:
                temp |= 0x10;
                temp &= ~0x20;
                break;
            case 5:
                temp |= 0x20;
                temp &= ~0x10;
                break;
            case 6:
                temp |= 0x40;
                temp &= ~0x80;
                break;
            case 7:
                temp |= 0x80;
                temp &= ~0x40;
                break;
          }
        check_relais_k1(temp);
        check_relais_k2(temp);
        if (temp != logical_output)
          {
            logical_output = temp;
            PortState = temp;
            semaphor_set(C_DoSave);
          }
      }
  } 
        
// init

void init_reverser_engine(void)
  {
    k1_trigger1_normal = my_eeprom_read_byte(&CV.K1_Trg1_normal);
    k1_trigger2_normal = my_eeprom_read_byte(&CV.K1_Trg2_normal);
    k1_trigger1_invers = my_eeprom_read_byte(&CV.K1_Trg1_invers);
    k1_trigger2_invers = my_eeprom_read_byte(&CV.K1_Trg2_invers);
    k2_trigger1_normal = my_eeprom_read_byte(&CV.K2_Trg1_normal);
    k2_trigger2_normal = my_eeprom_read_byte(&CV.K2_Trg2_normal);
    k2_trigger1_invers = my_eeprom_read_byte(&CV.K2_Trg1_invers);
    k2_trigger2_invers = my_eeprom_read_byte(&CV.K2_Trg2_invers);  
    
    logical_output = my_eeprom_read_byte(&CV.LastState);

    check_relais_k1(logical_output);
    check_relais_k2(logical_output);
  }  

#endif  // REVERSER_ENABLED
