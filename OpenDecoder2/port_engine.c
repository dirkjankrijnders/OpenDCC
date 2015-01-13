//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2006,2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      port_engine.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-20 V0.1 kw copied from opendecoder.c
//                               removed code for versatile output -
//                               we only do turnout commands and permanent
//                               commands (up to now)
//            2007-04-10 V0.2 kw added init code for timer 1
//                               turn off output only if time > 0
//                               outputs with time = 0 are kept forever
//            2007-04-23 V0.3 kw added compile switch
//                               PORT_ENABLED
//            2007-04-27 V0.4 kw evaluation of ReceivedActivate
//            2007-05-14 V0.5 kw added masking of TOIE1 to prevent reentry
//            2007-05-25 V0.6 kw Bugfix in init of FEEDBACK_PULLUP
//            2007-01-09 V0.7 kw Feedback lines are backdriven just before
//                               reading - to load the lines and not to
//                               read just random noise.                   
//
// tests:     2007-04-14 kw: feedback tested, FBM = 0,1; Magnet coils
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
// Port Engine does the control of the outputs only if PORT_ENABLED is true
//
#ifndef PORT_ENABLED
  #warning PORT_ENABLED not defined, now enabled by default (should be done in config.h)
  define PORT_ENABLED  TRUE
#endif

// Timing Definitions:
//

#define TICK_PERIOD 20000L       // 20ms tick for Timing Engine
                                 // => possible values for timings up to
                                 //    5.1s (=255/0.020)
                                 // note: this is also used as frame for
                                 // Servo-Outputs (OC1A and OC1B)



//----------------------------------------------------------------------------
// Global Data
// out_pwm controls outputs, monoflop pulses and toggling operations
//

volatile t_out_pwm out_pwm[8];


// control structure for turnouts
struct
  {
    unsigned char pulse_duration;
    unsigned char position;         // red (=0) or green (=1)
    enum fb_modes                   // mirror of cv546 ... cv549
      {                           
         COMMAND_ONLY = 0,          // 0:  no feedback, just a mirror of the command
         END_SWITCH = 1,            // 1:  feedback through intrinsic 
                                    //     coil end switches (like with roco line)
         EXTERN_LOW = 2,            // 2:  feedback with extra indicator -
                                    //     PC0 is low, when in *red* position
         EXTERN_HIGH = 3            // 3:  feedback with extra indicator -
                                    //     PC0 is high, when in *red* position
      } feedback_mode;        
  } turnout[4];


unsigned char failed_turnout;       // this is the last turnout with an error [0..3]

unsigned char feedback_pullup_mask; // active bita are pulled up.

enum feedback_methodes              // a mirror of cv545
  {
    NO_FB = 0,
    POS_ACK = 1,
    ALARM = 2,
  } feedback_method;

//----------------------------
volatile struct
  {
    unsigned char rest;     // Zeit bis zum wechsel in Ticks (20ms)
    unsigned char ontime;   // Einschaltzeit
    unsigned char offtime;  // Ausschaltzeit
    unsigned char pause;    
    unsigned char flashes;  // Anzahl Pulse
    unsigned char act_flash;    
  } led;


void turn_led_on(void)
  {
    led.rest = 0;
    LED_ON;
  }

void turn_led_off(void)
  {
    led.rest = 0;
    LED_OFF;
  }


// make a series of flashes, then a longer pause
void flash_led_fast(unsigned char count)
  {
    led.act_flash = 1;
    led.flashes = count;
    led.pause   = 1000000L / TICK_PERIOD;
    led.offtime = 240000L / TICK_PERIOD;
    led.ontime  = 120000L / TICK_PERIOD;
    led.rest    = 120000L / TICK_PERIOD;
    LED_ON;
  }



// Trick:   Speed up port access by declaring port access as inline code
//          This is done with a subroutine; if gcc runs with -os, this results in
//          single cbi and sbi statements!
// Note:    OUTPUT_PORT is defined in hardware.h


void output(unsigned char port_no, unsigned char state)
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


#if (NEON_ENABLED == TRUE)

unsigned char seed = 0xAA;             // var for prbs

// linear feedback shift register (prbs)
//
//      |---|    |---|    |---|    |---|    |---|    |---|    |---|    |---|
//    ->| 0 |--->| 1 |-o->| 2 |-o->| 3 |-o->| 4 |--->| 5 |--->| 6 |--->| 7 |--o--->
//   |  |---|    |---| |  |---| |  |---| |  |---|    |---|    |---|    |---|  |
//   |                 |        |        |                                    |
//    <--------------- + <----- + <----- + <----------------------------------
//
unsigned char prbs8(unsigned char seed)
  {
    unsigned char new_rnd;

    new_rnd = seed;                // copy bit 1
    new_rnd = new_rnd << 1;
    new_rnd = new_rnd ^ seed;      // xor bit 2
    new_rnd = new_rnd << 1;
    new_rnd = new_rnd ^ seed;      // xor bit 3
    new_rnd = new_rnd << 4;
    new_rnd = new_rnd ^ seed;      // xor bit 7

    // now put this bit to seed's lsb
    new_rnd = new_rnd >> 7;
    seed = seed << 1;
    new_rnd = new_rnd + seed; 
    
    return(new_rnd);
  }

unsigned char new_random(void)
  {
    seed = prbs8(seed);             // call 5* to decorrelate seeds
    seed = prbs8(seed);
    seed = prbs8(seed);
    seed = prbs8(seed);
    seed = prbs8(seed);
    return(seed);
  }

void all_off(void)
  {
    unsigned char i;
    for (i=0; i<8; i++)
      {
        out_pwm[i].mode = DELAY_TO_OFF;
        out_pwm[i].val  = 0;            
      }
    PORTB = 0;
    seed = prbs8(seed);      // run prbs - make it more random
  }

//==============================================================================
//
// Section 
//
// Timing Engine for NEON
//
// Howto:    Timer0 (with prescaler 1024 and 8 bit total count) triggers
//           an interrupt every TICK_PERIOD (=10ms @10MHz);
//           this interrupt does:
//           1. check mode
//           2. depending on mode:
//              DELAY_TO..: decrements port_action.delay of every port.
//                          if port_action.delay gets 0 the new value is loaded to the port
//              FLICKER:    decrement port_action.time
//                          if zero - load next bit of random pattern (val) and shift random
//                          pattern
//                          if random pattern is all zero -> load one
//              BLINK_IT:   decrements port_action.time of every port.
//                          if port_action.time gets 0 the invers value is loaded to the port
//                          port_action.time is reloaded with BLINK_IT_VAL.

#define BLINK_PERIOD 800000L      // 800000us = 0.8s
                                  // if a port is set to blinking


#define BLINK_IT_VAL ( BLINK_PERIOD / 2 / TICK_PERIOD)
#if (BLINK_IT_VAL > 255L)
      #warning: overflow in BLINK_IT_VAL - check TICK_PERIOD and/or BLINK_PERIOD
      #warning: suggestion: use a smaller BLINK_PERIOD
#endif    
#if (BLINK_IT_VAL == 0)
      #warning: underflow in BLINK_IT_VAL - check TICK_PERIOD and/or BLINK_PERIOD
      #warning: suggestion: use a larger BLINK_PERIOD
      #undef BLINK_IT_VAL
      #define BLINK_IT_VAL 1  // ätsch!
#endif    

volatile unsigned char tick_ratio = 1;  // = 1: every interrupts run state engine
                                        // > 1: we do subsampling => port actions are slower

volatile unsigned char isr_ratio;       // this is the running var for tick_ratio

#endif // (NEON_ENABLED == TRUE)

//==============================================================================
//
// Section 2
//
//------------------------------------------------------------------------------
//
// init_port_engine
//   initializes the output of the decoder 
//   a) setup timer (with TICK_PERIOD)
//   b) load values for port states from eeprom
//   c) preload outputs

void init_port_engine(void)
  {
    // Init Timer1 as Fast PWM with a CLKDIV (prescaler) of 8


    #define T1_PRESCALER   8    // may be 1, 8, 64, 256, 1024
                                // see also servo.c
    #if   (T1_PRESCALER==1)
        #define T1_PRESCALER_BITS   ((0<<CS12)|(0<<CS11)|(1<<CS10))
    #elif (T1_PRESCALER==8)
        #define T1_PRESCALER_BITS   ((0<<CS12)|(1<<CS11)|(0<<CS10))
    #elif (T1_PRESCALER==64)
        #define T1_PRESCALER_BITS   ((0<<CS12)|(1<<CS11)|(1<<CS10))
    #elif (T1_PRESCALER==256)
        #define T1_PRESCALER_BITS   ((1<<CS12)|(0<<CS11)|(0<<CS10))
    #elif (T1_PRESCALER==1024)
        #define T1_PRESCALER_BITS   ((1<<CS12)|(0<<CS11)|(1<<CS10))
    #endif


    // check TICK_PERIOD and F_CPU

    #if (F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER) > 65535L
      #warning: overflow in ICR1 - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a larger T1_PRESCALER
    #endif    
    #if (F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER) < 5000L
      #warning: resolution accuracy in ICR1 too low - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a smaller T1_PRESCALER
    #endif    

    // Timer 1 runs in FAST-PWM-Mode with ICR1 as TOP-Value (WGM13:0 = 14).
    // note: due to a bug in AVRstudio this can't be simulated !!

    ICR1 = F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER ;  

    OCR1A = F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER / 20;  // removed 24.12.2008 ???
    OCR1B = F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER / 15;   

    // OC1A and OC1B are mapped to Timer (for Servo Operation)
    //   (1 << COM1A1)          // compare match A
    // | (0 << COM1A0)          // Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at TOP
    // | (1 << COM1B1)          // compare match B
    // | (0 << COM1B0) 

    TCCR1A = (0 << COM1A1)          // compare match A
           | (0 << COM1A0)          // not activated yet -> this is done in init_servo();
           | (0 << COM1B1)          // compare match B
           | (0 << COM1B0) 
           | 0                      // reserved
           | 0                      // reserved
           | (1 << WGM11)  
           | (0 << WGM10);          // Timer1 Mode 14 = FastPWM - Int on Top (ICR1)
    TCCR1B = (0 << ICNC1) 
           | (0 << ICES1) 
           | (1 << WGM13) 
           | (1 << WGM12) 
           | (T1_PRESCALER_BITS);   // clkdiv


    TIMSK |= (1<<TOIE1)             // Timer1 Overflow
           | (0<<OCIE1A)            // Timer1 Compare A
           | (0<<OCIE1B)            // Timer1 Compare B
           | 0                      // reserved
           | (0<<TICIE1)            // Timer1 Input Capture
           | 0                      // reserved (Timer0 Compare B)
           | (0<<TOIE0)             // Timer0 Overflow
           | (0<<OCIE0);            // Timer0 Compare A
      

    timerval = 0;

    #if (PORT_ENABLED == TRUE)
      {
        unsigned char last_state, mask, i;
        
        feedback_pullup_mask = 0;
  
        // get cv's from eeprom

        turnout[0].pulse_duration = my_eeprom_read_byte(&CV.T_on_F1);
        turnout[0].feedback_mode =  my_eeprom_read_byte(&CV.FBM_F1);   // cv546
        turnout[1].pulse_duration = my_eeprom_read_byte(&CV.T_on_F2);
        turnout[1].feedback_mode =  my_eeprom_read_byte(&CV.FBM_F2);   // cv547
        turnout[2].pulse_duration = my_eeprom_read_byte(&CV.T_on_F3);
        turnout[2].feedback_mode =  my_eeprom_read_byte(&CV.FBM_F3);   // cv548
        turnout[3].pulse_duration = my_eeprom_read_byte(&CV.T_on_F4);
        turnout[3].feedback_mode =  my_eeprom_read_byte(&CV.FBM_F4);   // cv549

        feedback_method = my_eeprom_read_byte(&CV.FM);    // cv545

        // check out for last states

        last_state = my_eeprom_read_byte(&CV.LastState);
        mask = 0b000011;

        for (i=0; i<4; i++)
          {
            if ((turnout[i].feedback_mode == EXTERN_LOW) || (turnout[i].feedback_mode == EXTERN_HIGH))
              {
                // this needs pullup
                feedback_pullup_mask |= mask;
              }
  
            if (turnout[i].pulse_duration != 0)
              {
                // this is pulsed, no last state
                last_state &= ~mask;
              }
            mask = mask << 2;
          }

        FEEDBACK_PULLUP = feedback_pullup_mask;              // this is write only
        OUTPUT_PORT = last_state;
        PortState = last_state;  

        failed_turnout = 0x80;   // nothing wrong
      }
    #endif

    #if (NEON_ENABLED == TRUE)
        feedback_method = my_eeprom_read_byte(&CV.FM);    // cv545 ->  MyOpMode 

        if (feedback_method == 1)     tick_ratio = 25;
        if (feedback_method == 2)     tick_ratio = 50;
        if (feedback_method == 3)     tick_ratio = 100;     // slow mode
    
        if (feedback_method < 4)
          {  // Modes 0,1,2,3 are pulse modes - no last state
            PORTB = 0;
          }
        else
          {
            PORTB = my_eeprom_read_byte(&CV.LastState);
          }         
    #endif
  }

//==============================================================================
//
// Section 3
//
// Timing Engine
//
// Howto:    Timer1 (with prescaler 8 and 16 bit total count) triggers
//           an interrupt every TICK_PERIOD (=20ms @8MHz);
//           this interrupt decrements out_pwm.rest of every port.
//           if out_pwm touches 0 a new value is reloaded.
//           This result in flexible programmable timing of PORTB.
// 
  

ISR(TIMER1_OVF_vect)                        // Timer1 Overflow Int
  {
    
    disable_timer_interrupt();
    
    sei();                                  // allow DCC interrupt

    timerval++;                             // advance global clock

    #if (NEON_ENABLED == TRUE)
     {
       unsigned char port;
       unsigned char mask; 
       unsigned char my_time,my_val;
       t_mode  my_mode; 
       
       if (--isr_ratio == 0)
          {
            isr_ratio = tick_ratio;             // reload divider 
            mask = 1;
            for (port=0; port<8; port++)
              {
                my_mode = out_pwm[port].mode;
                my_val = out_pwm[port].val;
                my_time = out_pwm[port].rest;
                switch (my_mode)
                  {
                    case DELAY_TO_ON:
                        if (my_time !=0) 
                          {
                            if (--my_time == 0)
                              {
                                OUTPUT_PORT |= mask;
                              }
                            out_pwm[port].rest = my_time;
                          }
                        break;
                    case DELAY_TO_OFF:
                        if (my_time !=0) 
                          {
                            if (--my_time == 0)
                              {
                                OUTPUT_PORT &= ~mask;
                              }
                            out_pwm[port].rest = my_time;
                          }
                        break;
                    case BLINK_IT:
                        if (my_time !=0) 
                          {
                            if (--my_time == 0)
                              {
                                if (OUTPUT_PORT & mask)
                                  { // bit was on
                                    OUTPUT_PORT &= ~mask;
                                  }
                                else
                                  {
                                    OUTPUT_PORT |= mask;
                                  }
                                my_time = BLINK_IT_VAL;   // reload, to blink again
                              }
                            out_pwm[port].rest = my_time;
                          }
                        break;
                    case FLICKER:
                        if (my_val !=0) 
                          {
                            if (--my_time == 0)
                              {
                                // load next bit
                                my_val = my_val >> 1;
                                if (my_val == 0) 
                                  {
                                    // all flicker bits done - turn on port
                                    OUTPUT_PORT |= mask;
                                  }
                                else
                                  {
                                    if (my_val & 0x01)
                                      { 
                                        OUTPUT_PORT |= mask;
                                        my_time = 1;
                                      }
                                    else
                                      {
                                        OUTPUT_PORT &= ~mask;
                                        my_time = my_val & 0x07;     // new delay = random + 5 extra
                                        my_time += 10;
                                      }
                                  }
                              }
                            out_pwm[port].rest = my_time;
                            out_pwm[port].val = my_val;
                          }
                        break;
                  }
                 mask = mask << 1;                  // do *not* calc mask from port
              }
          }
      }
    #elif (PORT_ENABLED == TRUE)
      {
        unsigned char port;
        unsigned char mask; 
        unsigned char my_rest; 
       
        mask = 1;
        for (port=0; port<8; port++)
          {
            my_rest = out_pwm[port].rest;       // use a local variable to force
            if (my_rest !=0)                    // compiler to tiny code
              {
                if (--my_rest == 0)
                  {
                     if (OUTPUT_PORT & mask)
                      { // bit was on
                        my_rest = out_pwm[port].offtime;
                        OUTPUT_PORT &= ~mask;
                      }
                    else
                      {
                        my_rest = out_pwm[port].ontime;
                        OUTPUT_PORT |= mask;
                      }
                  }
                out_pwm[port].rest = my_rest;
              }
             mask = mask << 1;                  // do *not* calc mask from port
          }
      }
    #endif

    // now process led

    if (led.rest != 0)
      {
        if (--led.rest == 0)
          {
             if (LED_STATE)
              {
                if (led.act_flash >= led.flashes)
                  {
                    led.rest = led.pause;
                    led.act_flash = 0;
                  }
                else
                  {
                    led.rest = led.offtime;
                  }
                LED_OFF;
              }
            else
              {
                led.act_flash++;
                led.rest = led.ontime;
                LED_ON;
              }
          }
      }
    enable_timer_interrupt();
  }



//==============================================================================
//
// Section 4
//
// Timing Values for the Timing Engine
//
// All output control instructions are stored in tables. These tables are
// evaluated, when a action a pin is be performed.
//
// If a port is never toggled from the timing engine, it my be directly set.
//
struct t_action
  {
    unsigned char status;           // Multi purpose byte:
                                    // Bit 7:    new state of output pin
                                    // Bit 6:    variable first pulse enabled
                                    // Bit 2..0: output pin number
    unsigned char rest;             // time to keep the state; if 0: keep forever
    unsigned char ontime;           // ontime: if elapsed, output will turn off
    unsigned char offtime;          // 
  };


/*------------- Some predefined structures

struct t_action port0_on PROGMEM =   { (1 << 7) + 0, 0, 0, 0};
struct t_action port1_on PROGMEM =   { (1 << 7) + 1, 0, 0, 0};
struct t_action port2_on PROGMEM =   { (1 << 7) + 2, 0, 0, 0};
struct t_action port3_on PROGMEM =   { (1 << 7) + 3, 0, 0, 0};
struct t_action port4_on PROGMEM =   { (1 << 7) + 4, 0, 0, 0};
struct t_action port5_on PROGMEM =   { (1 << 7) + 5, 0, 0, 0};
struct t_action port6_on PROGMEM =   { (1 << 7) + 6, 0, 0, 0};
struct t_action port7_on PROGMEM =   { (1 << 7) + 7, 0, 0, 0};

struct t_action port0_off PROGMEM =  { (0 << 7) + 0, 0, 0, 0};
struct t_action port1_off PROGMEM =  { (0 << 7) + 1, 0, 0, 0};
struct t_action port2_off PROGMEM =  { (0 << 7) + 2, 0, 0, 0};
struct t_action port3_off PROGMEM =  { (0 << 7) + 3, 0, 0, 0};
struct t_action port4_off PROGMEM =  { (0 << 7) + 4, 0, 0, 0};
struct t_action port5_off PROGMEM =  { (0 << 7) + 5, 0, 0, 0};
struct t_action port6_off PROGMEM =  { (0 << 7) + 6, 0, 0, 0};
struct t_action port7_off PROGMEM =  { (0 << 7) + 7, 0, 0, 0};

//                                        "ON"     "PULS"  NR  
struct t_action port0_pulse PROGMEM =   { (1<<7) + (1<<6) + 0, 0, 0, 0};
struct t_action port1_pulse PROGMEM =   { (1<<7) + (1<<6) + 1, 0, 0, 0};
struct t_action port2_pulse PROGMEM =   { (1<<7) + (1<<6) + 2, 0, 0, 0};
struct t_action port3_pulse PROGMEM =   { (1<<7) + (1<<6) + 3, 0, 0, 0};
struct t_action port4_pulse PROGMEM =   { (1<<7) + (1<<6) + 4, 0, 0, 0};
struct t_action port5_pulse PROGMEM =   { (1<<7) + (1<<6) + 5, 0, 0, 0};
struct t_action port6_pulse PROGMEM =   { (1<<7) + (1<<6) + 6, 0, 0, 0};
struct t_action port7_pulse PROGMEM =   { (1<<7) + (1<<6) + 7, 0, 0, 0}; */



/*/                                           "ON" NR       rest,   ontime, offtime
struct t_action port0_amp1_rot    PROGMEM = { (0<<7) + 0,  AMP_YE, 0,      0   };
struct t_action port1_amp1_rot    PROGMEM = { (1<<7) + 1,  AMP_YE, 0,      0   };
struct t_action port3_amp1_rot    PROGMEM = { (0<<7) + 3,AMP_YE/2, 0,      0   };

struct t_action port0_amp1_green  PROGMEM = { (1<<7) + 0,  AMP_YE, 0,      0   };
struct t_action port1_amp1_green  PROGMEM = { (1<<7) + 1,  AMP_YE, 0,      0   };
struct t_action port2_amp1_green  PROGMEM = { (0<<7) + 2,  AMP_YE, 0,      0   };
struct t_action port3_amp1_green  PROGMEM = { (1<<7) + 3,3*AMP_YE/2, 0,      0   };

struct t_action port1_amp1_blink  PROGMEM = { (1<<7) + 1,  AMP_BL, AMP_BL, AMP_BL };

//
struct t_action port4_amp2_rot    PROGMEM = { (0<<7) + 4,  AMP_YE, 0,      0   };
struct t_action port5_amp2_rot    PROGMEM = { (1<<7) + 5,  AMP_YE, 0,      0   };
struct t_action port7_amp2_rot    PROGMEM = { (0<<7) + 7,AMP_YE/2, 0,      0   };

struct t_action port4_amp2_green  PROGMEM = { (1<<7) + 4,  AMP_YE, 0,      0   };
struct t_action port5_amp2_green  PROGMEM = { (1<<7) + 5,  AMP_YE, 0,      0   };
struct t_action port6_amp2_green  PROGMEM = { (0<<7) + 6,  AMP_YE, 0,      0   };
struct t_action port7_amp2_green  PROGMEM = { (1<<7) + 7,3*AMP_YE/2, 0,      0   };

struct t_action port5_amp2_blink  PROGMEM = { (1<<7) + 5,  AMP_BL, AMP_BL, AMP_BL };
*/


// Nachfolgend mal ein Baustellenblitzer
// LED0: ____x______________________________________x_____________________________
// LED1: _______x______________________________________x__________________________
// LED2: __________x______________________________________x_______________________
// LED3: _____________x______________________________________x____________________
// LED4: ________________x______________________________________x_________________
// LED5: ___________________x______________________________________x______________
// LED6: ______________________x______________________________________x___________
// LED7: _________________________x______________________________________x________



// #define BST_ON 1     // Time for one flash
// #define BST_OFF 80   // BST_ON + BST_OFF = period
// #define BST_RUN 4    // Offset between succeeding flashes
#define BST_ON  (  30000L / TICK_PERIOD)     // 30ms: Time for one flash
#define BST_OFF (1100000L / TICK_PERIOD)     // BST_ON + BST_OFF = period
#define BST_RUN ( 100000L / TICK_PERIOD)     // Offset between succeeding flashes

//                                        "ON" NR       rest, ontime, offtime
const struct t_action port0_bst PROGMEM =   { (0<<7) + 0,   BST_RUN, BST_ON, BST_OFF};
const struct t_action port1_bst PROGMEM =   { (0<<7) + 1, BST_RUN*2, BST_ON, BST_OFF};
const struct t_action port2_bst PROGMEM =   { (0<<7) + 2, BST_RUN*3, BST_ON, BST_OFF};
const struct t_action port3_bst PROGMEM =   { (0<<7) + 3, BST_RUN*4, BST_ON, BST_OFF};
const struct t_action port4_bst PROGMEM =   { (0<<7) + 4, BST_RUN*5, BST_ON, BST_OFF};
const struct t_action port5_bst PROGMEM =   { (0<<7) + 5, BST_RUN*6, BST_ON, BST_OFF};
const struct t_action port6_bst PROGMEM =   { (0<<7) + 6, BST_RUN*7, BST_ON, BST_OFF};
const struct t_action port7_bst PROGMEM =   { (0<<7) + 7, BST_RUN*8, BST_ON, BST_OFF};


//------------------------------------------------------------------------------
// This Routine copies the given action values to the corresponding array
// (loading the values is a semaphor operation, therefore we disable int)

/*
void perform_action(const struct t_action *pData)
  {
    unsigned char ctrl;
    unsigned char port;
    unsigned char mask; 

    ctrl = pgm_read_byte(&pData->status);

    port = ctrl & 0b0111;
    mask = 1 << port;
    cli();
    if (ctrl & (1<<7)) OUTPUT_PORT |= mask;
    else               OUTPUT_PORT &= ~mask;
    
    out_pwm[port].rest    = pgm_read_byte(&pData->rest);
    out_pwm[port].ontime  = pgm_read_byte(&pData->ontime);
    out_pwm[port].offtime = pgm_read_byte(&pData->offtime);
    sei(); 
  }

void port_test(void)
  {
    perform_action (&port0_bst);
    perform_action (&port1_bst);
    perform_action (&port2_bst);
    perform_action (&port3_bst);
    perform_action (&port4_bst);
    perform_action (&port5_bst);
    perform_action (&port6_bst);
    perform_action (&port7_bst);
  }
*/


//==============================================================================
//
// Section 5
//
// Performing the Command

//------------------------------------------------------------------------------
// This Routine is called when myAdr is received


#if (PORT_ENABLED == TRUE)

void port_action(unsigned int Command, unsigned char Activate)
  {
    unsigned char myCommand, myTurnout;
    myCommand = Command & 0b00000111;
    myTurnout = myCommand >> 1;

    if (Command > 7) return;                    // not our Address
    
    // checkout local alarm

    if (failed_turnout == myTurnout)   
      { // we got a command to the same turnout
        turn_led_off(); 
        if (feedback_method == ALARM) DCC_ACK_OFF;
        failed_turnout = 0x80;
      }

    if (Activate)                               //  activate coil?
      {
        
        disable_timer_interrupt(); 
        
        if (myCommand == 0)
          {
            out_pwm[0].rest = turnout[0].pulse_duration;         
            out_pwm[1].rest = 0;
            turnout[0].position = 0;         
            output(PB0,1);
            output(PB1,0);
            if (turnout[0].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else if (myCommand == 1)
          {
            out_pwm[1].rest = turnout[0].pulse_duration;         
            out_pwm[0].rest = 0;         
            turnout[0].position = 1;         
            output(PB0,0);
            output(PB1,1);
            if (turnout[0].pulse_duration == 0) 
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else if (myCommand == 2)
          {
            out_pwm[2].rest = turnout[1].pulse_duration;         
            out_pwm[3].rest = 0;         
            turnout[1].position = 0;         
            output(PB2,1);
            output(PB3,0);
            if (turnout[1].pulse_duration == 0) 
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else if (myCommand == 3)
          {
            out_pwm[3].rest = turnout[1].pulse_duration;         
            out_pwm[2].rest = 0;         
            turnout[1].position = 1;         
            output(PB2,0);
            output(PB3,1);
            if (turnout[1].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }

          }
        else if (myCommand == 4)
          {
            out_pwm[4].rest = turnout[2].pulse_duration;         
            out_pwm[5].rest = 0;         
            turnout[2].position = 0;         
            output(PB4,1);
            output(PB5,0);
            if (turnout[2].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else if (myCommand == 5)
          {
            out_pwm[5].rest = turnout[2].pulse_duration;         
            out_pwm[4].rest = 0;         
            turnout[2].position = 1;         
            output(PB4,0);
            output(PB5,1);
            if (turnout[2].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else if (myCommand == 6)
          {
            out_pwm[6].rest = turnout[3].pulse_duration;         
            out_pwm[7].rest = 0;         
            turnout[3].position = 0;         
            output(PB6,1);
            output(PB7,0);
            if (turnout[3].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }
        else // (myCommand == 7)
          {
            out_pwm[7].rest = turnout[3].pulse_duration;         
            out_pwm[6].rest = 0;         
            turnout[3].position = 1;         
            output(PB6,0);
            output(PB7,1);
            if (turnout[3].pulse_duration == 0)
              {
                PortState = PORTB;
                semaphor_set(C_DoSave);
              }
          }       
         
        enable_timer_interrupt(); 
      } // coil
    else
      {
        // coil off command - we also do a feedback
        disable_timer_interrupt(); 
        
        if (myCommand == 0)
          {
            out_pwm[0].rest = 0;         // stop every timer
            out_pwm[1].rest = 0;
            if (turnout[0].pulse_duration != 0)
              {                          // in pulse mode: allways turn off both coils
                output(PB0,0);
                output(PB1,0);
              }
          }
        else if (myCommand == 1)
          {
            out_pwm[1].rest = 0;         
            out_pwm[0].rest = 0;         
            if (turnout[0].pulse_duration != 0)
              {
                output(PB0,0);
                output(PB1,0);
              }
          }
        else if (myCommand == 2)
          {
            out_pwm[2].rest = 0;         
            out_pwm[3].rest = 0;         
            if (turnout[1].pulse_duration != 0)
              {
                output(PB2,0);
                output(PB3,0);
              }
          }
        else if (myCommand == 3)
          {
            out_pwm[3].rest = 0;         
            out_pwm[2].rest = 0;         
            if (turnout[1].pulse_duration != 0)
              {
                output(PB2,0);
                output(PB3,0);
              }
          }
        else if (myCommand == 4)
          {
            out_pwm[4].rest = 0;         
            out_pwm[5].rest = 0;         
            if (turnout[2].pulse_duration != 0)
              {
                output(PB4,0);
                output(PB5,0);
              }
          }
        else if (myCommand == 5)
          {
            out_pwm[5].rest = 0;         
            out_pwm[4].rest = 0;         
            if (turnout[2].pulse_duration != 0)
              {
                output(PB4,0);
                output(PB5,0);
              }
          }
        else if (myCommand == 6)
          {
            out_pwm[6].rest = 0;         
            out_pwm[7].rest = 0;         
            if (turnout[3].pulse_duration != 0)
              {
                output(PB6,0);
                output(PB7,0);
              }
          }
        else // (myCommand == 7)
          {
            out_pwm[7].rest = 0;         
            out_pwm[6].rest = 0;         
            if (turnout[3].pulse_duration != 0)
              {
                output(PB6,0);
                output(PB7,0);
              }
          }       
         
        enable_timer_interrupt(); 
        
        _mydelay_us(10);   // wait 10us to settle IO lines
        
        // feedback is communicated to the host depending on feedback_method:
        //      0:  not at all
        //      1:  positve feedback (announcing right position with a pulse)
        //      2:  negative feedback (ACK aktivated until the next turnout command)
        
        switch(feedback_method)
          {
            case NO_FB:
                if (check_position(myCommand) == 1) return;
                local_alarm(myTurnout);
                break;
            case POS_ACK:
                if (check_position(myCommand) == 1) 
                  {
                    activate_ACK(3);            // next preamble in 1.6ms over
                    return;
                  }
                else
                  {
                    local_alarm(myTurnout);
                  }
                break;
            case ALARM:
                if (check_position(myCommand) == 1) 
                  {
                    return;
                  }
                else
                  {
                    DCC_ACK_ON;                 // remote alarm (permanent)
                    local_alarm(myTurnout);
                  }
                break;
          }
      }
  }


// issue an alarm:
// parameter count: this turnout failed
//                  range: 0...3 -> turnout 1..4 

void local_alarm(unsigned char count)
  {
    failed_turnout = count;
    flash_led_fast(count+1);  // show the turnout
  }



//------------------------------------------------------------------------------------
// check_position tests the position of a turnout:
// depending on feedback mode of this turnout:
//      0:  no feedback, just a control of the previous command
//      1:  feedback through intrinsic coil end switches (like in roco line)
//      2:  feedback with extra indicator - PA0 is low, when in *red* position
//      3:  feedback with extra indicator - PA0 is high, when in *red* position
//
// PA0 may be PA2, PA4, PA6 depending on turnout
// PINA is aliased as FEEDBACK_IN
// PORTA is aliased as FEEDBACK_PULLUP  (note: this is write only)
//
// return: true (non zero) if position is okay
//         false if wrong


unsigned char check_position(unsigned char myCommand)
  {
    unsigned char myMask;
    unsigned char myTurnout, myBits;

    myTurnout = myCommand >> 1;

    switch (turnout[myTurnout].feedback_mode)
      {
        case COMMAND_ONLY:
            if ((myCommand & 0b01) == 0b00)
              {
                // is it red?
                if (turnout[myTurnout].position == 0) return(1);    
              }
            else  
              {
                // is it green?
                if (turnout[myTurnout].position == 1) return(1);    
              }
            break;

        case END_SWITCH:
            // set port to zero - try to pull down the feedback line
            
            myMask = 0b011 << (myTurnout * 2);
            DDRA = myMask;                     // PORTA: set these bits as outputs
                                               // set the readback as low
            _mydelay_us(1);                     // to discharge any noise on them
            DDRA = 0;                          // input again

            _mydelay_us(1);                    // some delay to let it pullup

            myBits = FEEDBACK_IN >> (myTurnout * 2);
            myBits &= 0b11;

            if ((myCommand & 0b01) == 0b00)
              {
                // is it red?
                if (myBits == 0b10) return(1);
              }
            else
              {
                // is it green?
                if (myBits == 0b01) return(1);
              }
            break;

        case EXTERN_LOW:

            myBits = FEEDBACK_IN >> (myTurnout * 2);
            // myBits &= 0b11;

            if ((myCommand & 0b01) == 0b00)
              {
                // is it red?
                if ((myBits & 0b01) == 0b00) return(1);
              }
            else
              {
                // is it green?
                if ((myBits & 0b01) == 0b01) return(1);
              }
            break;

        case EXTERN_HIGH:

            myBits = FEEDBACK_IN >> (myTurnout * 2);
            // myBits &= 0b11;
            
            if ((myCommand & 0b01) == 0b00)
              {
                // is it red?
                if ((myBits & 0b01) == 0b01) return(1);
              }
            else
              {
                // is it green?
                if ((myBits & 0b01) == 0b00) return(1);
              }
            break;

      } 
    return(0);
  }

#endif  // PORT_ENABLED




#if (NEON_ENABLED == TRUE)
//------------------------------------------------------------------------------
// This Routine is called when myAdr is received
// the received command is executed, depending on the operation mode

void neon_action(unsigned int Command)
  {
    unsigned char MyOpMode = feedback_method;  // copy from CV-Value (reuse this CV)
    
    unsigned char myCommand, myTurnout;
    myCommand = Command & 0b00000111;
    myTurnout = myCommand >> 1;
    
    if (Command > 7) return;                    // not our Address
    
    cli();                      // block interrupts

    // output(PB2,0); 
    
    if (MyOpMode == 7)                  // set all bits individually
      {
        Communicate |= (1<<C_DoSave);

        if ((myCommand & 0x01) == 0)
          {
            out_pwm[myTurnout].mode = DELAY_TO_OFF;   
          }
        else
          {
            out_pwm[myTurnout].mode = DELAY_TO_ON; 
          }
        out_pwm[myTurnout].rest  = 1;    
      }
    else if (MyOpMode == 6)                         // blink all bits individually
      {
        Communicate |= (1<<C_DoSave);

        if ((myCommand & 0x01) == 0)
          {
            out_pwm[myTurnout].mode = DELAY_TO_OFF;   
          }
        else
          {
            out_pwm[myTurnout].mode = BLINK_IT; 
          }
        out_pwm[myTurnout].rest  = 1;

      }
    else if (MyOpMode == 5)
      {
      }
    else if (MyOpMode == 4)
      {
      }
    else // all OpMode 3...0 are equal, only tick_ratio is different
      {  // 
        Communicate |= (1<<C_DoSave); 
         
        if (myCommand == 0)
          {
            all_off(); 
          }
        else if (myCommand == 1)  // all on immediately
          {
            unsigned char i;
            for (i=0; i<8; i++)
              {
                out_pwm[i].mode = DELAY_TO_ON;
                out_pwm[i].rest  = 1;
              }
          }
        else if (myCommand == 2)
          {
            unsigned char i;
            for (i=0; i<8; i++)
              {
                out_pwm[i].mode = DELAY_TO_OFF;
                out_pwm[i].rest  = new_random();
              }   
          }
        else if (myCommand == 3)
          {
            unsigned char i;
            for (i=0; i<8; i++)
              {
                out_pwm[i].mode = DELAY_TO_ON;
                out_pwm[i].rest  = new_random();
              }      
          }
        else if (myCommand == 4)
          {
                all_off();   
          }
        else if (myCommand == 5)
          {
            unsigned char i;
            for (i=0; i<8; i++)
              {
                out_pwm[i].mode = FLICKER;
                out_pwm[i].val  = new_random();
                out_pwm[i].rest  = new_random() & 0x0f;
                out_pwm[i].rest  += 1;
              }     
           }
         else if (myCommand == 6)
           {
                // reserved
                output(PB6,1);
                output(PB7,0);
           }
         else if (myCommand == 7)
           {
                output(PB6,0);
                output(PB7,1);
           }           
      }  // OpMode
    sei();
  }

#endif  // (NEON_ENABLED == TRUE)

void direct_action(unsigned int Command)
  {
    unsigned char myCommand;
    myCommand = Command & 0b00001111;

    if (Command >= 16) return;                    // not our Address

    if (myCommand == 0)
      {
        output(PB0,0);
      }
    else if (myCommand == 1)
      {
        output(PB0,1);
      }
    else if (myCommand == 2)
      {
        output(PB1,0);
      }
    else if (myCommand == 3)
      {
        output(PB1,1);
      }
    else if (myCommand == 4)
      {
        output(PB2,0);
      }
    else if (myCommand == 5)
      {
        output(PB2,1);
      }
    else if (myCommand == 6)
      {
        output(PB3,0);
      }
    else if (myCommand == 7)
      {
        output(PB3,1);
      }
    else if (myCommand == 8)
      {
        output(PB4,0);
      }
    else if (myCommand == 9)
      {
        output(PB4,1);
      }
    else if (myCommand == 10)
      {
        output(PB5,0);
      }
    else if (myCommand == 11)
      {
        output(PB5,1);
      }
    else if (myCommand == 12)
      {
        output(PB6,0);
      }
    else if (myCommand == 13)
      {
        output(PB6,1);
      }
    else if (myCommand == 14)
      {
        output(PB7,0);
      }
    else if (myCommand == 15)
      {
        output(PB7,1);
      }
    PortState = PORTB;
    semaphor_set(C_DoSave);
  }
