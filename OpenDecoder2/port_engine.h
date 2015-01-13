//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder
//
// Copyright (c) 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      port_engine.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-14 V0.01 kw copied from opendecoder.c
//            2011-11-09 V0.02 kw added NEON_ENABLED
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: hardware defintions
//
//
//------------------------------------------------------------------------

// performs the command 

void port_action(unsigned int Command, unsigned char Activate);

void neon_action(unsigned int Command);

void init_port_engine(void);

// data structures for port control

typedef enum {DELAY_TO_ON,      // Port hat Einschaltverzögerung 
              DELAY_TO_OFF,     // Port hat Ausschaltverzögerung 
              BLINK_IT,         // Port Blinkt
              FLICKER,          // Port Flackert
             } t_mode;


#if (NEON_ENABLED == TRUE)
    typedef struct
      {
        t_mode mode;                // This is the mode of this port bit 
        unsigned char rest;         // current duration of state
        unsigned char val;          // current value for special operation
        unsigned char ontime;       // ontime (to be reloaded)
        unsigned char offtime;      // offtime (to be reloaded)
      } t_out_pwm;
#else
    typedef struct
      {
        unsigned char rest;         // time to keep the actual state (given in ticks (20ms)
        unsigned char ontime;       // ontime (to be reloaded)
        unsigned char offtime;      // offtime (to be reloaded)
      } t_out_pwm;
#endif

extern volatile t_out_pwm out_pwm[8];

// general support routines

void turn_led_on(void);
  
void turn_led_off(void);
  
void flash_led_fast(unsigned char count);

unsigned char check_position(unsigned char myCommand);

void local_alarm(unsigned char count);

void direct_action(unsigned int Command);
