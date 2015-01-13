//----------------------------------------------------------------
//
// OpenDCC - OpenDecoder2 / OpenDecoder3 / OpenDCC
//
// Copyright (c) 2011 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      rgb.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2011-09-20 V0.01 kw started
//            2011-10-04 V0.02 kw added predefined profiles
//
//-----------------------------------------------------------------

// Hardware:
// we use 3 compare registers on timer2 and timer3
//


#include <avr/sleep.h>          // definitions for power-down modes
#include <avr/pgmspace.h>       // definitions or keeping constants in program memory
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/pgmspace.h>        // put var to program memory
#include <avr/io.h>              // this contains all the IO port definitions
#include <avr/eeprom.h> 
#include <avr/interrupt.h>
#include <string.h>

#include "config.h"              // general definitions the decoder, cv's
#include "myeeprom.h"            // wrapper for eeprom

#include "servo.h"              // calls for servo movement


#define SIMULATION  0            // 0: real application
                                 // 1: test action
#if (SIMULATION != 0)
   #warning SIMULATION is on! - do not use on real hardware!
#endif


#if (RGB_ENABLED == TRUE)
#if ((TARGET_HARDWARE == OPENDECODER25) || (TARGET_HARDWARE == OPENDECODER28))
//------------------------------------------------------------------------------------
// Doku
//
// LED:         red        green     blue
// Port:        PD4        PB1       PB4      ; high = LED on
// OC-register: OC3A       OC2       OC3B
// Operation:   FastPWM    FastPWM   FastPWM  ; all 8 bit mode
//              Mode5      Mode3     Mode5
//              WGM=0101   WGM=11    WGM=0101
// Prescaler:   64         64        64       ; = 488Hz @ 8MHz CPU
// Compare:     inv        inv       inv
// code:        set_R()    set_G()   set_B()  ; ocr = 255 - color value (to get it completely dark)
// 


//------------------------------------------------------------------------------------
// Data

//------------------------------------------------------------------------------

#define RGB_UPDATE_PERIOD     20000L    // 20ms -> 50Hz

// bit field for servo.control:
#define SC_BIT_ACTUAL    0          // 0=pre or during A, 1=pre or during B movement
#define SC_BIT_MOVING    1          // 0=stopped, 1=moving
#define SC_BIT_OUT_CTRL  5          // 0=no Output Control, 1=Control corresponding output
#define SC_BIT_REPEAT    6          // 1=repeat mode
#define SC_BIT_TERMINATE 7

#define MOVE2A           0
#define MOVE2B           1



// internal, but static:
enum rgb_states
  {                                 // actual state
     IDLE,
     RGB_WAIT_TICK,                   // wait for next active slot
  } rgb_state;



void init_rgb_timer(void)
  {
    // Init Timer2 as Fast PWM with a CLKDIV (prescaler) of 64

    #define T2_PRESCALER   64    // may be 1, 8, 32, 64, 128, 256, 1024
    #if   (T2_PRESCALER==1)
        #define T2_PRESCALER_BITS   ((0<<CS22)|(0<<CS21)|(1<<CS20))
    #elif (T2_PRESCALER==8)
        #define T2_PRESCALER_BITS   ((0<<CS22)|(1<<CS21)|(0<<CS20))
    #elif (T2_PRESCALER==32)
        #define T2_PRESCALER_BITS   ((0<<CS22)|(1<<CS21)|(1<<CS20))
    #elif (T2_PRESCALER==64)
        #define T2_PRESCALER_BITS   ((1<<CS22)|(0<<CS21)|(0<<CS20))
    #elif (T2_PRESCALER==128)
        #define T2_PRESCALER_BITS   ((1<<CS22)|(0<<CS21)|(1<<CS20))
    #elif (T2_PRESCALER==256)
        #define T2_PRESCALER_BITS   ((1<<CS22)|(1<<CS21)|(0<<CS20))
    #elif (T2_PRESCALER==1024)
        #define T2_PRESCALER_BITS   ((1<<CS22)|(1<<CS21)|(1<<CS20))
    #else
        #error void value T2_PRESCALER
    #endif

    TCCR2 = (0 << FOC2)        // Timer2: fastpwm
          | (1 << WGM20)       // wgm = 00: normal mode, top=0xff
          | (1 << COM21)       // COMx1,COMx0 = 00: normal mode, pin operates as usual
          | (1 << COM20)       // COMx1,COMx0 = 10: compare match: set OC2 at TOP. clear at OCR   xxxxx
                               // COMx1,COMx0 = 11: compare match: clear OC2 at TOP. set at OCR
                               // 
          | (1 << WGM21)       // 
          | T2_PRESCALER_BITS; 

    TCNT2 = 0;

    // Init Timer3 as Fast PWM with a CLKDIV (prescaler) of 64

    #define T3_PRESCALER   64   // may be 1, 8, 16, 32, 64, 256, 1024
    #if   (T3_PRESCALER==1)
        #define T3_PRESCALER_BITS   ((0<<CS32)|(0<<CS31)|(1<<CS30))
    #elif (T3_PRESCALER==8)
        #define T3_PRESCALER_BITS   ((0<<CS32)|(1<<CS31)|(0<<CS30))
    #elif (T3_PRESCALER==16)
        #define T3_PRESCALER_BITS   ((1<<CS32)|(1<<CS31)|(0<<CS30))
    #elif (T3_PRESCALER==32)
        #define T3_PRESCALER_BITS   ((1<<CS32)|(1<<CS31)|(1<<CS30))
    #elif (T3_PRESCALER==64)
        #define T3_PRESCALER_BITS   ((0<<CS32)|(1<<CS31)|(1<<CS30))
    #elif (T3_PRESCALER==256)
        #define T3_PRESCALER_BITS   ((1<<CS32)|(0<<CS31)|(0<<CS30))
    #elif (T3_PRESCALER==1024)
        #define T3_PRESCALER_BITS   ((1<<CS32)|(0<<CS31)|(1<<CS30))
    #else
        #error void value T3_PRESCALER
    #endif

    // FastPWM, 8 Bit = Mode 5: WGM3 = 0101

    TCCR3A = (1 << COM3A1)          // compare match A
           | (1 << COM3A0)          // xxxxxxx
           | (1 << COM3B1)          // compare match B
           | (1 << COM3B0)          // xxxxxxx
           | (0 << FOC3A)
           | (0 << FOC3B)
           | (0 << WGM31)  
           | (1 << WGM30);  
    TCCR3B = (0 << ICNC3) 
           | (0 << ICES3) 
           | (0 << WGM33) 
           | (1 << WGM32) 
           | (T3_PRESCALER_BITS);   // clkdiv

    TCNT3 = 0;
  }

unsigned char RED;
unsigned char GREEN;
unsigned char BLUE;




// hardware access, values given from 0..255

void set_R(unsigned char red_value)
  {
    OCR3AL = 255-red_value;
    RED = red_value;
  }

void set_G(unsigned char green_value)
  {
    OCR2 = 255-green_value;
    GREEN = green_value;
  }

void set_B(unsigned char blue_value)
  {
    OCR3BL = 255-blue_value;
    BLUE = blue_value;
  }


//------------------------------------------------------------------------------






//------------------------------------------------------------------------------




#define SIZE_RGB_FADE      24         // number of entries (triples) in one control

typedef struct    // fade values, normalized [0..255]
      {
        unsigned char time;       
        unsigned char red;            
        unsigned char green;            
        unsigned char blue;            
      } t_rgb_point;



t_rgb_point farbkreis[] PROGMEM = 
  {
   // time, r, g, b
    {   0 , 255 ,   0 ,   0 },
    {   1 , 255 , 255 ,   0 },
    {   2 ,   0 , 255 ,   0 },
    {   3 ,   0 , 255 , 255 },
    {   4 ,   0 ,   0 , 255 },
    {   5 , 255 ,   0 , 255 },
    {   6 , 255 ,   0 ,   0 },
    {   0 ,   0 ,   0 ,   0 },              // end of list: time = 0
  };

t_rgb_point einzelfarben[] PROGMEM = 
  {
   // time, r, g, b
    {   0 ,   0 ,   0 ,   0 },
    {   1 , 255 ,   0 ,   0 },
    {   2 ,   0 ,   0 ,   0 },
    {   3 ,   0 , 255 ,   0 },
    {   4 ,   0 ,   0 ,   0 },
    {   5 ,   0 ,   0 , 255 },
    {   6 ,   0 ,   0 ,   0 },
    {   0 ,   0 ,   0 ,   0 },              // end of list: time = 0
  };

t_rgb_point einzelfarben1[] PROGMEM = 
  {
   // time, r, g, b
    {   0 ,   1 ,   1 ,   1 },
    {   1 , 255 ,   1 ,   1 },
    {   2 ,   1 ,   1 ,   1 },
    {   3 ,   1 , 255 ,   1 },
    {   4 ,   1 ,   1 ,   1 },
    {   5 ,   1 ,   1 , 255 },
    {   6 ,   1 ,   1 ,   1 },
    {   0 ,   0 ,   0 ,   0 },              // end of list: time = 0
  };

t_rgb_point tuerkis_schimmer[] PROGMEM = 
  {
   // time, r, g, b
    {   0 ,   0 , 128 , 255 },
    {   1 ,   0 , 255 , 128 },
    {   2 ,   0 , 128 , 128 },
    {   3 ,  10 , 200 , 128 },
    {   4 ,  10 , 100 , 200 },
    {   5 ,  20 , 160 , 160 },
    {   6 ,  15 , 120 , 200 },
    {   7 ,   0 , 120 , 200 },
    {   8 ,   0 ,  20 , 160 },
    {   9 ,   0 , 180 ,  40 },
    {  10 ,  10 , 180 , 200 },
    {  11 ,   0 , 128 , 255 },
    {   0 ,   0 ,   0 ,   0 },              // end of list: time = 0
  };




t_rgb_point *pre_def_fades[] =
  {
    einzelfarben1,                           //  0: einzelfarben
    farbkreis,                              //  1: farbkreis
    tuerkis_schimmer,                       //  2:
  };    


t_rgb_point *eeprom_fades[] =
  {
    (t_rgb_point *) CV.RGB_fade1,           //  1: eeprom curve 1
  };    
    


t_rgb_point rgb_fade[SIZE_RGB_FADE]; 


typedef struct
  { 
    unsigned char redmax;               // upper  limit [0..255]
    
    unsigned char greenmax;               // upper  limit [0..255]
    
    unsigned char bluemax;               // upper  limit [0..255]
    
    unsigned char control;          // Bit 0: ACTUAL:   0=pre A, 1=pre B movement
                                    // Bit 1: MOVING:   0=at endpoint, 1=currently moving
                                    // Bit 5: OUT_CTRL: 0=no, 1=yes
                                    // Bit 6: REPEAT:   0=single, 1=forever
                                    // Bit 7: TERMINATE: flag: terminate after next B

    unsigned char repeat;           // if REPEAT: this is the number of repeats to do

    unsigned char fade_index;      // points to actual target in curve

    t_rgb_point *fade;           // fading curve, normalized [0..255]
      
    unsigned int active_time;       // runtime: relative time to start point
                                    // 0        = restart Servos
                                    // 0xffff   = finished

    unsigned char time_ratio;       // ratio between runtime and curve time
  } t_rgb_ctrl;



t_rgb_ctrl rgb_ctrl =
  {     255,            // unsigned char redmax; 
        255,            // unsigned char greenmax; 
        255,            // unsigned char bluemax; 
          0,            // unsigned char control;
          0,            // unsigned char repeat;
          1,            // unsigned char fade_index;
      rgb_fade,     // pointer to curve
          0,            // active time
          1,            // ratio of curve
  };


void rgb_copy_fade(unsigned char fade_ean)
  {
    t_rgb_point *dest;
    t_rgb_point *src;
    unsigned char my_fade_ean;
    unsigned char i;

    dest = rgb_ctrl.fade;
    if (fade_ean < 0x80)
      {
        my_fade_ean = fade_ean;
        if (my_fade_ean >= (sizeof(pre_def_fades)/sizeof(pre_def_fades[0])))
            my_fade_ean = 0;  // default
        src = pre_def_fades[my_fade_ean];

        // memcpy_P((void *)dest, (PGM_VOID_P)src, SIZE_SERVO_CURVE*sizeof(t_curve_point));
        // copy one less, to keep zero at end of memory
        for (i=0; i<(SIZE_RGB_FADE-1); i++)
          {
            dest->time = pgm_read_byte(&src->time);         // check end of list!!
            dest->red = pgm_read_byte(&src->red);
            dest->green = pgm_read_byte(&src->green);
            dest->blue = pgm_read_byte(&src->blue);
            dest++;
            src++;
          }
      }
    else
      {
        my_fade_ean = fade_ean & 0x7F;
        if (my_fade_ean >= (sizeof(eeprom_fades)/sizeof(eeprom_fades[0])))
            my_fade_ean = 0;  // default
        src = eeprom_fades[my_fade_ean];

        // memcpy_P((void *)dest, (PGM_VOID_P)src, SIZE_SERVO_CURVE*sizeof(t_curve_point));
        // copy one less, to keep zero at end of memory
        for (i=0; i<(SIZE_RGB_FADE-1); i++)
          {
            dest->time = my_eeprom_read_byte(&src->time);        
            dest->red = my_eeprom_read_byte(&src->red);
            dest->green = my_eeprom_read_byte(&src->green);
            dest->blue = my_eeprom_read_byte(&src->blue);
            dest++;
            src++;
          }
      }
  }


void do_rgb_fade(void)
  {
    rgb_ctrl.redmax = my_eeprom_read_byte(&CV.REDmax);
    rgb_ctrl.greenmax = my_eeprom_read_byte(&CV.GREENmax);
    rgb_ctrl.bluemax = my_eeprom_read_byte(&CV.BLUEmax);
    
    rgb_copy_fade(my_eeprom_read_byte(&CV.RGB_profile));
    rgb_ctrl.time_ratio = my_eeprom_read_byte(&CV.RGB_time);
    rgb_ctrl.fade_index = 1;
    rgb_ctrl.active_time = 0;
    rgb_ctrl.repeat = my_eeprom_read_byte(&CV.RGB_repeat);
    rgb_ctrl.control = 0;    
    set_R(0);   //   red_value
    set_G(0);  //   green_value;
    set_B(0);  //   blue_value;
  }

void stop_rgb_fade(void)
  {
    rgb_state = IDLE;
    rgb_ctrl.active_time = 0xFFFF;
    set_R(20);   //   red_value
    set_G(0);  //   green_value;
    set_B(0);  //   blue_value;
  }
//------------------------------------------------------------------------------
//
#define CALC_GAIN  128L            // 2 bis 128; 128 ist obere Grenze wegen möglichen Überlauf

void calc_rgb_next_val(void)
  {
    unsigned char myindex;        // index in curve
    int16_t posi;
    int16_t dt, delta_t;          // time delta always > 0
    int32_t posl;
    int16_t delta_pos;
    unsigned char end_of_list_reached = 0;

    if (rgb_ctrl.active_time == 0xFFFF) return;    // inactive - do nothing

    rgb_ctrl.active_time++;
    
    // check, if next curve point is reached
    myindex = rgb_ctrl.fade_index;
    
    if ((rgb_ctrl.fade[myindex].time * rgb_ctrl.time_ratio) == rgb_ctrl.active_time)
      {
        myindex++;
        // new curve point reached, how to proceed?
        if (rgb_ctrl.fade[myindex].time == 0)
          {
            // end of list
            end_of_list_reached = 1;
            myindex--;                    // stay on last curve point             
          }
        else
          {
            rgb_ctrl.fade_index = myindex;        // save index
          } 
      }

    // now calc linear interpolation
    // val = val_prev + (dt / delta_t) * (val - val_prev)

    dt = rgb_ctrl.active_time - (int)rgb_ctrl.fade[myindex-1].time * rgb_ctrl.time_ratio;   // int

    delta_t = (int)(rgb_ctrl.fade[myindex].time - rgb_ctrl.fade[myindex-1].time)            // int
              *  rgb_ctrl.time_ratio;

    if (delta_t == 0) delta_t = 1;                                     // avoid div0 (this is dirty)

    //---->  make red

    delta_pos = ((int)rgb_ctrl.fade[myindex].red
                 - (int)rgb_ctrl.fade[myindex-1].red)   
                * CALC_GAIN;                                            // int (with gain 128)
                                                                        // range -32640 ... 32640
    posl = (int32_t)delta_pos * dt;
        
    posl = posl / delta_t;                                              // interpolated value
    posl = posl + (int)rgb_ctrl.fade[myindex-1].red * CALC_GAIN;      // +offset (gain 128)
                                                                        // range 0 ... 32640
    // scale this fade value according to max volume
    // scaled = interpolated * max

    posl = posl * rgb_ctrl.redmax;                                            // range -2^29 ... 2^29

    // now scale to timer

    posl = posl / 256;
    posi = posl / CALC_GAIN;
                                          // range 0 ... 256
    set_R(posi);

    //---->  make green

    delta_pos = ((int)rgb_ctrl.fade[myindex].green
                 - (int)rgb_ctrl.fade[myindex-1].green)   
                * CALC_GAIN;                                            // int (with gain 128)
                                                                        // range -32640 ... 32640
    posl = (int32_t)delta_pos * dt;
        
    posl = posl / delta_t;                                              // interpolated value
    posl = posl + (int)rgb_ctrl.fade[myindex-1].green * CALC_GAIN;      // +offset (gain 128)
                                                                        // range 0 ... 32640
    // scale this fade value according to max volume
    // scaled = interpolated * max

    posl = posl * rgb_ctrl.greenmax;                                            // range -2^29 ... 2^29

    // now scale to timer

    posl = posl / 256;
    posi = posl / CALC_GAIN;
                                          // range 0 ... 256
    set_G(posi);

    //---->  make blue

    delta_pos = ((int)rgb_ctrl.fade[myindex].blue
                 - (int)rgb_ctrl.fade[myindex-1].blue)   
                * CALC_GAIN;                                            // int (with gain 128)
                                                                        // range -32640 ... 32640
    posl = (int32_t)delta_pos * dt;
        
    posl = posl / delta_t;                                              // interpolated value
    posl = posl + (int)rgb_ctrl.fade[myindex-1].blue * CALC_GAIN;      // +offset (gain 128)
                                                                        // range 0 ... 32640
    // scale this fade value according to max volume
    // scaled = interpolated * max

    posl = posl * rgb_ctrl.bluemax;                                            // range -2^29 ... 2^29

    // now scale to timer

    posl = posl / 256;
    posi = posl / CALC_GAIN;
                                          // range 0 ... 256
    set_B(posi);


    if (end_of_list_reached == 1)
      {
        // now decide further processing

        if (rgb_ctrl.repeat)
          {
            rgb_ctrl.repeat--;
            if (rgb_ctrl.repeat == 0)
              { // all repeats done, stop now
                rgb_ctrl.active_time = 0xFFFF;
              }
            else
              {
                rgb_ctrl.fade_index = 1;                // restart
                rgb_ctrl.active_time = 0;
              }
          }
        else
          {
            rgb_ctrl.fade_index = 1;                // restart
            rgb_ctrl.active_time = 0;
          }
      }              
    return;
  }



//-------------------------------------------------------------------------------
static signed char last_rgb_run;   // timer variable to create a update grid;

// Multitask replacement, must be called in a loop
void run_rgb_fader(void)
  {
    switch (rgb_state)
      {
        case IDLE:
            last_rgb_run = timerval;                    // remember time 
            rgb_state = RGB_WAIT_TICK;
            break;

        case RGB_WAIT_TICK:
            // note: cast the difference down to char, otherwise the wrap around fails
            
            #if (SIMULATION == 0)
                if ((char)(timerval - last_rgb_run) < (RGB_UPDATE_PERIOD / TICK_PERIOD))  return;
            #endif

            // 20ms passed, now calc new rgb values

            last_rgb_run = timerval;              // remember time 
            
            // check for an update of fade values
             
            calc_rgb_next_val();
            break;
     }
  }

void rgb_direct_action(unsigned int Command)
  {
    unsigned char myCommand, myTurnout;
    myCommand = Command & 0b00000111;
    myTurnout = myCommand >> 1;

    if (Command > 7) return;
    
    switch(myCommand)
      {
        case 0:
            set_R(0);
            break;
        case 1:
            set_R(my_eeprom_read_byte(&CV.REDmax));
            break;
        case 2:
            set_G(0);
            break;
        case 3:
            set_G(my_eeprom_read_byte(&CV.GREENmax));
            break;
        case 4:
            set_B(0);
            break;
        case 5:
            set_B(my_eeprom_read_byte(&CV.BLUEmax));
            break;
        default:
            break; // ignore all other commands
      }
  }



void rgb_action(unsigned int Command)
  {
    unsigned char myCommand, myTurnout;
    myCommand = Command & 0b00000111;
    myTurnout = myCommand >> 1;

    if (Command > 7) return;
    
    switch(myCommand)
      {
        case 0:
            stop_rgb_fade();
            set_R(10);
            break;
        case 1:
            do_rgb_fade();
            break;
        case 2:
            stop_rgb_fade();          // start fade
            servo_action(0);        // start servos
            servo_action(2);
            break;
        case 3:
            do_rgb_fade();          // start fade
            servo_action(1);        // start servos
            servo_action(3);
            break;

        default:
            break; // ignore all other commands
      }
  }



void init_rgb(void)
  {
    rgb_state = IDLE;
    rgb_ctrl.active_time = 0xFFFF;
    init_rgb_timer();

    set_R(0);   //   red_value
    set_G(100);  //   green_value;
    set_B(10);  //   blue_value;
  }

void rgb_simu(void)
  {
    unsigned char i;
    do_rgb_fade();
    for (i=0; i < 100; i++)
      {
        PORTA = 12;
        calc_rgb_next_val();
        PORTA = 13;
      }
  }

#else
     #error RGB_ENABLED, but not supported on TARGET_HARDWARE
#endif // (TARGET_HARDWARE == OPENDECODER25)
#endif // (RGB_ENABLED == 1)
#if (TRUE == 0) 
   #error Oops, TRUE seems not defined
#endif
