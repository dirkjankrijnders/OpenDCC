//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2006, 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      servo.c
// author:    Wolfgang Kufer (kw)
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-16 V0.01 kw:started (ideas copied from dmxout.c of opendcc)
//            2007-02-19 V0.02 kw:servo control changed to a list of
//                                time - position tags
//                                added scaling of the control list.
//            2007-03-09 V0.03 kw:added recall of start position
//                                in init routines
//            2007-04-01 V0.04 kw added english text and loading of eeprom curves
//            2007-04-11 V0.05 kw added trace flags for position remembering
//            2007-04-18 V0.06 kw changed runs A and B
//            2007-04-24 V0.07 kw tested: Segment Mode!
//            2007-05-13 V0.08 kw added Servo Repeat, changed min and max to 16 bit
//                                added code template for multiposition
//            2007-05-22 V0.09 kw added Pos_Mode Codes
//                                -- first release!
//            2007-05-30 V0.10 kw move readout of min and max moved to init
//            2007-06-06 V0.11 kw bug fix in save last. pos.
//                                values are always taken from eeprom to get an
//                                dynamic update.
//                                Inverted OCR-Handling to allow for turnoff
//            2007-08-06 V0.12 kw bug fix with increment position (unwanted cast of compiler)
//            2007-12-31          FLASH_DURING_MOVE for Photograph added
//            2008-09-26          extended Servo range
//            2008-12-15 V0.13 kw bugfix in extended Servo range
//            2012-02-12 V0.14    added SWITCH_4567
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: servo control with interpolation
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
// used:      Timer 1, OCR registers
//            OUTPUT_PORT (all bits)
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

#include "main.h"
#include "servo.h"

#if (SERVO_ENABLED == TRUE)

#define FLASH_DURING_MOVE   FALSE  // TRUE: put a flashing light on port B0 during move

#define SIMULATION  0               // 0: real application
                                    // 1: test servo call

#if (SIMULATION != 0)
   #warning servo SIMULATION is on! - do not use on real hardware!
   // a note on simulation: AVR Studio does NOT correctly simulate OCR1 (only as 9 Bit values)
#endif

//---------------------------------------------------------------------
// Timing Definitions:
// (all values given in us)

#define TICK_PERIOD       20000L    // 20ms tick for Timing Engine
                                    // => possible values for timings up to
                                    //    5.1s (=255/0.020)
                                    // note: this is also used as frame for
                                    // Servo-Outputs (OCR1A and OCR1B)


#define UPDATE_PERIOD     20000L    // 20ms -> 50Hz
#define CTRL_PERIOD      100000L    // 0.1s resolution -> 25 sec max.

// Timing Borders for Servo Pulse

#define SERVO_MINTIME      1000L    // 1ms
#define SERVO_INTERVAL     1000L    // 1ms

// defines for the Timer and the OCR
// note: only valid for "rounded" F_CPU values
#define T1_PRESCALER   8 

#define SERVO_MIN          (F_CPU / 1000000L * SERVO_MINTIME / T1_PRESCALER) 
#define SERVO_DELTA        (F_CPU / 1000000L * SERVO_INTERVAL / T1_PRESCALER) 
#define SERVO_MAX          (F_CPU / 1000000L * (SERVO_MINTIME+SERVO_INTERVAL) / T1_PRESCALER) 


//---------------------------------------------------------------------
// Hardware dependent options
//
// Opendecoder2:    use commands 4,5,6,7 as normal outputs, permanently switched
// Opendecoder3:    use commands 4,5,6,7 to adjust servo endposition
//

#if (TARGET_HARDWARE == OPENDECODER2)
    // #define USE4567_FOR_ADJUST   FALSE
    #define USE4567_FOR_ADJUST   TRUE
#elif (TARGET_HARDWARE == OPENDECODER25) 
    #define USE4567_FOR_ADJUST   TRUE 
#elif (TARGET_HARDWARE == OPENDECODER28) 
    #define USE4567_FOR_ADJUST   TRUE 
#elif (TARGET_HARDWARE == OPENDECODER3) 
    #define USE4567_FOR_ADJUST   TRUE 
#endif
#ifndef USE4567_FOR_ADJUST
    #define USE4567_FOR_ADJUST   FALSE 
#endif 

#define SWITCH_4567         TRUE // FALSE        // die Ausgänge 4567 werden bei manual adjust
                                        // als permanentes Paar mitgeschaltet

//------------------------------------------------------------------------------
// internal, but static:
enum servo_states
  {                                 // actual state
     IDLE,
     WF_INIT_DONE,                  // wait for init done
     WF_TIMESLOT,                   // wait for next active slot
  } servo_state;


// Trick:   Speed up port access by declaring port access as inline code
//          This is done with a subroutine; if gcc runs with -os, this results in
//          single cbi and sbi statements!
// Note:    a) OUTPUT_PORT is defined in hardware.h
//          b) we use always PBxx as mask, this is just a define to the number and
//             can be used on PORTC as well!

void my_output(unsigned char port_no, unsigned char state)
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
      

//==============================================================================
//
// Section 2
//
// Servo Engine
//
//------------------------------------------------------------------------------
// Definitionen zur Servo Programmierung (english text follows)
//
// Bewegung:    Die Auflösung eines Servos ist ca. 0,2% bei 90° Weg. 
//              Wir arbeiten mit etwa 1us Auflösung am Timer, damit
//              ergeben sich für den Bereich 1ms bis 2ms 1000 Steps.
//              (Also etwa 0,1%).
//              Die Start- und Zielposition wird als unsigned char abgelegt,
//              was einer Genauigkeit von 0,3° entspricht.
//              Die Bewegungskurve wird als normierte (Bereich 0..255) Liste
//              von Zeit- und Positionspunkten abgelegt;
//
// Position:    Diese Kurve wird bei der Ausgabe mit den für diesen
//              Servo geltenden Endanschlägen umgerechnet.
//
// Zeit:        Die Normkurve wird in Einheiten von 0,02s abgelegt, Zeit als
//              unsigned char -> damit sind 5,12s erfassbar.
//              Zusätzlich gibt es noch einen Streckungsfaktor,
//              (time_ratio) mit dem die Normkurve zeitlich gestreckt werden
//              kann.
//
// Steuerung:   active_time: Zeit = 0: frisch begonnen
//                                = 0xffff: beendet
//              curve_index: zeigt auf den nächsten Zielpunkt, beim Start somit
//                           auf 1.
//
// Am Ende:     Der letzte Kurvenpunkt ist (t=0, pos=void).
//              servo.control bestimmt, wie es weitergeht:
//              Bit meaning
//               0: shows act position: 0=pre A, 1=pre B movement
//               6: 0=single movement; position is saved after movement 
//                  1=forever
//               7: flag: terminate after next B
//
//------------------------------------------------------------------------------
// Definitions for Servo Programming
//
// Movement:    The resolution of a servos is approx. 0.2% for 90° travel.
//              The timer is running at 1us resolution, thus the range
//              from 1ms to 2ms will have 1000 steps (~ 0,1%).
//              The start and destination is stored as unsigned char.
//              which corresponds to an accuracy of 0.3°.
//              The whole curve for movement is stored as normalized list
//              of time and position points.
//
// Position:    The (ideal) curve above ist ranged during output with the
//              current valid limts.
//
// Time:        The time in the curve is stored in ticks, 0.02s each.
//              The value is stored as unsigned char -> resultiong range 5.12s.
//              Additionally there is stretch factor (time_ratio) for extending
//              the time of the curve.
//
// Control:     active_time: time = 0: just started
//                                = 0xffff: finished
//              curve_index: points to next local target (this is 1 at start)
//
// At End:      The last curvepoint is always (t=0, pos=void).
//              servo.control selects further processing:
//              Bit meaning
//               0: shows act position: 0=pre A, 1=pre B movement
//               6: 0=single movement; position is saved after movement 
//                  1=forever
//               7: flag: terminate after next B
//
//-------------------------------------------------------------------------------
//
//   Data fields 
//
//   servo_var:    'live' array for calculating data
//   servo_ctrl:   array with control values (pairs of position and time)
//
//--------------------------------------------------------------------------------
//

#define NO_OF_SERVOS          2         // number of Servos 
                                        
#define SIZE_SERVO_CURVE      24         // number of entries (pairs) in one control

typedef struct    // curve, normalized [0..255]
      {
        unsigned char time;            
        unsigned char position;       
      } t_curve_point;

//----------------------------------------------------------------------------------------
// 
// Predefined Curves
//
// Note on Scaling: 
//  x: this is the time scaled in integer steps. When the curve is
//     processed, the time is stretched by servo.time_ratio.
//  y: Independant of the actual positioning the predefined curves
//     always have a range from 0 to 255. When the curve is
//     processed, this is scaled between MIN and MAX of this servos.
//     For convienience and after whip, all curves should have
//     25 and 230 resp. as endpoints.
//
// These curves are created with an excel-sheet: servo_curve.xls
//
// -------------------------------------------------------------------------------------
//    Name         Decription                    min  max   start  end    time
// -------------------------------------------------------------------------------------
//  5 lin_A      | linear move, part A            25  230      25  230    80ms
//  6 lin_B      | linear move, part B            25  230     230   25    80ms
//  7 move_A     | smooth move, part A (cos)      25  230      25  230   180ms
//  8 move_B     | smooth move, part B (cos)      25  230     230   25   180ms
//  9 sine_A     | sinosoidal wave, part A       128  255     128  128   400ms
// 10 sine_B     | sinosoidal wave, part B         1  128     128  128   400ms 
// 11 whip_A     | parabola, part A              128  255     128  128   320ms
// 12 whip_B:    | parabola, part B                1  128     128  128   320ms 
// 13 sig_hp0:   | close flag, after-whip          1  240     230   25   700ms
// 14 sig_hp1:   | open flag, after-whip           1  240      25  230   700ms
// 15 sig_hp1p:  | open flag, pause, after-whip    1  240      25  230  1280ms
//

const t_curve_point lin_A[] PROGMEM = 
  {
    { 0 , 25 },
    { 2 , 128 },
    { 4 , 230 },
    { 0 , 0 },
  };

const t_curve_point lin_B[] PROGMEM = 
  {
    { 0 , 230 },
    { 2 , 128 },
    { 4 , 25 },
    { 0 , 0 },
  };

const t_curve_point move_A[] PROGMEM = 
  {
    { 0 , 25 },
    { 1 , 29 },
    { 2 , 39 },
    { 3 , 55 },
    { 4 , 77 },
    { 6 , 128 },
    { 8 , 180 },
    { 9 , 201 },
    { 10 , 217 },
    { 11 , 227 },
    { 12 , 231 },
    { 0 , 0 },
  };

const t_curve_point move_B[] PROGMEM = 
  {
    { 0 , 231 },
    { 1 , 227 },
    { 2 , 217 },
    { 3 , 201 },
    { 4 , 180 },
    { 6 , 128 },
    { 8 , 77 },
    { 9 , 55 },
    { 10 , 39 },
    { 11 , 29 },
    { 12 , 25 },
    { 0 , 0 },
  };

const t_curve_point sig_hp0[] PROGMEM = 
  {
    { 0 , 230 },
    { 1 , 224 },
    { 7 , 140 },
    { 9 , 89 },
    { 10 , 51 },
    { 11 , 26 },
    { 12 , 16 },
    { 13 , 15 },
    { 15 , 29 },
    { 16 , 33 },
    { 17 , 31 },
    { 19 , 21 },
    { 20 , 20 },
    { 22 , 27 },
    { 23 , 29 },
    { 25 , 26 },
    { 26 , 23 },
    { 27 , 23 },
    { 29 , 26 },
    { 30 , 27 },
    { 31 , 27 },
    { 33 , 24 },
    { 35 , 25 },
    { 0 , 0 },
  };

const t_curve_point sig_hp1[] PROGMEM = 
  {
    { 0 , 26 },
    { 11 , 115 },
    { 13 , 122 },
    { 16 , 128 },
    { 28 , 122 },
    { 40 , 230 },
    { 41 , 239 },
    { 42 , 240 },
    { 44 , 226 },
    { 45 , 222 },
    { 46 , 224 },
    { 48 , 234 },
    { 49 , 235 },
    { 51 , 228 },
    { 52 , 226 },
    { 54 , 230 },
    { 55 , 232 },
    { 56 , 232 },
    { 58 , 229 },
    { 59 , 228 },
    { 60 , 228 },
    { 62 , 231 },
    { 64 , 230 },
    { 0 , 0 },
  };



const t_curve_point sine_A[] PROGMEM = 
  { 
    { 0 , 128 },
    { 3 , 186 },
    { 5 , 218 },
    { 7 , 241 },
    { 8 , 249 },
    { 9 , 253 },
    { 10 , 255 },
    { 11 , 253 },
    { 12 , 249 },
    { 13 , 241 },
    { 15 , 218 },
    { 17 , 186 },
    { 20 , 128 },
    { 0 , 0 },
  };

const t_curve_point sine_B[] PROGMEM = 
  { 
    { 0 , 128 },
    { 3 , 70 },
    { 5 , 38 },
    { 7 , 15 },
    { 8 , 7 },
    { 9 , 3 },
    { 10 , 1 },
    { 11 , 3 },
    { 12 , 7 },
    { 13 , 15 },
    { 15 , 38 },
    { 17 , 70 },
    { 20 , 128 },
    { 0 , 0 },
  };
   


const t_curve_point whip_A[] PROGMEM = 
  { 
    { 0 , 128 },
    { 1 , 135 },
    { 2 , 145 },
    { 3 , 157 },
    { 4 , 172 },
    { 5 , 189 },
    { 6 , 208 },
    { 8 , 255 },
    { 10 , 208 },
    { 11 , 189 },
    { 12 , 172 },
    { 13 , 157 },
    { 14 , 145 },
    { 15 , 135 },
    { 16 , 128 },
    { 0 , 0 },
  };


const t_curve_point whip_B[] PROGMEM = 
  {
    {  0 , 128 },
    {  1 , 121 },
    {  2 , 111 },
    {  3 , 99 },
    {  4 , 84 },
    {  5 , 67 },
    {  6 , 48 },
    {  8 , 1 },
    { 10 , 48 },
    { 11 , 67 },
    { 12 , 84 },
    { 13 , 99 },
    { 14 , 111 },
    { 15 , 121 },
    { 16 , 128 },
    { 0 , 0 },
  };

const t_curve_point hp0[] PROGMEM = 
  { 
    { 0 , 230 },
    { 1 , 224 },
    { 7 , 140 },
    { 9 , 89 },
    { 10 , 51 },
    { 11 , 26 },
    { 12 , 16 },
    { 13 , 15 },
    { 15 , 29 },
    { 16 , 33 },
    { 17 , 31 },
    { 19 , 21 },
    { 20 , 20 },
    { 22 , 27 },
    { 23 , 29 },
    { 25 , 26 },
    { 26 , 23 },
    { 27 , 23 },
    { 29 , 26 },
    { 30 , 27 },
    { 31 , 27 },
    { 33 , 24 },
    { 35 , 25 },
    { 0 , 0 },
  };

const t_curve_point hp1[] PROGMEM = 
  { 
    { 0 , 26 },
    { 1 , 31 },
    { 7 , 115 },
    { 9 , 166 },
    { 10 , 204 },
    { 11 , 230 },
    { 12 , 239 },
    { 13 , 240 },
    { 15 , 226 },
    { 16 , 222 },
    { 17 , 224 },
    { 19 , 234 },
    { 20 , 235 },
    { 22 , 228 },
    { 23 , 226 },
    { 25 , 230 },
    { 26 , 232 },
    { 27 , 232 },
    { 29 , 229 },
    { 30 , 228 },
    { 31 , 228 },
    { 33 , 231 },
    { 35 , 230 },
    { 0 , 0 },
  };

const t_curve_point hp1p[] PROGMEM = 
  { 
    { 0 , 26 },
    { 11 , 115 },
    { 13 , 122 },
    { 16 , 128 },
    { 28 , 122 },
    { 40 , 230 },
    { 41 , 239 },
    { 42 , 240 },
    { 44 , 226 },
    { 45 , 222 },
    { 46 , 224 },
    { 48 , 234 },
    { 49 , 235 },
    { 51 , 228 },
    { 52 , 226 },
    { 54 , 230 },
    { 55 , 232 },
    { 56 , 232 },
    { 58 , 229 },
    { 59 , 228 },
    { 60 , 228 },
    { 62 , 231 },
    { 64 , 230 },
    { 0 , 0 },
  };

// eeprom curves -> they are defined in config.c - reason is the
// EEMEM handling of gcc

// this allocates 4 * 24 * 2 = 192 in EEPROM
// t_curve_point ee_curve1[SIZE_SERVO_CURVE] EEMEM;
// t_curve_point ee_curve2[SIZE_SERVO_CURVE] EEMEM;
// t_curve_point ee_curve3[SIZE_SERVO_CURVE] EEMEM;
// t_curve_point ee_curve4[SIZE_SERVO_CURVE] EEMEM;


t_curve_point *pre_def_curves[] =
  {
    (t_curve_point *) move_A,                                 //  0: reserved
    (t_curve_point *) EE_servo_curve1,      //  1: eeprom curve 1
    (t_curve_point *) EE_servo_curve2,      //  2: eeprom curve 2
    (t_curve_point *) EE_servo_curve3,      //  3: eeprom curve 3
    (t_curve_point *) EE_servo_curve4,      //  4: eeprom curve 4
    (t_curve_point *) lin_A,                                  //  5: linear A
    (t_curve_point *) lin_B,                                  //  6: linear B
    (t_curve_point *) move_A,                                 //  7: move A
    (t_curve_point *) move_B,                                 //  8: move B
    (t_curve_point *) sine_A,                                 //  9: sine A
    (t_curve_point *) sine_B,                                 // 10: sine B
    (t_curve_point *) whip_A,                                 // 11: whip A
    (t_curve_point *) whip_B,                                 // 12: whip B
    (t_curve_point *) hp0,                                    // 13: hp0
    (t_curve_point *) hp1,                                    // 14: hp1
    (t_curve_point *) hp1p,                                   // 15: hp1 with pause
  };    
                                   



//-----------------------------------------------------------------------------------
//
// Data Structure to handle servo movement


// bit field for servo.control:
#define SC_BIT_ACTUAL    0          // 0=pre or during A, 1=pre or during B movement
#define SC_BIT_MOVING    1          // 0=stopped, 1=moving
#define SC_BIT_OUT_CTRL  5          // 0=no Output Control, 1=Control corresponding output
#define SC_BIT_REPEAT    6          // 1=repeat mode
#define SC_BIT_TERMINATE 7

#define MOVE2A           0
#define MOVE2B           1

t_curve_point servo1_curve[SIZE_SERVO_CURVE]; 
t_curve_point servo2_curve[SIZE_SERVO_CURVE]; 


typedef struct
  { 
    unsigned int pulse_min;         // lowest pulse width [0..65535]
    unsigned int pulse_delta;       // range for pulse width [0..65535]
    unsigned int min;               // lower move limit [0..65535]
    unsigned int max;               // upper move limit [0..65535]
    
    unsigned char control;          // Bit 0: ACTUAL:   0=pre A, 1=pre B movement
                                    // Bit 1: MOVING:   0=at endpoint, 1=currently moving
                                    // Bit 5: OUT_CTRL: 0=no, 1=yes
                                    // Bit 6: REPEAT:   0=single, 1=forever
                                    // Bit 7: TERMINATE: flag: terminate after next B

    unsigned char repeat;           // if REPEAT: this is the number of repeats to do

    unsigned char curve_index;      // points to actual target in curve

    t_curve_point *curve;           // curve, normalized [0..255]
      
    unsigned int active_time;       // runtime: relative time to start point
                                    // 0        = restart Servos
                                    // 0xffff   = finished

    unsigned char time_ratio;       // ratio between runtime and curve time
  } t_servo;



t_servo servo[NO_OF_SERVOS] =
  { { SERVO_MIN,
      SERVO_DELTA,
          0,            // unsigned int min;  
      65000,            // unsigned int max; 
          0,            // unsigned char control;
          0,            // unsigned char repeat;
          1,            // unsigned char curve_index;
      servo1_curve,     // pointer to curve
          0,            // time
          1,            // ratio of curve
    },
    { SERVO_MIN,
      SERVO_DELTA,
       1000,            // unsigned int min;  
      50000,            // unsigned int max; 
        0,              // unsigned char control;
        0,              // unsigned char repeat;
        1,              // unsigned char curve_index;
      servo2_curve,     // pointer to curve
        0,              // time
        1,              // ratio of curve
    },
  };

// Note on setting: if a zero is output to OCR, this will give small spike
// Therefore we invert all:
// clear output at TOP and set at OCR; writing a value equal to TOP will give a all zero output.
// -> OCR1 = TOP - ocrval
//

// see port_engine.c!
#define T1_PRESCALER  8
#define TOPVAL   (F_CPU / 1000000L * TICK_PERIOD / T1_PRESCALER) 

void set_servo_valA(unsigned int ocrval)
  {
    OCR1A = TOPVAL - ocrval;
  }

void set_servo_valB(unsigned int ocrval)
  {
    OCR1B = TOPVAL - ocrval;
  }

void set_relais_for_actual(unsigned char index)
  {
    switch(index)
      {
        case 0:
            if (servo[0].control & (1<<SC_BIT_OUT_CTRL))
              {
                if (servo[0].control & (1<<SC_BIT_ACTUAL))  // we are pre B
                  {
                    my_output(PB0,1);                       // Output 0 on
                  }
                else
                  {
                    my_output(PB1,1);                       // Output 1 on
                  }
              }
            break;
        case 1:
            if (servo[1].control & (1<<SC_BIT_OUT_CTRL))
              {
                if (servo[1].control & (1<<SC_BIT_ACTUAL))  // we are pre B
                  {
                    my_output(PB2,1);                       // Output 0 on
                  }
                else
                  {
                    my_output(PB3,1);                       // Output 1 on
                  }
              }
            break;
      }
  }

//---------------------------------------------------------------------------------
// calc_servo_next_val(unsigned char nr)
//      generates the actual setting of OCR out of actual position.
//      does the following:
//      a) makes a linear interpolation on the loaded curve
//      b) check for total limits recalcs the interpolated value according to these
//         limits.
//      c) transforms the point to real servo position
//
//      This routine is to be called every timeslot (20ms).
//      
// Parameters:
//      nr: this servo is calculated
//      
// Return:
//      timer value (OCR1) of this servo; if retval = 0, movement is finished
//
// Scaling:
//      input positions (from curve table) are 8 bit;
//      intermediate interpolations are scaled up to 32 bits; headroom for signed types is reserved.
//      this scaling is neccesary to achieve a good resolution when moving;
//

#if (SIMULATION != 0)
volatile int16_t simint;  volatile int32_t simlong;
#endif

unsigned int calc_servo_next_val(unsigned char nr)
  {
    unsigned char myindex;        // index in curve
    int16_t posi;
    int16_t dt, delta_t;          // time delta always > 0
    int32_t posl;
    int16_t delta_pos;
    unsigned char end_of_list_reached = 0;

    if (servo[nr].active_time == 0xFFFF) return(0);    // inactive - do nothing

    servo[nr].active_time++;
    
    // check, if next curve point is reached
    myindex = servo[nr].curve_index;
    
    if ((servo[nr].curve[myindex].time * servo[nr].time_ratio) == servo[nr].active_time)
      {
        myindex++;
        // new curve point reached, how to proceed?
        if (servo[nr].curve[myindex].time == 0)
          {
            // end of list
            end_of_list_reached = 1;
            myindex--;                    // stay on last curve point             
          }
        else
          {
            servo[nr].curve_index = myindex;        // save index
          } 
      }

    // now calc linear interpolation
    // pos = pos_prev + (dt / delta_t) * (pos - pos_prev)

    dt = servo[nr].active_time - (int)servo[nr].curve[myindex-1].time * servo[nr].time_ratio;   // int

    delta_t = (int)(servo[nr].curve[myindex].time - servo[nr].curve[myindex-1].time)         // int
              *  servo[nr].time_ratio;

    if (delta_t == 0) delta_t = 1;                                     // avoid div0, this is dirty

    delta_pos = ((int)servo[nr].curve[myindex].position
                 - (int)servo[nr].curve[myindex-1].position)   
                * 128L;                                                 // int (with gain 128)
                                                                        // range -32640 ... 32640
    posl = (int32_t)delta_pos * dt;
        
    posl = posl / delta_t;                                              // interpolated value
    posl = posl + (int)servo[nr].curve[myindex-1].position * 128L;      // +offset (gain 128)
                                                                        // range 0 ... 32640
    // now correct for 255 as full scale of curve

    posl = posl * 256L;
    posl = posl / 255L;                                                 // range 0 ... 32768 (2^15)

    // scale this position according to min/max
    // scaled = min + interpolated * (max-min)

    delta_pos = (int)(servo[nr].max / 4) - (int)(servo[nr].min / 4);    // range -16384 ... 16384 (2^14)

    posl = posl * delta_pos;                                            // range -2^29 ... 2^29

    posl = posl + (servo[nr].min * (8192L));                            // gain: 13 Bits => range 2^29

    posl = posl / (16384L);                                              

    // now scale to timer

    posi = (int16_t) posl;                                  // range 0 ... 32767
    posl = (int32_t) posi * servo[nr].pulse_delta;          // range 0 ... 32767000
    posi = posl >> 15;                                      // range 0 ... 1000  
    posi = posi + servo[nr].pulse_min;                      //      + SERVO_MIN;  // add 1ms

    if (end_of_list_reached == 1)
      {
        // now decide further processing

        if (servo[nr].control & (1<<SC_BIT_ACTUAL))
          {                                                 // we did movement "B"
            
            servo[nr].control &= ~(1<<SC_BIT_ACTUAL);       // now we are pre A
            if (servo[nr].control & (1<<SC_BIT_REPEAT))
              {
                // repeatmode -> go to A (if no terminate is requested)
                if (servo[nr].control & (1<<SC_BIT_TERMINATE))
                  {
                    // terminate flag detected - stop now
                    servo[nr].control &= ~(1<<SC_BIT_TERMINATE);
                    servo[nr].control &= ~(1<<SC_BIT_MOVING);
                    servo[nr].active_time = 0xFFFF;         // terminate
                  }
                else
                  {
                    if (servo[nr].repeat)
                      {
                        servo[nr].repeat--;
                        if (servo[nr].repeat == 0)
                          { // all repeats done, stop now
                            servo[nr].control &= ~(1<<SC_BIT_TERMINATE);
                            servo[nr].control &= ~(1<<SC_BIT_MOVING);
                            servo[nr].active_time = 0xFFFF;
                          }
                        else
                          {
                            do_servo(nr, MOVE2A);               // goto A;
                          }
                      }
                    else
                      {
                        do_servo(nr, MOVE2A);                   // goto A;
                      }
                  }
              }
            else
              {
                servo[nr].control &= ~(1<<SC_BIT_MOVING);
                servo[nr].active_time = 0xFFFF;             // terminate 
                // now save position (0=pre A)
                if (nr == 0) 
                  {
                    my_eeprom_write_byte(&CV.Sv1_Loc, 0);
                    if (servo[nr].control & (1<<SC_BIT_OUT_CTRL))
                      {
                        my_output(PB1,1);                       // Output 1 on
                      }
                  }
                if (nr == 1)
                  {
                    my_eeprom_write_byte(&CV.Sv2_Loc, 0);
                    if (servo[nr].control & (1<<SC_BIT_OUT_CTRL))
                      {
                        my_output(PB3,1);                       // Output 3 on
                      }
                 }
              }
          }
        else
          {                                                 // we did movement "A"
            servo[nr].control |= (1<<SC_BIT_ACTUAL);        // now we are pre B
            
            if (servo[nr].control & (1<<SC_BIT_REPEAT))
              {
                // repeatmode -> go to B
                do_servo(nr, MOVE2B);
              }
            else
              {
                servo[nr].control &= ~(1<<SC_BIT_MOVING);   // stopped
                servo[nr].active_time = 0xFFFF;             // terminate
                // save position (1= pre B)
                if (nr == 0) 
                  {
                    my_eeprom_write_byte(&CV.Sv1_Loc, 1);
                    if (servo[nr].control & (1<<SC_BIT_OUT_CTRL))
                      {
                        my_output(PB0,1);                       // Output 0 on
                      }
                  }
                if (nr == 1)
                  {
                    my_eeprom_write_byte(&CV.Sv2_Loc, 1);
                    if (servo[nr].control & (1<<SC_BIT_OUT_CTRL))
                      {
                        my_output(PB2,1);                       // Output 2 on
                      }
                  }
              }
          }         
      }              
    return(posi);
  }



//---------------------------------------------------------------------------------
// calc_servo_single_val(unsigned char nr, unsigned char position)
//      generates the actual setting of OCR out of actual position.
//      does the following:
//      a) --
//      b) --
//      c) transforms the point to real servo position
//      
// Parameters:
//      nr: this servo is calculated
//      position: this the normalized position from curve
//      
// Return:
//      timer value (OCR1) of this servo; if retval = 0, movement is finished
//


unsigned int calc_servo_single_val(unsigned char nr, unsigned char position)
  {
    int16_t posi;
    int32_t posl;
    int16_t delta_pos;

    
    posl = (int)position * 128L;                                        // +offset (gain 128)
                                                                        // range 0 ... 32640

    // now correct for 255 as full scale of curve

    posl = posl * 256L;
    posl = posl / 255L;                                                 // range 0 ... 32768

    // scale this position according to min/max
    // scaled = min + interpolated * (max-min)

    delta_pos = (int)(servo[nr].max / 4) - (int)(servo[nr].min / 4);    // range -16384 ... 16384 (2^14)

    posl = posl * delta_pos;                                            // range -2^29 ... 2^29

    posl = posl + (servo[nr].min * (8192L));                            // gain: 13 Bits => range 2^29

    posl = posl / (16384L);                                                

    // now scale to timer

    posi = (int16_t) posl;                                  // range 0 ... 32767
    posl = (int32_t) posi * servo[nr].pulse_delta;          // range 0 ... 32767000
    posi = posl >> 15;                                      // range 0 ... 1000
    posi = posi + servo[nr].pulse_min;                      //      + SERVO_MIN;  // add 1ms

    return(posi);
  }


unsigned char read_curve_start(my_curve_ean)
  {
    t_curve_point *src;
    unsigned char i = 0;

    if (my_curve_ean >= (sizeof(pre_def_curves)/sizeof(pre_def_curves[0])))
        my_curve_ean = 5;  // default

    src = pre_def_curves[my_curve_ean];

    switch(my_curve_ean)
      {
        case 0: // reserved
        case 1: // eeprom
        case 2: // eeprom
        case 3: // eeprom
        case 4: // eeprom
            i = my_eeprom_read_byte(&src->position); 
            break;
        case 5: // programm 
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:           
            i = pgm_read_byte(&src->position);
            break;
      }
    return(i);
  }


#if (SIMULATION != 0)
unsigned int T1;  unsigned int T2;
#endif


void move_servo_to_start(unsigned char index)
  {
    unsigned char my_curve_ean;
    unsigned char start;
    switch(index)
      {
        case 0:
            if (servo[0].control & (1<<SC_BIT_ACTUAL)) // pre B 
              {
                my_curve_ean = my_eeprom_read_byte(&CV.Sv1_CurveB) & 0x7F;
              }
            else
              {
                my_curve_ean = my_eeprom_read_byte(&CV.Sv1_CurveA) & 0x7F;
              }
    
            start = read_curve_start(my_curve_ean);
            #if (SIMULATION != 0)
                T1 = start;
                T1 = calc_servo_single_val(0, start);
            #endif
            set_servo_valA(calc_servo_single_val(0, start));
            break;
        case 1:
            if (servo[1].control & (1<<SC_BIT_ACTUAL)) // pre B 
              {
                my_curve_ean = my_eeprom_read_byte(&CV.Sv2_CurveB) & 0x7F;
              }
            else
              {
                my_curve_ean = my_eeprom_read_byte(&CV.Sv2_CurveA) & 0x7F;
              }
    
            start = read_curve_start(my_curve_ean);
            #if (SIMULATION != 0)
                T2 = start;
                T2 = calc_servo_single_val(1, start);
            #endif
            set_servo_valB(calc_servo_single_val(1, start));
            break;
        default:
            break;
      }
  }


void load_min_max(void)
  {
    unsigned char control;

    control = my_eeprom_read_byte(&CV.Sv1_Mode);
    if (control & (1 << CVbit_SvMode_Stretch))
      {
        servo[0].pulse_delta = SERVO_DELTA * 2;
        servo[0].pulse_min = SERVO_MIN / 2;
      }
    else
      {
        servo[0].pulse_delta = SERVO_DELTA;
        servo[0].pulse_min = SERVO_MIN;
      }
    servo[0].min = my_eeprom_read_byte(&CV.Sv1_minL) + 256 * my_eeprom_read_byte(&CV.Sv1_min);
    servo[0].max = my_eeprom_read_byte(&CV.Sv1_maxL) + 256 * my_eeprom_read_byte(&CV.Sv1_max);

    control = my_eeprom_read_byte(&CV.Sv2_Mode);
    if (control & (1 << CVbit_SvMode_Stretch))
      {
        servo[1].pulse_delta = SERVO_DELTA * 2;
        servo[1].pulse_min = SERVO_MIN / 2;
      }
    else
      {
        servo[1].pulse_delta = SERVO_DELTA;
        servo[1].pulse_min = SERVO_MIN;
      }
    servo[1].min = my_eeprom_read_byte(&CV.Sv2_minL) + 256 * my_eeprom_read_byte(&CV.Sv2_min);
    servo[1].max = my_eeprom_read_byte(&CV.Sv2_maxL) + 256 * my_eeprom_read_byte(&CV.Sv2_max);
  }


void init_servo(void)
  {
    unsigned char location;
    unsigned char control;

    servo_state = IDLE;
    load_min_max();

    // try to find out actual position of Servo 1

    location = my_eeprom_read_byte(&CV.Sv1_Loc);

    if (location == 0)
      {
        servo[0].control &= ~(1<<SC_BIT_ACTUAL);                   // pre A 
      }
    else
      {
        servo[0].control |= (1<<SC_BIT_ACTUAL);                    // pre B 
      } 

    control = my_eeprom_read_byte(&CV.Sv1_Mode);
    if (control & (1 << CVbit_SvMode_OUT_CTRL))
      {
        servo[0].control |= (1<<SC_BIT_OUT_CTRL);        // set flag for output control
      }
    else
      {
        servo[0].control &= ~(1<<SC_BIT_OUT_CTRL);       // set flag for output control
      } 

    set_relais_for_actual(0); 
    move_servo_to_start(0);

    // try to find out actual position of Servo 2

    location = my_eeprom_read_byte(&CV.Sv2_Loc);

    if (location == 0)
      {
        servo[1].control &= ~(1<<SC_BIT_ACTUAL);                   // pre A 
      }
    else
      {
        servo[1].control |= (1<<SC_BIT_ACTUAL);                    // pre B 
      } 

    control = my_eeprom_read_byte(&CV.Sv2_Mode);
    if (control & (1 << CVbit_SvMode_OUT_CTRL))
      {
        servo[1].control |= (1<<SC_BIT_OUT_CTRL);        // set flag for output control
      }
    else
      {
        servo[1].control &= ~(1<<SC_BIT_OUT_CTRL);       // set flag for output control
      } 

    set_relais_for_actual(1); 
    move_servo_to_start(1);

    // now enable these OCR outputs
    // OC1A and OC1B are mapped to Timer (for Servo Operation)

    // busy waiting for a good moment to turn servo on
    
    signed char my_timerval;
    my_timerval = timerval;
    while (my_timerval == timerval); 

    TCCR1A |= (1 << COM1A1)          // compare match A
            | (1 << COM1A0)          // set OC1A/OC1B on Compare Match, clear OC1A/OC1B at TOP
            | (1 << COM1B1)          // compare match B
            | (1 << COM1B0);

    #if (SERVO_POWER_UP == SPU_SOFT_PWM)
        // use a software pwm to ramp up servo power supply very slowly, to avoid unintended moves
        // this is done by delayloops

        cli();
        // This is just a test - no success :-(
        
        #define SS_PWM 32               // resolution

        unsigned char pwm_i, ss_ontime; unsigned int j;
        
        for (ss_ontime = 1; ss_ontime < SS_PWM; ss_ontime++)
          {
            PORTE |= (1<<SERVO2);
            for (j = 0; j < 1000; j++)
              {
                for (pwm_i = 0; pwm_i < SS_PWM; pwm_i++)    // inner pwm loop: SS_PWM * 9 => 300 cycles -> 40us
                  {
                    if (pwm_i < ss_ontime) SERVO1_POWER_ON; 
                    else                   SERVO1_POWER_OFF;
                    _delay_loop_1(3);           // burn 9 cycles
                  }
              }
            PORTE &= ~(1<<SERVO2);
            for (j = 0; j < 255; j++)
              {
                for (pwm_i = 0; pwm_i < SS_PWM; pwm_i++)    // inner pwm loop: SS_PWM * 9 => 300 cycles -> 40us
                  {
                    if (pwm_i < ss_ontime) SERVO1_POWER_ON; 
                    else                   SERVO1_POWER_OFF;
                    _delay_loop_1(3);           // burn 9 cycles
                  }
              }
           }
        SERVO1_POWER_ON;
        sei();
        // SERVO2_POWER_ON;
    #elif (SERVO_POWER_UP == SPU_PULS_ON)
    #elif (SERVO_POWER_UP == SPU_PULS_OFF)
    #else
      #warning SERVO_POWER_UP method not defined 
    #endif


    #if (SIMULATION != 0)
      {unsigned char i;
        run_servo();
        run_servo();
        run_servo();
        run_servo();
        run_servo();
        run_servo();
        servo_action(1);             // run turnout 0 - green
        for (i=0; i<20; i++)
          {
            run_servo();
          }
          
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        for (i=0; i<20; i++)
          {
            run_servo();
          }
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        servo_action(4);             // run turnout 0 - green
        while(1)
          {
            run_servo();
          }
      }
    #endif
  }


//=================================================================================
//
// run_servo: multitask replacement, must be called in loop
//
//=================================================================================

signed char last_servo_run;   // timer variable to create a update grid;

#if (FLASH_DURING_MOVE == TRUE)
 unsigned char flash = 0;
 unsigned char flash_period = 0;
 unsigned char xxx[] = {".... Flashing Port 0 ..."};
#endif

// Multitask replacement, must be called in a loop
void run_servo(void)
  {
    unsigned char i;
    unsigned int ocrval;
    
    switch (servo_state)
      {
        case IDLE:
            last_servo_run = timerval;                    // remember time 
            servo_state = WF_INIT_DONE;
            break;

        case WF_INIT_DONE:
            // keep output for 500ms alive
            #if (SIMULATION == 0)
                if ((char)(timerval - last_servo_run) < (500000L / TICK_PERIOD))  return;
            #endif

            last_servo_run = timerval;                    // remember time 
            for (i=0; i<NO_OF_SERVOS; i++)
              {
                servo[i].active_time = 0xffff;
		      }
		    servo_state = WF_TIMESLOT;
            break;

        case WF_TIMESLOT:
            // note: cast the difference down to char, otherwise the wrap around fails
            
            #if (SIMULATION == 0)
                if ((char)(timerval - last_servo_run) < (UPDATE_PERIOD / TICK_PERIOD))  return;
            #endif

            // 20ms passed, now calc new servos values
            
            last_servo_run = timerval;              // remember time 
            
            // check for an update of positions
             
            ocrval = calc_servo_next_val(0);
            #if (SIMULATION != 0)
                T1 = ocrval;
            #endif
            if (my_eeprom_read_byte(&CV.Sv1_Mode) & (1 << CVbit_SvMode_KeepOn))
              {
                if (ocrval != 0) set_servo_valA(ocrval);     // keep old value in case of no movement
              }
            else
              {
                set_servo_valA(ocrval);                      // always update
              }

            ocrval = calc_servo_next_val(1);
            #if (SIMULATION != 0)
                T2 = ocrval;
            #endif
            if (my_eeprom_read_byte(&CV.Sv2_Mode) & (1 << CVbit_SvMode_KeepOn))
              {
                if (ocrval != 0) set_servo_valB(ocrval);     // keep old value in case of no movement
              }
            else
              {
                set_servo_valB(ocrval);                      // always update
              }

            // Flasher
            #if (FLASH_DURING_MOVE == TRUE)
            if (flash == 1)
              {
                my_output(PB0,0);                                    // turn off
                flash = 0;
                flash_period = 20;            // 400ms
              }

            if (servo[0].control & (1<<SC_BIT_MOVING))
              {
                if (flash_period == 0)
                  {
                    flash = 1;
                    my_output(PB0,1);                                    // turn on
                  }
                else
                  {
                    flash_period--;
                  }
              }
            #endif

            break;
     }
  }


void copy_curve(unsigned char curve_ean, unsigned char myservo)
  {
    t_curve_point *dest;
    t_curve_point *src;
    unsigned char my_curve_ean;
    unsigned char i;

    my_curve_ean = curve_ean & 0x7F;
    if (my_curve_ean >= (sizeof(pre_def_curves)/sizeof(pre_def_curves[0])))
      my_curve_ean = 5;  // default

    dest = servo[myservo].curve;
    src = pre_def_curves[my_curve_ean];

    switch(my_curve_ean)
      {
        case 0: // reserved
        case 1: // eeprom
        case 2: // eeprom
        case 3: // eeprom
        case 4: // eeprom
            // copy one less, to keep zero at end of memory
            // thus the curve is always terminated 
            for (i=0; i<(SIZE_SERVO_CURVE-1); i++)
              {
                dest->time = my_eeprom_read_byte(&src->time);        
                dest->position = my_eeprom_read_byte(&src->position);
                dest++;
                src++;
              } 

            break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        default:
            // memcpy_P((void *)dest, (PGM_VOID_P)src, SIZE_SERVO_CURVE*sizeof(t_curve_point));
            // copy one less, to keep zero at end of memory
            for (i=0; i<(SIZE_SERVO_CURVE-1); i++)
              {
                dest->time = pgm_read_byte(&src->time);         // check end of list!!
                dest->position = pgm_read_byte(&src->position);
                dest++;
                src++;
              } 

            break;
      }
  }


#if (USE4567_FOR_ADJUST == TRUE) 
void increment_last_position(unsigned char index)
  {
    unsigned char manincr;
    unsigned int incr;
    unsigned long temp;

    manincr = my_eeprom_read_byte(&CV.ManIncr);
    if (manincr == 0) manincr=1;
    
    incr = manincr * 16;                      // 16 Step for full scale incr,
                                              // 4096 steps for small incr

    servo_state = IDLE;                       // reset servo engine to get a 500ms chain of pulses
    switch(index)
      {
        case 0:
            temp = (unsigned long)servo[0].min + (unsigned long)incr;
            if (temp < 65536L)
              {
                servo[0].min = (unsigned int)temp;
              }
            move_servo_to_start(0);
            break;
        case 1:
            temp = (unsigned long)servo[0].max + (unsigned long)incr;
            if (temp < 65536L)
              {
                servo[0].max = (unsigned int)temp;
              }
            move_servo_to_start(0);
            break;
        case 2:
            temp = (unsigned long)servo[1].min + (unsigned long)incr;
            if (temp < 65536L)
              {
                servo[1].min = (unsigned int)temp;
              }
            move_servo_to_start(1);
            break;
        case 3:
            temp = (unsigned long)servo[1].max + (unsigned long)incr;
            if (temp < 65536L)
              {
                servo[1].max = (unsigned int)temp;
              }
            move_servo_to_start(1);
            break;
        default:
            return;
      }
  }

void decrement_last_position(unsigned char index)
  {
    unsigned char manincr;
    unsigned int incr;

    manincr = my_eeprom_read_byte(&CV.ManIncr);
    if (manincr == 0) manincr=1;
    
    incr = manincr * 16;                      // 16 Step for full scale incr,
                                              // 4096 steps for small incr

    servo_state = IDLE;                       // reset servo engine to get a 500ms chain of pulses
    switch(index)
      {
        case 0:
            if (servo[0].min > incr) servo[0].min -= incr;
            move_servo_to_start(0);
            break;
        case 1:
            if (servo[0].max > incr) servo[0].max -= incr;
            move_servo_to_start(0);
            break;
        case 2:
            if (servo[1].min > incr) servo[1].min -= incr;
            move_servo_to_start(1);
            break;
        case 3:
            if (servo[1].max > incr) servo[1].max -= incr;
            move_servo_to_start(1);
            break;
        default:
            return;
      }
  }

void save_last_position(unsigned char index)
  {
    switch(index)
      {
        case 0:
            my_eeprom_write_byte(&CV.Sv1_minL, (unsigned char) servo[0].min);
            my_eeprom_write_byte(&CV.Sv1_min, (unsigned char) (servo[0].min >> 8));
            break;
        case 1:
            my_eeprom_write_byte(&CV.Sv1_maxL, (unsigned char) servo[0].max);
            my_eeprom_write_byte(&CV.Sv1_max, (unsigned char) (servo[0].max >> 8));
            break;
        case 2:
            my_eeprom_write_byte(&CV.Sv2_minL, (unsigned char) servo[1].min);
            my_eeprom_write_byte(&CV.Sv2_min, (unsigned char) (servo[1].min >> 8));
            break;
        case 3:
            my_eeprom_write_byte(&CV.Sv2_maxL, (unsigned char) servo[1].max);
            my_eeprom_write_byte(&CV.Sv2_max, (unsigned char) (servo[1].max >> 8));
            break;
        default:
            return;
      }
  }

unsigned char is_manual_adjust_allowed(unsigned char index)
  {
    unsigned char temp;
    switch(index)
      {
        case 0:
        case 1:
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_ADJ)) return(TRUE);
            break;
        case 2:
        case 3:
            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_ADJ)) return(TRUE);
            break;
        default:
            break;
      }
    return(FALSE);

  }



#endif


//---------------------------------------------------------------------------------------------
// parameter: nr: this servo
//            move: 0 = move to A ( MOVE2A )
//                  1 = move to B ( MOVE2B )

void do_servo(unsigned char nr, unsigned char move)
  {
    load_min_max();
    if (nr == 0)
      {
        if (servo[0].control & (1<<SC_BIT_OUT_CTRL))
          {
            my_output(PB0,0);                                    // turn off both
            my_output(PB1,0); 
          }  
        switch(move)
          {
            case MOVE2A: // Servo 1, move A
                copy_curve(my_eeprom_read_byte(&CV.Sv1_CurveA), 0);
                servo[0].time_ratio = my_eeprom_read_byte(&CV.Sv1_TimeA);
                servo[0].curve_index = 1;
                servo[0].active_time = 0;
                servo[0].control |= (1<<SC_BIT_MOVING);
                break;
            case MOVE2B: // Servo 1, move B
                copy_curve(my_eeprom_read_byte(&CV.Sv1_CurveB), 0);
                servo[0].time_ratio = my_eeprom_read_byte(&CV.Sv1_TimeB);
                servo[0].curve_index = 1;
                servo[0].active_time = 0;
                servo[0].control |= (1<<SC_BIT_MOVING);
                break;
          }
      }
     if (nr == 1)
      {
        if (servo[1].control & (1<<SC_BIT_OUT_CTRL))
          {
            my_output(PB2,0);                                    // turn off both
            my_output(PB3,0); 
          }  
        switch(move)
          {
            case MOVE2A: // Servo 2, move A
                copy_curve(my_eeprom_read_byte(&CV.Sv2_CurveA), 1);
                servo[1].time_ratio = my_eeprom_read_byte(&CV.Sv2_TimeA);
                servo[1].curve_index = 1;
                servo[1].active_time = 0;
                servo[1].control |= (1<<SC_BIT_MOVING);
                break;
            case MOVE2B: // Servo 2, move B
                copy_curve(my_eeprom_read_byte(&CV.Sv2_CurveB), 1);
                servo[1].time_ratio = my_eeprom_read_byte(&CV.Sv2_TimeB);
                servo[1].curve_index = 1;
                servo[1].active_time = 0;
                servo[1].control |= (1<<SC_BIT_MOVING);
                break;
          }
      }
  }


unsigned char LastServoCommand = 0;

void servo_action(unsigned int Command)
  {
    unsigned char myCommand; // unsigned char myTurnout;
    unsigned char temp;
    myCommand = Command & 0b00000111;
    // myTurnout = myCommand >> 1;

    if (Command > 7) return;
    
    switch(myCommand)
      {
        case 0:
            LastServoCommand = myCommand;
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_MOVMOD))
              {
                // Es ist ein Dauerlaeufer, dann nur ein Flag setzen,
                // dass es sauber heinlaufen soll.
                servo[0].control |= (1<<SC_BIT_TERMINATE);       // set flag for terminate
              }
            else
              {
                if ( ( (servo[0].control & (1<<SC_BIT_ACTUAL))) &&
                     (!(servo[0].control & (1<<SC_BIT_MOVING)))  ) 
                  {                             
                    // Servo steht auf anderer Seite und bewegt sich nicht, 
                    // also hier die andere Bewegung starten.
                    do_servo(0, MOVE2B);
                  }
              }
            break;
        case 1:
            LastServoCommand = myCommand;
            servo[0].repeat = my_eeprom_read_byte(&CV.Sv1_Repeat);
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_MOVMOD))
              {
                if ( (!(servo[0].control & (1<<SC_BIT_MOVING)))  )
                  {
                    servo[0].control &= ~(1<<SC_BIT_TERMINATE); // no terminate
                    servo[0].control |= (1<<SC_BIT_REPEAT);     // set flag for permanent
                    do_servo(0, MOVE2A);
                  }
              }
            else
              {
                if ( (!(servo[0].control & (1<<SC_BIT_ACTUAL))) &&
                     (!(servo[0].control & (1<<SC_BIT_MOVING)))  )
                  {                          // run A only if pre A and not moving
                    do_servo(0, MOVE2A);
                  }
              }
            break;

        case 2:
            LastServoCommand = myCommand;
            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_MOVMOD))
              {
                servo[1].control |= (1<<SC_BIT_TERMINATE);       // set flag for terminate
              }
            else
              {
                if ( ( (servo[1].control & (1<<SC_BIT_ACTUAL))) &&
                     (!(servo[1].control & (1<<SC_BIT_MOVING)))  ) 
                  {                             // run A only if pre A and not moving
                    do_servo(1, MOVE2B);
                  }
              }
            break;
        case 3:
            LastServoCommand = myCommand;
            servo[1].repeat = my_eeprom_read_byte(&CV.Sv2_Repeat);
            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_MOVMOD))
              {
                if ( (!(servo[1].control & (1<<SC_BIT_MOVING)))  )
                  {
                    servo[1].control &= ~(1<<SC_BIT_TERMINATE); // no terminate
                    servo[1].control |= (1<<SC_BIT_REPEAT);     // set flag for permanent
                    do_servo(1, MOVE2A);
                  }
              }
            else
              {
                if ( (!(servo[1].control & (1<<SC_BIT_ACTUAL))) &&
                     (!(servo[1].control & (1<<SC_BIT_MOVING)))  )
                  {
                    do_servo(1, MOVE2A);
                  }
              }
            break;
    #if (USE4567_FOR_ADJUST == FALSE) 
        case 4:
            my_output(PB4,1);
            my_output(PB5,0); 
            break;
        case 5:
            my_output(PB4,0);            
            my_output(PB5,1); 
            break;
        case 6:
            my_output(PB6,1);            
            my_output(PB7,0); 
            break;
        case 7:
            my_output(PB6,0);            
            my_output(PB7,1); 
            break;
    #else
        case 4:                                                 // decrement this endpoint
            if (is_manual_adjust_allowed(LastServoCommand))
                decrement_last_position(LastServoCommand);
            #if (SWITCH_4567 == TRUE) 
                else
                  {
                    my_output(PB4,1);
                    my_output(PB5,0); 
                  }
            #endif
            break;
        case 5:                                                 // increment this endpoint
            if (is_manual_adjust_allowed(LastServoCommand))
                increment_last_position(LastServoCommand);
            #if (SWITCH_4567 == TRUE) 
                else
                  {
                    my_output(PB4,0);
                    my_output(PB5,1); 
                  }
            #endif
            break;                                           
        case 6:
            if (is_manual_adjust_allowed(LastServoCommand))
                save_last_position(LastServoCommand);
            #if (SWITCH_4567 == TRUE) 
                else
                  {
                    my_output(PB6,1);
                    my_output(PB7,0); 
                  }
            #endif
            break;
        case 7:
            #if (SWITCH_4567 == TRUE) 
                my_output(PB6,0);            
                my_output(PB7,1);
            #endif
            break;

    #endif
        default:
            break; // ignore all other commands
      }
  }

void servo_key_action(unsigned int Command)                    // execute the key command
  {
    unsigned char temp;
    switch(Command)
      {
        case 0:
        case 1:
            temp = eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_MAN))
              {
                servo_action(Command);
              }
            break;
        case 2:
        case 3:
            temp = eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_MAN))
              {
                servo_action(Command);
              }
            break;
        default:
            break; // ignore all other commands
      }
  }




#if (SEGMENT_ENABLED == TRUE)
//---------------------------------------------------------------------------
// Ideas for controlling a segmented turntable
//
// Array of destinations:       table_position[0...7]
// 
// 


unsigned int table_position[8];

unsigned char last_position;            // this is our last location



// see servo_curve.xls on the definition of these curves

#define DIVISOR_LIN (16*256)    

const t_curve_point lin_curve[] PROGMEM = 
  {
    { 0 , 0 },
    { 15 , 255 },
    { 0 , 0 },
  };



#define DIVISOR_FAST (16*256)           // timescale of scale of curves is char
                                        // scale of min max is int.

const t_curve_point fast_curve[] PROGMEM = 
  {
    { 0 , 0 },
    { 1 , 1 },
    { 2 , 5 },
    { 3 , 12 },
    { 4 , 20 },
    { 5 , 31 },
    { 6 , 44 },
    { 8 , 75 },
    { 11 , 128 },
    { 14 , 180 },
    { 16 , 211 },
    { 17 , 224 },
    { 18 , 235 },
    { 19 , 243 },
    { 20 , 250 },
    { 21 , 254 },
    { 22 , 255 },
    { 0 , 0 },
  };


#define DIVISOR_SLOW (32*256)

const t_curve_point slow_curve[] PROGMEM = 
  {
    { 0 , 0 },
    { 1 , 1 },
    { 2 , 3 },
    { 3 , 6 },
    { 4 , 11 },
    { 5 , 17 },
    { 6 , 24 },
    { 8 , 42 },
    { 10 , 64 },
    { 12 , 88 },
    { 15 , 128 },
    { 18 , 167 },
    { 20 , 191 },
    { 22 , 213 },
    { 24 , 231 },
    { 25 , 238 },
    { 26 , 244 },
    { 27 , 249 },
    { 28 , 252 },
    { 29 , 254 },
    { 30 , 255 },
    { 0 , 0 },
  };


unsigned int get_position(unsigned char index)
  {
    void *eeptr;
    unsigned int pos;

    eeptr = &CV.Pos1_L + index * 2; 

    pos = my_eeprom_read_byte(eeptr);
    eeptr++;
    pos += 256 * my_eeprom_read_byte(eeptr);               // we make little endian
    return(pos);
  }


void init_segment(void)
  {
    unsigned char i;
    
    last_position = my_eeprom_read_byte(&CV.Last_Pos);        // recover last position

    for (i=0; i < 8; i++)
      {
        table_position[i] = get_position(i);
      }

    servo_state = IDLE;
    load_min_max();          // also get stretch

    // try to find out actual position of Servo 1

    servo[0].min = table_position[last_position];
    servo[0].max = table_position[last_position];

    set_servo_valA(calc_servo_single_val(0, 128));                  // dont care, min + max are equal

    // now enable these OCR outputs

    signed char my_timerval;
    my_timerval = timerval;
    while (my_timerval == timerval);    // wait for a good moment to enable pulses

    // OC1A and OC1B are mapped to Timer (for Servo Operation)

    TCCR1A |= (1 << COM1A1)          // compare match A
            | (1 << COM1A0)          // set OC1A/OC1B on Compare Match, clear OC1A/OC1B at TOP
            | (1 << COM1B1)          // compare match B
            | (1 << COM1B0);
  }


void increment_position(unsigned char index)
  {
    unsigned char manincr;
    unsigned int incr;
    unsigned long temp;

    manincr = my_eeprom_read_byte(&CV.ManIncr);
    if (manincr == 0) manincr=1;

    incr = manincr * 16;                      // 16 Step for full scale incr,
                                              // 4096 steps for small incr.
    temp = (unsigned long)table_position[index] + incr;

    if (temp < 65536L)
      {
        table_position[index] = temp;
      }
    
    servo[0].min = table_position[index];
    servo[0].max = table_position[index];

    set_servo_valA(calc_servo_single_val(0, 128));                  // dont care, min + max are equal
  }

void decrement_position(unsigned char index)
  {
    unsigned char manincr;
    unsigned int incr;

    manincr = my_eeprom_read_byte(&CV.ManIncr);
    if (manincr == 0) manincr=1;
    
    incr = manincr * 16;                      // 16 Step for full scale incr,
                                              // 4096 steps for small incr.
    if (table_position[index] > incr)
      {
        table_position[index] -= incr;
      }
    
    servo[0].min = table_position[index];
    servo[0].max = table_position[index];

    set_servo_valA(calc_servo_single_val(0, 128));                  // dont care, min + max are equal
  }

void save_position(unsigned char index)
  {
    void *eeptr;

    eeptr = &CV.Pos1_L + index * 2; 

    my_eeprom_write_byte(eeptr++, (unsigned char) table_position[index]);
    my_eeprom_write_byte(eeptr++, (unsigned char) (table_position[index] >> 8));

  }



//
// calc_curve generates a new servo curve, loads this curve
// to servo[0] and starts it.
unsigned char calc_curve(unsigned char start_i, unsigned char dest_i)
  {
    unsigned int mystart, mydest, delta;
    unsigned char scale;

    t_curve_point *src;
    t_curve_point *dest;
    

    mystart = get_position(start_i);
    mydest = get_position(dest_i);

    if (mydest == mystart)
      {  // no move --> exit here
         return(FALSE);
      }
    else if (mydest > mystart)
      {
        delta = mydest - mystart;
        servo[0].min = mystart;
        servo[0].max = mydest;
      }
    else
      {
        delta = mystart - mydest;
        servo[0].min = mystart;
        servo[0].max = mydest;
      }

    if (my_eeprom_read_byte(&CV.Pos_Mode) & (1 << CVbit_PosMode_SMTH))
      {                                                     // smooth curve
        if (delta < (64*256))
          {
            // take curve "fast" and set scaling 
            src = (t_curve_point *) fast_curve;
            scale = delta / DIVISOR_FAST;
            if (scale == 0) scale = 1;
          }
        else
          {
            // take curve "slow" and scale
            src = (t_curve_point *) slow_curve;
            scale = delta / DIVISOR_SLOW;
            if (scale == 0) scale = 1;
          }
      }
    else
      {                                                     // lin curve
        src = (t_curve_point *) lin_curve;
        scale = delta / DIVISOR_LIN;
        if (scale == 0) scale = 1;
      }


    // now scale and copy to servo array

    dest = servo[0].curve;

    unsigned char i;
    for (i=0; i<(SIZE_SERVO_CURVE-1); i++)
      {
        dest->time = scale * pgm_read_byte(&src->time);         // check end of list!!
        dest->position = pgm_read_byte(&src->position);
        dest++;
        src++;
      } 

    servo[0].time_ratio = my_eeprom_read_byte(&CV.Pos_Time);
    servo[0].curve_index = 1;
    servo[0].active_time = 0;
    servo[0].control |= (1<<SC_BIT_MOVING);

    return(TRUE);
  }



 
void servo_action2(unsigned int Command)
  {
    unsigned char myCommand;

    if (Command <= 7)
      {
        myCommand = Command & 0x07;
        if (!(servo[0].control & (1<<SC_BIT_MOVING)))
          {
            if (last_position != myCommand)
              {
                calc_curve(last_position,myCommand);
                last_position = myCommand;
                my_eeprom_write_byte(&CV.Last_Pos, last_position);
              }
          }
      }
    else
      {
        if (my_eeprom_read_byte(&CV.Pos_Mode) & (1 << CVbit_PosMode_ADJ))
          {                                                             // manual Adjustment via DCC enabled
            if (Command == 8)
              {
                // move this position down
                decrement_position(last_position);
              }
            else if (Command == 9)
              {
                // move this position up
                increment_position(last_position);
              }
            else if (Command == 10)
              {
                // save this position to EEPROM
                save_position(last_position);
              }
          }
      }
  }

#endif // SEGMENT_ENABLED

#endif  // SERVO_ENABLED




