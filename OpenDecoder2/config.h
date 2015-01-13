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
// file:      config.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-14 V0.1 kw start
//            2007-04-01 V0.2 kw added variables for Servo Curve
//            2007-05-07 V0.3 kw added Servo Repeat
//                               and parameters for multi positioning
//            2007-06-18 V0.4 kw added C_Tick
//            2007-09-18 V0.5 kw added extra CV for DMX (gap),
//                               no functional change
//            2008-12-14 V0.9 kw added variable scaling (min+delta instead SERVO_MAX)
//            2008-12-14 V0.11 kw added selectable power up sequence
//            2010-09-15 V0.12 kw added REVERSER_ENABLED
//            2011-12-08 V0.12 kw added Hardware OPENDECODER28
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            This is the central definition file for the project
//
//------------------------------------------------------------------------
//
// content:   1. Project settings
//               1.a) General
//               1.b) Configuration of Software Modules
//
//            2. EEPROM definitions (CV's)
//            3. Global variables
//            4. Useful inline routines (delay, Semaphor-operations)
//
//========================================================================
//
#ifndef _CONFIG_H_
#define _CONFIG_H_

//========================================================================
// 1. Project Definitions
//========================================================================
//
// 1.a) General
//
#define OPENDECODER_VERSION   0x21

// Definition of target hardware:
//   
#define OPENDECODER2 2
//  OPENDECODER2:  this board has:
//                 - ATmega8515 @ 8MHz
//                 - 2 Servo connectors
//                 - DMX (Tx and Rx),
//                 - 8 Powerports with integrated feedback
//                 see: www.opendcc.de
//                 (also 2.6)
//
#define OPENDECODER25 25
//  OPENDECODER2:  this board has:
//                 - ATmega8515 @ 8MHz or ATmega162 @ 10MHz
//                 - 2 Servo connectors
//                 - PWM output with MosFETs (for RGB)
//                 see: www.opendcc.de 
//
#define OPENDECODER28 28
//  OPENDECODER2:  this board has:
//                 - ATmega8515 @ 8MHz or ATmega162 @ 10MHz
//                 - 2 Servo connectors
//                 - PWM output with MosFETs
//                 - frog polarisation on PORTA; 4 tracers inputs like HW3
//                 see: www.opendcc.de 
//
#define OPENDECODER3 3
//  OPENDECODER3:  this board has:
//                 - ATmega162 @ 8MHz (2 Uarts)
//                 - 2 Servo connectors with polyfuse
//                 - complete BiDi / railcom interface
//                 - optional: DMX, Xpressnet, Loconet
//                 - 4 relais out, 4 tracers inputs
//                 see: www.opendcc.de and www.tams-online.de


#define TARGET_HARDWARE     OPENDECODER2


#define FALSE  0
#define TRUE   (!FALSE)


// Debugswitches:  0: no debugging, interrupts running -> alive!
//                 1: no ints, some lowlevel test

#define DEBUG  0

//-------------------------------------------------------------------------------------------
// 1.b) Configuration of Software Modules
//
// Note: depending on the desired target decoder, you may disable one of the
//       following enables.
//       Due to EEPROM and FLASH limitations, only one of the following is allowed:
//         SERVO or 
//         DMX   or
//         REVERSER
// 

#define PORT_ENABLED    FALSE        // TRUE: PortEngine does control the outputs
                                     // FALSE: PortEngine only does the timers

#define NEON_ENABLED    FALSE        // TRUE: NEON-Mode incuded (runs only on HW2)

#define SERVO_ENABLED   TRUE        // TRUE: include code and data for servo control

#define DMX_ENABLED     FALSE       // TRUE: DMX Decoder

#define RGB_ENABLED     FALSE       // TRUE: RGB control (requires mega162+HW 2.5 or HW2.8)

#define REVERSER_ENABLED   FALSE    // TRUE: include code and data for reverser relais control

#define SEGMENT_ENABLED   FALSE     // TRUE: include multi position Servodecoder


//-------------------------------------------------------------------------------------------
// Decoder Model Configuration Check

#if (REVERSER_ENABLED == TRUE)
  #if (SERVO_ENABLED == TRUE)
   #warning: cant do SERVO with REVERSER - SERVO has been disabled
   #undef SERVO_ENABLED
   #define SERVO_ENABLED   FALSE
  #endif
  #if (DMX_ENABLED == TRUE)
   #warning: cant do SERVO with REVERSER - DMX has been disabled
   #undef DMX_ENABLED
   #define DMX_ENABLED   FALSE
  #endif
  #if (PORT_ENABLED ==  FALSE)
   #warning: REVERSER needs output control, PORT must be enabled
   #undef PORT_ENABLED
   #define PORT_ENABLED  TRUE
  #endif
#endif
#if (DMX_ENABLED == TRUE)
  #if (SERVO_ENABLED == TRUE)
   #warning: cant do SERVO with DMX - SERVO has been disabled
   #undef SERVO_ENABLED
   #define SERVO_ENABLED   FALSE
  #endif
#endif

#if (SERVO_ENABLED == FALSE)
 #if (SEGMENT_ENABLED == TRUE)
  #warning: cant do SEGMENT without SERVO - SEGMENT has been disabled
  #undef SEGMENT_ENABLED
  #define SEGMENT_ENABLED   FALSE
 #endif
#endif

#if (TARGET_HARDWARE == OPENDECODER28)
   #if (PORT_ENABLED == TRUE)
     #warning: cant do PORT_ENGINE on OPENDECODER28
     #undef PORT_ENABLED
     #define PORT_ENABLED   FALSE
   #endif
#endif


//------------------------------------------------------------------------------------------
// Servo Power up

#define SPU_SOFT_PWM  1             // ramp up power supply slowly
#define SPU_PULS_ON   2             // puls = XXXXXXXXX_____X_____X_____X_____
#define SPU_PULS_OFF  3             // puls = ______________X_____X_____X_____

#define SERVO_POWER_UP       SPU_PULS_OFF   // SPU_SOFT_PWM    // SPU_PULS_OFF  

#if (SERVO_POWER_UP == SPU_SOFT_PWM)
    #define SERVO_INIT_PULS 0
#elif (SERVO_POWER_UP == SPU_PULS_ON)
    #define SERVO_INIT_PULS 1
#elif (SERVO_POWER_UP == SPU_PULS_OFF)
    #define SERVO_INIT_PULS 0
#else
    #warning SERVO_POWER_UP method not defined 
#endif


//========================================================================
// 2. EEPROM Definitions (CV's)
//========================================================================
//
// content is defined in config.c

#define CV_REMAPPING   TRUE              // if true: remap cv 513 to cv 1 aso;

#include "cv_define.h"

extern t_cv_record CV EEMEM;

extern const t_cv_record CV_PRESET PROGMEM;


/*
extern unsigned char EE_myAddrL    EEMEM; //513  M       Decoder Adresse low, 6 bits
extern unsigned char EE_auxActiv   EEMEM; //514  O       Auxiliary activation of outputs
extern unsigned char EE_T_on_F1    EEMEM; //515  O       Time on F1
extern unsigned char EE_T_on_F2    EEMEM; //516  O       Time on F2
extern unsigned char EE_T_on_F3    EEMEM; //517  O       Time on F3
extern unsigned char EE_T_on_F4    EEMEM; //518  O       Time on F4
extern unsigned char EE_version    EEMEM; //519  M       Version
extern unsigned char EE_VID        EEMEM; //520  M       Vendor ID (0x13 = DIY Decoder)
extern unsigned char EE_myAddrH    EEMEM; //521  M       Decoder Adresse high (3 bits)
extern unsigned char EE_522        EEMEM; //522  -       reserved
extern unsigned char EE_523        EEMEM; //523  -       reserved
extern unsigned char EE_524        EEMEM; //524  -       reserved
extern unsigned char EE_525        EEMEM; //525  -       reserved
extern unsigned char EE_526        EEMEM; //526  -       reserved
extern unsigned char EE_527        EEMEM; //527  -       reserved
extern unsigned char EE_528        EEMEM; //528  -       reserved
extern unsigned char EE_529        EEMEM; //529  -       reserved
extern unsigned char EE_530        EEMEM; //530  -       reserved
extern unsigned char EE_531        EEMEM; //531  -       reserved
extern unsigned char EE_532        EEMEM; //532  -       reserved
extern unsigned char EE_533        EEMEM; //533  -       reserved
extern unsigned char EE_534        EEMEM; //534  -       reserved
extern unsigned char EE_535        EEMEM; //535  -       reserved
extern unsigned char EE_536        EEMEM; //536  -       reserved
extern unsigned char EE_537        EEMEM; //537  -       reserved
extern unsigned char EE_538        EEMEM; //538  -       reserved
extern unsigned char EE_539        EEMEM; //539  -       reserved
extern unsigned char EE_BiDi       EEMEM; //540  -       Bi-Directional Communication Config
extern unsigned char EE_Config     EEMEM; //541  -       similar to CV#29; for acc. decoders
extern unsigned char EE_542        EEMEM; //542  -       reserved
extern unsigned char EE_543        EEMEM; //543  -       reserved
extern unsigned char EE_LastState  EEMEM; //544  -       saved last port state (was reserved)
// vendor specific 
extern unsigned char EE_MODE       EEMEM; //545  33  -      global decoder mode
                                                            // 00 = turnout decoder with feedback
                                                            // 01 = dual servo decoder
                                                            // 02 = multiposition servo decoder
                                                            // 08 = dmx
                                                            // 16 = light effects
extern unsigned char EE_FM         EEMEM; //546  34  -      global feedback mode
                                                            // 00 = no feedback
                                                            // 01 = positive acknowledge (pulsed)
                                                            // 02 = ALARM (permanent)
extern unsigned char EE_FBM_F1     EEMEM; //547  35  -      feedback mode Func 1
extern unsigned char EE_FBM_F2     EEMEM; //548  36  -      feedback mode Func 2
extern unsigned char EE_FBM_F3     EEMEM; //549  37  -      feedback mode Func 3
extern unsigned char EE_FBM_F4     EEMEM; //550  38  -      feedback mode Func 4

#if (SERVO_ENABLED == TRUE)
extern unsigned char EE_Sv1_minL   EEMEM; //551  39  -      Servo 1 Min (low part, reserved)
extern unsigned char EE_Sv1_min    EEMEM; //552  39  -      Servo 1 Min
extern unsigned char EE_Sv1_maxL   EEMEM; //553  40  -      Servo 1 Max (low part, reserved)
extern unsigned char EE_Sv1_max    EEMEM; //554  40  -      Servo 1 Max
extern unsigned char EE_Sv1_Mode   EEMEM; //555  41  -      Servo 1 Mode
 #define BIT_EE_SV_MOVMOD    0            //                   move mode (1=permanent)
 #define BIT_EE_SV_OUT_CTRL  1            //                   relais control
 #define BIT_EE_SV_MAN       2            //                   manual tracers allowed
 #define BIT_EE_SV_FEEDBACK  3            //                   feedback installed
 #define BIT_EE_SV_ADJ       4            //                   manual adjustment
extern unsigned char EE_Sv1_Repeat EEMEM; //556  41  -      Servo 1 Repeat
extern unsigned char EE_Sv1_Loc    EEMEM; //557             Servo 1 Location
extern unsigned char EE_Sv1_CurveA EEMEM; //558  ??  -      Servo 1 Curve Movement A
extern unsigned char EE_Sv1_TimeA  EEMEM; //559  ??  -      Servo 1 Curve Time stretch A
extern unsigned char EE_Sv1_CurveB EEMEM; //560  ??  -      Servo 1 Curve Movement A
extern unsigned char EE_Sv1_TimeB  EEMEM; //561  ??  -      Servo 1 Curve Time stretch B
extern unsigned char EE_Sv1_res    EEMEM; //562  ??  -      Servo 1 reserved

extern unsigned char EE_Sv2_minL   EEMEM; //563  ??  -      Servo 2 Min (low part, reserved)
extern unsigned char EE_Sv2_min    EEMEM; //564  ??  -      Servo 2 Min
extern unsigned char EE_Sv2_maxL   EEMEM; //565  ??  -      Servo 2 Max (low part, reserved)
extern unsigned char EE_Sv2_max    EEMEM; //566  ??  -      Servo 2 Max
extern unsigned char EE_Sv2_Mode   EEMEM; //567  ??  -      Servo 2 Mode
extern unsigned char EE_Sv2_Repeat EEMEM; //568  41  -      Servo 2 Repeat
extern unsigned char EE_Sv2_Loc    EEMEM; //569             Servo 2 Location
extern unsigned char EE_Sv2_CurveA EEMEM; //570  ??  -      Servo 2 Curve Movement A
extern unsigned char EE_Sv2_TimeA  EEMEM; //571  ??  -      Servo 2 Curve Time stretch A
extern unsigned char EE_Sv2_CurveB EEMEM; //572  ??  -      Servo 2 Curve Movement A
extern unsigned char EE_Sv2_TimeB  EEMEM; //573  ??  -      Servo 2 Curve Time stretch B
extern unsigned char EE_Sv2_res    EEMEM; //574  ??  -      Servo 2 reserved

extern unsigned char EE_575        EEMEM; //575  ??  -      reserved

// Variables for multiposition servo control 

extern unsigned char EE_Pos_Mode   EEMEM; //576  ??  -      Position Mode
 #define BIT_EE_POSMOD_SMTH    0          //                   smooth operation
 #define BIT_EE_POSMOD_ADJ     1          //                   manual adjustment
 #define BIT_EE_POSMOD_MAN     2          //                   manual tracers allowed
extern unsigned char EE_Pos_Time   EEMEM; //577  ??  -      Position Time stretch

extern unsigned char EE_Last_Pos   EEMEM; //578  ??  -      keep track of psoitions
extern unsigned char EE_ManIncr    EEMEM; //579  ??  -      Stepsize for manual control

extern unsigned char EE_Pos1_L     EEMEM; //580  ??  -      Servo 1 Position A, low part
extern unsigned char EE_Pos1_H     EEMEM; //581  ??  -      Servo 1 Position A, high part
extern unsigned char EE_Pos2_L     EEMEM; //582  ??  -      Servo 1 Position B, low part
extern unsigned char EE_Pos2_H     EEMEM; //583  ??  -      Servo 1 Position B, high part
extern unsigned char EE_Pos3_L     EEMEM; //584  ??  -      Servo 1 Position C, low part
extern unsigned char EE_Pos3_H     EEMEM; //585  ??  -      Servo 1 Position C, high part
extern unsigned char EE_Pos4_L     EEMEM; //586  ??  -      Servo 1 Position D, low part
extern unsigned char EE_Pos4_H     EEMEM; //587  ??  -      Servo 1 Position D, high part
extern unsigned char EE_Pos5_L     EEMEM; //588  ??  -      Servo 1 Position E, low part
extern unsigned char EE_Pos5_H     EEMEM; //589  ??  -      Servo 1 Position E, high part
extern unsigned char EE_Pos6_L     EEMEM; //590  ??  -      Servo 1 Position F, low part
extern unsigned char EE_Pos6_H     EEMEM; //591  ??  -      Servo 1 Position F, high part
extern unsigned char EE_Pos7_L     EEMEM; //592  ??  -      Servo 1 Position G, low part
extern unsigned char EE_Pos7_H     EEMEM; //593  ??  -      Servo 1 Position G, high part
extern unsigned char EE_Pos8_L     EEMEM; //594  ??  -      Servo 1 Position H, low part
extern unsigned char EE_Pos8_H     EEMEM; //595  ??  -      Servo 1 Position H, high part

extern unsigned char EE_596        EEMEM; //596  ??  -      reserved
extern unsigned char EE_597        EEMEM; //597  ??  -      reserved
extern unsigned char EE_598        EEMEM; //598  ??  -      reserved
extern unsigned char EE_599        EEMEM; //599  ??  -      reserved


extern unsigned char EE_servo_curve1[48] EEMEM;
extern unsigned char EE_servo_curve2[48] EEMEM;
extern unsigned char EE_servo_curve3[48] EEMEM;
extern unsigned char EE_servo_curve4[48] EEMEM;
#endif // SERVO_ENABLED

*/

#if (SERVO_ENABLED == TRUE)
extern unsigned char EE_servo_curve1[48] EEMEM;
extern unsigned char EE_servo_curve2[48] EEMEM;
extern unsigned char EE_servo_curve3[48] EEMEM;
extern unsigned char EE_servo_curve4[48] EEMEM;
#endif // SERVO_ENABLED

//========================================================================
// 3. Global variables
//========================================================================

//---------------------------------------------------------------------
// Timing Definitions:
//

#define TICK_PERIOD 20000L       // 20ms tick for Timing Engine
                                 // => possible values for timings up to
                                 //    5.1s (=255/0.020)
                                 // note: this is also used as frame for
                                 // Servo-Outputs (OC1A and OC1B)

//----------------------------------------------------------------------------

extern volatile signed char timerval;    // gets incremented in the timetick



//========================================================================
// 4. Project Definitions
//========================================================================
//

//-------------------------------------------------------------------------
// 4.a) Definitions for inter process communication
//-------------------------------------------------------------------------
// These routines are *inline*, so we keep them in the header.
//
// usage:   semaphor_set(flag, state) to do the communication.
//          semaphor_get(flag)




extern volatile unsigned char Communicate;


#define C_Received        0    // a new DCC was received - issued by ISR(Timer1)
                               //                          cleared by main
#define C_DoSave          1    // a new PORT state should be saved
                               //                        - issued by action
                               //                          cleared by main
#define C_Tick            2    // a Tickevent happened
  

//========================================================================
// 4.b) Useful inline code
//========================================================================

static inline unsigned char semaphor_query(unsigned char flag) 
       __attribute__((always_inline));

unsigned char semaphor_query(unsigned char flag)
  {
    return (Communicate & (1<<flag));
  }
               
static inline void semaphor_set(unsigned char flag) 
       __attribute__((always_inline));

void semaphor_set(unsigned char flag)
  {
    cli();
    Communicate |= (1<<flag);
    sei();
  }
       
static inline unsigned char semaphor_get(unsigned char flag) 
       __attribute__((always_inline));

unsigned char semaphor_get(unsigned char flag)
  {
    unsigned char value;
    cli();
    value = Communicate & (1<<flag);
    Communicate &= (~(1<<flag));
    sei();
    return(value);
  }
        

//------------------------------------------------------------------------
// Delay-Macro (all values in us) -> this is busy waiting
//------------------------------------------------------------------------
//
// same macro as in util/delay.h, but use up to some ms.
// The maximal possible delay is 262.14 ms / F_CPU in MHz.
// This is 16ms for 16MHz; longest used delay: 1000us

#ifndef _UTIL_DELAY_H_
  #include <util/delay.h>
#endif

static inline void _mydelay_us(double __us) __attribute__((always_inline));
void
_mydelay_us(double __us)
{
    uint16_t __ticks;
    double __tmp = ((F_CPU) / 4e6) * __us;
    if (__tmp < 1.0)
        __ticks = 1;
    else if (__tmp > 65535)
        __ticks = 0;    /* i.e. 65536 */
    else
        __ticks = (uint16_t)__tmp;
    _delay_loop_2(__ticks);
}   

static inline void _restart(void) __attribute__((always_inline));
void
_restart(void)
{
    cli();
                    
    // laut diversen Internetseiten sollte folgender Code laufen -
    // tuts aber nicht, wenn man das Assemblerlistung ansieht.
    // void (*funcptr)( void ) = 0x0000;    // Set up function pointer
    // funcptr();                        // Jump to Reset vector 0x0000
    
    __asm__ __volatile 
    (
       "ldi r30,0"  "\n\t"
       "ldi r31,0"  "\n\t"
       "icall" "\n\t"
     );
}


#endif   // _config_h_
