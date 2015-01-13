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
// file:      main.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-25 V0.01 kw start
//            2007-04-01 V0.02 kw added variables for Servo Curve
//            2007-05-07 V0.03 kw added Servo Repeat
//            2007-08-06 V0.04 changed to CV-struct
//            2010-09-14 V0.05 added reverser
//            2011-09-22 V0.06 RGB added
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: global variables (RAM and EEPROM)
//
//            Note: all eeprom variables must be allocated here
//            (reason: AVRstudio doesn't handle eeprom directives
//            correctly)
//
// content:   A DCC-Decoder for ATmega8515 and other AVR
//
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
#include "hardware.h"
#include "dcc_receiver.h"
//#include "port_engine.h"
// #include "servo.h"

//#include "main.h"

#define SIMULATION  0            // 0: real application
                                 // 1: test receive routine
                                 // 2: test timing engine
                                 // 3: test action

//----------------------------------------------------------------------------
// Timing Definitions:
// (note: every timing is given in us)

#define TICK_PERIOD 20000L       // 20ms tick for Timing Engine
                                 // => possible values for timings up to
                                 //    5.1s (=255/0.020)
                                 // note: this is also used as frame for
                                 // Servo-Outputs (OC1A and OC1B)

//----------------------------------------------------------------------------
// Global Data


volatile signed char timerval;          // generell timer tick, this is incremented
                                        // by Timer-ISR, wraps around

volatile unsigned char Communicate = 0; // Communicationregister (for semaphors)
   

//-----------------------------------------------------------------------------
// data in EEPROM:
// Note: the order of these data corresponds to physical CV-Address
//       CV1 is coded at #00
//       see RP 9.2.2 for more information



/*#if (SERVO_ENABLED == TRUE)
    const unsigned char compilat[] PROGMEM = {".... SERVO ..."};
      #warning Info: SERVO-Software included
    #if (SEGMENT_ENABLED == TRUE)
      const unsigned char compilat1[] PROGMEM = {"..+ SEG ..."};
      #warning Info: SEGMENT-Software included
#endif
#if (NEON_ENABLED == TRUE)
      const unsigned char compilat2[] PROGMEM = {"..+ NEON ..."};
      #warning Info: NEON-Software included
#endif
#if (PORT_ENABLED == TRUE)
      const unsigned char compilat3[] PROGMEM = {"..+ PORT ..."};
      #warning Info: PORT-Software included
#endif
#if (RGB_ENABLED == TRUE)
      const unsigned char compilat4[] PROGMEM = {"..+ RGB ..."};
      #warning Info: RGB-Software included
#endif
*/
    t_cv_record CV EEMEM =
      {
        #include "cv_data_servo.h"
      };

    unsigned char EE_servo_curve1[48] EEMEM;  // these are pairs of time/positions - see servo.c
    unsigned char EE_servo_curve2[48] EEMEM;
    unsigned char EE_servo_curve3[48] EEMEM;
    unsigned char EE_servo_curve4[48] EEMEM;

    const t_cv_record CV_PRESET PROGMEM =
      {
        #include "cv_data_servo.h"
      };
/*
#elif (DMX_ENABLED == TRUE)
   const unsigned char compilat[] PROGMEM = {".... DMX ...."};

    t_cv_record CV EEMEM =
      {
        #include "cv_data_dmx.h"
      };

    const t_cv_record CV_PRESET PROGMEM =
      {
        #include "cv_data_dmx.h"
      };

#elif (REVERSER_ENABLED == TRUE)
   const unsigned char compilat[] PROGMEM = {".... RELAY + REVERSER ...."};

    t_cv_record CV EEMEM =
      {
        #include "cv_data_reverser.h"
      };

    const t_cv_record CV_PRESET PROGMEM =
      {
        #include "cv_data_reverser.h"
      };

#elif (PORT_ENABLED == TRUE)
    const unsigned char compilat[] PROGMEM = {".... PORT ....."};

    t_cv_record CV EEMEM =
      {
        #include "cv_data_port.h"
      };

    const t_cv_record CV_PRESET PROGMEM =
      {
        #include "cv_data_port.h"
      };


#endif // SERVO_ENABLED
*/

/*

//            Name              default    CV  -alt  type    comment
unsigned char EE_myAddrL    EEMEM = 0x01; //513   1  M       Decoder Adresse low, 6 bits
unsigned char EE_auxActiv   EEMEM = 0x00; //514   2  O       Auxiliary activation of outputs: 0=no aux activation
unsigned char EE_T_on_F1    EEMEM =   15; //515   3  O       Time on F1
unsigned char EE_T_on_F2    EEMEM =   15; //516   4  O       Time on F2
unsigned char EE_T_on_F3    EEMEM =   15; //517   5  O       Time on F3
unsigned char EE_T_on_F4    EEMEM =   15; //518   6  O       Time on F4
unsigned char EE_version    EEMEM = 0x20; //519   7  M       Version
unsigned char EE_VID        EEMEM = 0x0D; //520   8  M       Vendor ID (0x0D = DIY Decoder)
                                         //                            (0x12 = JMRI)
unsigned char EE_myAddrH    EEMEM = 0x00; //521   9  M       Decoder Adresse high (3 bits)
unsigned char EE_522        EEMEM = 0x00; //522  10  -       reserved
unsigned char EE_523        EEMEM = 0x00; //523  11  -       reserved
unsigned char EE_524        EEMEM = 0x00; //524  12  -       reserved
unsigned char EE_525        EEMEM = 0x00; //525  13  -       reserved
unsigned char EE_526        EEMEM = 0x00; //526  14  -       reserved
unsigned char EE_527        EEMEM = 0x00; //527  15  -       reserved
unsigned char EE_528        EEMEM = 0x00; //528  16  -       reserved
unsigned char EE_529        EEMEM = 0x00; //529  17  -       reserved
unsigned char EE_530        EEMEM = 0x00; //530  18  -       reserved
unsigned char EE_531        EEMEM = 0x00; //531  19  -       reserved
unsigned char EE_532        EEMEM = 0x00; //532  20  -       reserved
unsigned char EE_533        EEMEM = 0x00; //533  21  -       reserved
unsigned char EE_534        EEMEM = 0x00; //534  22  -       reserved
unsigned char EE_535        EEMEM = 0x00; //535  23  -       reserved
unsigned char EE_536        EEMEM = 0x00; //536  24  -       reserved
unsigned char EE_537        EEMEM = 0x00; //537  25  -       reserved
unsigned char EE_538        EEMEM = 0x00; //538  26  -       reserved
unsigned char EE_539        EEMEM = 0x00; //539  27  -       reserved
unsigned char EE_BiDi       EEMEM = 0x00; //540  28  -       Bi-Directional Communication Config - keep at 0
unsigned char EE_Config     EEMEM =       //541  29  -       similar to CV#29; for acc. decoders
                                    (1<<7)                   // 1 = we are accessory
                                  | (0<<6)                   // 0 = we do 9 bit decoder adressing
                                  | (0<<5)                   // 0 = we are basic accessory decoder
                                  | 0;                       // 4..0: all reserved
unsigned char EE_542        EEMEM = 0x00; //542  30  -       reserved
unsigned char EE_543        EEMEM = 0x00; //543  31  -       reserved
unsigned char EE_LastState  EEMEM = 0x00; //544  32  -       saved last port state (was reserved)
//
// 545-593 - Manufacturer Unique
//
unsigned char EE_MODE       EEMEM = 0x01; //545  33  -      global decoder mode
                                                            // 00 = turnout decoder with feedback
                                                            // 01 = dual servo decoder
                                                            // 02 = multiposition servo decoder
                                                            // 03 = reserved
                                                            // ...
                                                            // 08 = dmx decoder
                                                            // 10 = signal decoder (tbd.)
                                                            // 16 = kirmes decoder
                                                            // 17 = direct motor control (kirmes)
unsigned char EE_FM         EEMEM = 0x01; //546  34  -      global feedback mode
                                                            // 00 = no feedback
                                                            // 01 = pos. feedback (ack)
                                                            // 02 = neg. feedback (emergency stop)
unsigned char EE_FBM_F1     EEMEM = 0x02; //547  35  -      feedback mode Func 1
                                                            // 00 = command acknowledge
                                                            // 01 = turnout feedback with intrinsic endswitches
                                                            // 02 = feedback with extra detector
                                                            // 03 = feedback with extra detector
unsigned char EE_FBM_F2     EEMEM = 0x02; //548  36  -      feedback mode Func 2
unsigned char EE_FBM_F3     EEMEM = 0x02; //549  37  -      feedback mode Func 3
unsigned char EE_FBM_F4     EEMEM = 0x00; //550  38  -      feedback mode Func 4

#if (SERVO_ENABLED == TRUE)
unsigned char EE_Sv1_minL   EEMEM =    0; //551  39  -      Servo 1 Min low
unsigned char EE_Sv1_min    EEMEM =  100; // 10; //552  40  -      Servo 1 Min high
unsigned char EE_Sv1_maxL   EEMEM =    0; //553  41  -      Servo 1 Max low
unsigned char EE_Sv1_max    EEMEM =  130; // 200;   //554  42  -      Servo 1 Max high
unsigned char EE_Sv1_Mode   EEMEM =    3; //555  43  -      Servo 1 Mode: Bit 0: 0 = A-B; 1=permanent
                                          //                              Bit 1: Output Control
                                          //                              Bit 2: Manual Operation (missing)
unsigned char EE_Sv1_Repeat EEMEM =    10; //556  44  -      Servo 1 Repeat: 0=forever
unsigned char EE_Sv1_Loc    EEMEM =    0; //557  45         Servo 1 Location: 0 = pre Move A; 1=pre move B
unsigned char EE_Sv1_CurveA EEMEM =    9;  // 5; //558  46  -      Servo 1 Curve Movement A
unsigned char EE_Sv1_TimeA  EEMEM = 1; //559  47  -      Servo 1 Curve Time stretch A
unsigned char EE_Sv1_CurveB EEMEM =    10; //6; //560  48  -      Servo 1 Curve Movement B
unsigned char EE_Sv1_TimeB  EEMEM = 1; //561  49  -      Servo 1 Curve Time stretch B
unsigned char EE_Sv1_res    EEMEM =    0; //562  50  -      Servo 1 reserved

unsigned char EE_Sv2_minL   EEMEM =    0; //563  51  -      Servo 2 Min low
unsigned char EE_Sv2_min    EEMEM =   10; //564  52  -      Servo 2 Min high
unsigned char EE_Sv2_maxL   EEMEM =    0; //565  53  -      Servo 2 Max low
unsigned char EE_Sv2_max    EEMEM =  200; //566  54  -      Servo 2 Max high
unsigned char EE_Sv2_Mode   EEMEM =    2; //567  55  -      Servo 2 Mode = 0=A-B 1=permanent
unsigned char EE_Sv2_Repeat EEMEM =    0; //568  56  -      Servo 2 Repeat
unsigned char EE_Sv2_Loc    EEMEM =    0; //569  57         Servo 2 Location = 0 = pre Move A; 1=pre move B
unsigned char EE_Sv2_CurveA EEMEM =   13; //570  58  -      Servo 2 Curve Movement A
unsigned char EE_Sv2_TimeA  EEMEM =    4; //571  59  -      Servo 2 Curve Time stretch A
unsigned char EE_Sv2_CurveB EEMEM =   15; //572  60  -      Servo 2 Curve Movement B
unsigned char EE_Sv2_TimeB  EEMEM =    5; //573  61  -      Servo 2 Curve Time stretch B
unsigned char EE_Sv2_res    EEMEM =    0; //574  62  -      Servo 2 reserved

unsigned char EE_575        EEMEM =    0; //575  63  -      reserved

// Variables for multiposition servo control 

unsigned char EE_Pos_Mode   EEMEM =0b111; //576  64  -      Position Mode
// #define BIT_EE_POSMOD_SMTH    0        //                - Smooth operation
// #define BIT_EE_POSMOD_ADJ     1        //                - Adjustment via DCC
// #define BIT_EE_POSMOD_MAN     2        //                - Manual Operation
unsigned char EE_Pos_Time   EEMEM =    5; //577  65  -      Position Time stretch

unsigned char EE_Last_Pos   EEMEM =    0; //578  ??  -      Last index (read only)
unsigned char EE_ManIncr    EEMEM =    1; //579  ??  -        1 = 4096 steps for full range
                                          //                255 = 16 step from min to max

unsigned char EE_Pos1_L     EEMEM =    0; //580  ??  -      Servo 1 Position A, low part
unsigned char EE_Pos1_H     EEMEM =   10; //581  ??  -      Servo 1 Position A, high part
unsigned char EE_Pos2_L     EEMEM =    0; //582  ??  -      Servo 1 Position B, low part
unsigned char EE_Pos2_H     EEMEM =   50; //583  ??  -      Servo 1 Position B, high part
unsigned char EE_Pos3_L     EEMEM =    0; //584  ??  -      Servo 1 Position C, low part
unsigned char EE_Pos3_H     EEMEM =  100; //585  ??  -      Servo 1 Position C, high part
unsigned char EE_Pos4_L     EEMEM =    0; //586  ??  -      Servo 1 Position D, low part
unsigned char EE_Pos4_H     EEMEM =  150; //587  ??  -      Servo 1 Position D, high part
unsigned char EE_Pos5_L     EEMEM =    0; //588  ??  -      Servo 1 Position E, low part
unsigned char EE_Pos5_H     EEMEM =  200; //589  ??  -      Servo 1 Position E, high part
unsigned char EE_Pos6_L     EEMEM =    0; //590  ??  -      Servo 1 Position F, low part
unsigned char EE_Pos6_H     EEMEM =  210; //591  ??  -      Servo 1 Position F, high part
unsigned char EE_Pos7_L     EEMEM =    0; //592  ??  -      Servo 1 Position G, low part
unsigned char EE_Pos7_H     EEMEM =  220; //593  ??  -      Servo 1 Position G, high part
unsigned char EE_Pos8_L     EEMEM =    0; //594  ??  -      Servo 1 Position H, low part
unsigned char EE_Pos8_H     EEMEM =  250; //595  ??  -      Servo 1 Position H, high part


unsigned char EE_596        EEMEM =    0; //596  ??  -      reserved
unsigned char EE_597        EEMEM =    0; //597  ??  -      reserved
unsigned char EE_598        EEMEM =    0; //598  ??  -      reserved
unsigned char EE_599        EEMEM =    0; //599  ??  -      reserved


unsigned char EE_servo_curve1[48] EEMEM;  // these are pairs of time/positions - see servo.c
unsigned char EE_servo_curve2[48] EEMEM;
unsigned char EE_servo_curve3[48] EEMEM;
unsigned char EE_servo_curve4[48] EEMEM;
#endif // SERVO_ENABLED


#if (DMX_ENABLED == TRUE)
unsigned char EE_DMX_MODE   EEMEM = 0x60; //551  35  -      DMX Mode
                                                            // Bit 7:  1 = single call to dmx_operation allowed
                                                            //        *0 = only base address calls
                                                            // Bit 6: *1 = initial state = all on
                                                            //         0 = initial state = all off
                                                            // Bit 5  *1 = Watchdog event turns on lights
                                                            //         0 = no reaction
                                                            // other: reserved
//unsigned char EE_552        EEMEM =    0; //552  ??  -      reserved
//unsigned char EE_553        EEMEM =    0; //553  ??  -      reserved
//unsigned char EE_554        EEMEM =    0; //554  ??  -      reserved
//unsigned char EE_555        EEMEM =    0; //555  ??  -      reserved
//unsigned char EE_556        EEMEM =    0; //556  ??  -      reserved
//unsigned char EE_557        EEMEM =    0; //557  ??  -      reserved

//unsigned char eemac[4*16*2] EEMEM;
//unsigned char eedec[80*4] EEMEM;


#endif


*/


// 594-623 - Reserved
// 624-640 - Manufacturer Unique
// 641-1024 - Reserved

//---------------------------------------------------------------------------------------------
//
// CV541, bit 6 set the addressing mode:
// 0: Decoder Address Method:
//    Address = (CV513 & 0b00111111) + (CV521 & 0b00000111) << 6
// 1: Output Address Method:
//    Accessory-Output = (CV513+(CV521*256)) - 1
//    
/*
// Descriptions of Configuration Variables for Accessory Decoders
// Taken from NMRA

Configuration Variable 513 Decoder Address (LSB)
    Contains the low-order address bits for Accessory Decoders. The high-order
    address bits are stored in CV521.
    Two types of Accessory Decoder addressing are supported: Decoder-Address and Output-Address.
    An accessory decoder must support one type, and optionally the other type.
    The type of decoder is specified in CV541, bit 6.
    Decoders using either type of addressing will respond to the same Accessory Decoder
    Control Packet when CV513 = 1 and CV521 = 0.
    The factory default value is 1. 
    The type(s) of addressing supported must be clearly documented in the manual and on the packaging.
    (1) Decoder-Address: Contains the six least significant bits of the accessory decoder's address in bits 0-5.
        These bits are transmitted as bits 0-5 in the first byte of the accessory decoder packet. See RP-9.2.1 for more
        information.
    (2) Output-Address: The user places the output address. 
        Contains the address value results from the following formula:
        Output Address modulus 256. (ex. Output Address mod 256, or Output Address % 256).
        The values contained in CV513 and CV521 correspond to the bits in the
        Accessory Decoder packets as follows:
            Accessory-Output = (CV513+(CV521*256)) - 1
        Bits 0 & 1 of the Accessory-Output are transmitted as bits 1 & 2 of byte 2 of
        both Accessory Decoder Control Packets. 
        Bits 2-7 of the Accessory-Output are transmitted as bits 0-5 of byte 1 of 
        both Accessory Decoder Control Packets. The three least-significant bits of CV521
        contain the ones-complement of bits 4-6 of both
        Accessory Decoder Control Packets (See RP-9.2.1 for more information on the 
        Accessory Decoder Control Packets).
    If an accessory decoder supports more than one sequential output the value in CV513 will
    be the first output in the series

Configuration Variable 514 Auxiliary Activation
    Bits 1-8 = Auxiliary activation: = "0" output is not activated by an auxiliary input,
                                       "1" output can be activated by an auxiliary input.

Configuration Variables 515-518 Time On for Functions F1-F4
    Functions F1-F4 can have the time the outputs are active set by configuration variables #515-#518.
    Configuration Variable #515 controls Function F1, and Configuration Variable #518 Controls Function F4.
    Contains a time that the output is on each time the state of the function is activated.
    A value of all "0"s indicates continuous on.

Configuration Variable 519 Manufacturer Version Number
    See CV #7 for the description.

Configuration Variable 520 Manufacturer ID (See Appendix A for a list of Manufacturer IDs)
    See CV #8 for the description.

Configuration Variable 521 Decoder Address (MSB)
    See CV513 for an expl 370 anation of how to determine the contents of CV513 and CV521.
    The bits transmitted are the ones complement of the value in this CV.
    (1) Decoder-Address: Contains the three most significant bits of 
        the accessory decoder’s address in bits 0-2.
        These bits are transmitted as bits 4-6 in the second byte of the accessory decoder packet.
    (2) Output-Address: Contains the address value results from the quotient of the following formula: Output
        Address divided by 256 (Output Address div 256, Output Address / 256).

Configuration Variable 540 Bi-Directional Communication Configuration
    Used to Configure decoder’s Bi-Directional communication characteristics. when CV541-Bit 3 is set
    Bit 0 = Enable/Disable Unsolicited Decoder Initiated Transmission
            0 = Disabled
            1 = Enabled
    Bit 1 = Not Used
    Bits 2-5 = Reserved for future use.
    Bits 6-7 = Flag Bits, Reserved for future use
    *Note If the decoder does not support a feature contained in this table, 
    it shall not allow the corresponding bit to be set improperly
    (i.e. the bit should always contain it’s default value).

Configuration Variable 541 Accessory Decoder Configurations Supported
    Bits 0-2 = Reserved for future use.
    Bit 3 = Bi-Directional Communications: 
            "0" = Bi-Directional Communications disabled,
            "1" = Bi-Directional Communications enabled. See RP-9.3.2 for more information.`
    Bit 4 = Reserved for future use.
    Bit 5 = Decoder Type: 
            ‘0’ = Basic Accessory Decoder; 
            ‘1’ = Extended Accessory Decoder
    Bit 6 = Addressing Method: 
            ‘0’= Decoder Address method;
            ‘1’ = Output Address method
    Bit 7 = Accessory Decoder:
            "0" = Multifunction Decoder (See CV-29 for description of bit Assignments for bits 0-6),
            "1" = Accessory Decoder. 
                  If bit 7 = 1, then the decoder may ignore the two mostsignificant
                  bits of the CV number in Service Mode only. Using this feature CV513 becomes CV1, etc. 
                  Decoders which perform the translation must clearly document the feature in their manual.
                    Note: If the decoder does not support a feature contained in this table,
                    it must not allow the corresponding bit to be set improperly
                    (i.e. the bit should always contain it’s default value).

*/
