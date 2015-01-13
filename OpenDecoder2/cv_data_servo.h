//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder3
//
// Copyright (c) 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      cv_data_servo.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-08-03 V0.1 kw start
//            2007-08-18 V0.2 kw default for myADDR hight changed to 0x80
//                               -> this means: unprogrammed         
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            This is the default cv definition for the project
//            cv_define_xxxxx.c contains the definitions.
//
//------------------------------------------------------------------------
//
// content:   1. ....
//
//========================================================================
////-----------------------------------------------------------------------------
// data in EEPROM:
// Note: the order of these data corresponds to physical CV-Address
//       CV1 is coded at #00
//       see RP 9.2.2 for more information


//-----------------------------------------------------------------------------
// data in EEPROM:
// Note: the order of these data corresponds to physical CV-Address
//       CV1 is coded at #00
//       see RP 9.2.2 for more information

// Content          Name       CV  -alt  type    comment
   0x01,        //  myAddrL     513   1   M       Decoder Adresse low, 6 bits
   0,           //  auxActiv    514   2  O       Auxiliary activation of outputs: 0=no aux activation
   15,          //  T_on_F1     515   3  O       Time on F1
   15,          //  T_on_F2     516   4  O       Time on F2
   15,          //  T_on_F3     517   5  O       Time on F3
   15,          //  T_on_F4     518   6  O       Time on F4
   0x07,        //  version     519   7  M       Version
   0x0D,        //  VID         520   8  M       Vendor ID (0x0D = DIY Decoder)
                                                //        (0x3E = TAMS)
   0x80,        //  myAddrH     521   9  M       Decoder Adresse high (3 bits)
   0,           //  cv522       522  10  -       reserved
   0,           //  cv523       523  11  -       reserved
   0,           //  cv524       524  12  -       reserved
   0,           //  cv525       525  13  -       reserved
   0,           //  cv526       526  14  -       reserved
   0,           //  cv527       527  15  -       reserved
   0,           //  cv528       528  16  -       reserved
   0,           //  cv529       529  17  -       reserved
   0,           //  cv530       530  18  -       reserved
   0,           //  cv531       531  19  -       reserved
   0,           //  cv532       532  20  -       reserved
   0,           //  cv533       533  21  -       reserved
   0,           //  cv534       534  22  -       reserved
   0,           //  cv535       535  23  -       reserved
   0,           //  cv536       536  24  -       reserved
   0,           //  cv537       537  25  -       reserved
   0,           //  cv538       538  26  -       reserved
   0,           //  cv539       539  27  -       reserved
   0,           //  BiDi        540  28  -       Bi-Directional Communication Config - keep at 0
                //  Config      541  29  -       similar to CV#29; for acc. decoders
                     (1<<7)                   // 1 = we are accessory
                   | (0<<6)                   // 0 = we do 9 bit decoder adressing
                   | (0<<5)                   // 0 = we are basic accessory decoder
                   | 0,                       // 4..0: all reserved
   0,           //  cv542       542  30  -       reserved
   0,           //  cv543       543  31  -       reserved
   0,           //  LastState   544  32  -       saved last port state (was reserved)
                //
                // 545-593 - Manufacturer Unique
                //
#if (RGB_ENABLED)
   33,          //  MODE        545  33  -      global decoder mode
                                               // 00 = turnout decoder with feedback
                                               // 01 = dual servo decoder
                                               // 02 = multiposition servo decoder
                                               // 03 = direct output control
                                               // 04 = reserved
                                               // ...
                                               // 08 = dmx decoder
                                               // 16 = kirmes decoder
                                               // 17 = Neon
                                               // 33 = rgb
#else
   1,           //  MODE        545  33  -      global decoder mode
#endif
   1,           //  FM          546  34  -      global feedback mode
                                               // 00 = no feedback
                                               // 01 = pos. feedback (ack)
                                               // 02 = neg. feedback (emergency stop)
   1,           //  FBM_F1      547  35  -      feedback mode Func 1
                                               // 00 = command acknowledge
                                               // 01 = turnout feedback with intrinsic endswitches
                                               // 02 = feedback with extra detector
                                               // 03 = feedback with extra detector
   1,           //  FBM_F2      548  36  -      feedback mode Func 2
   1,           //  FBM_F3      549  37  -      feedback mode Func 3
   1,           //  FBM_F4      550  38  -      feedback mode Func 4

    // (SERVO_ENABLED == TRUE)
   0,           //  Sv1_minL    551  39  -      Servo 1 Min low
   20,          //  Sv1_min     552  40  -      Servo 1 Min high
   0,           //  Sv1_maxL    553  41  -      Servo 1 Max low
   250,         //  Sv1_max     554  42  -      Servo 1 Max high
                //  Sv1_Mode    555  43  -      Servo 1 Mode: 
   ( 0 << CVbit_SvMode_MOVMOD   ) |             // Bit 0: Move Mode:          0 = A-B;      1 = permanent
   ( 0 << CVbit_SvMode_OUT_CTRL ) |             // Bit 1: Relais control:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_MAN      ) |             // Bit 2: manual tracers:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_FEEDBACK ) |             // Bit 3: feedback installed  0 = no        1 = yes (will be used for BiDi)
   ( 1 << CVbit_SvMode_ADJ      ) |             // Bit 4: manual adjustment   0 = disabled  1 = enabled
   ( 0 << CVbit_SvMode_KeepOn   ) |             // Bit 5: pulses when no move 0 = off       1 = pulses even when not moving
   ( 0 << CVbit_SvMode_PowCtrl  ) |             // Bit 6: servo power         0 = always on 1 = turn off power after move
   ( 0 << CVbit_SvMode_Stretch  ),              // Bit 7: extend range        0 = 1ms ..2ms 1 = 0.5ms - 2.5ms
   5,           //  Sv1_Repeat  556  44  -      Servo 1 Repeat: 0=forever
   0,           //  Sv1_Loc     557  45         Servo 1 Location: 0 = pre Move A; 1=pre move B
   7,           //  Sv1_CurveA  558  46  -      Servo 1 Curve Movement A
   6,           //  Sv1_TimeA   559  47  -      Servo 1 Curve Time stretch A
   8,           //  Sv1_CurveB  560  48  -      Servo 1 Curve Movement B
   6,           //  Sv1_TimeB   561  49  -      Servo 1 Curve Time stretch B
   0,           //  Sv1_res     562  50  -      Servo 1 reserved

   0,           //  Sv2_minL    563  51  -      Servo 2 Min low
   50,          //  Sv2_min     564  52  -      Servo 2 Min high
   0,           //  Sv2_maxL    565  53  -      Servo 2 Max low
   200,         //  Sv2_max     566  54  -      Servo 2 Max high
                //  Sv2_Mode    567  55  -      Servo 2 Mode
   ( 0 << CVbit_SvMode_MOVMOD   ) |             // Bit 0: Move Mode:          0 = A-B;      1 = permanent
   ( 0 << CVbit_SvMode_OUT_CTRL ) |             // Bit 1: Relais control:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_MAN      ) |             // Bit 2: manual tracers:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_FEEDBACK ) |             // Bit 3: feedback installed  0 = no        1 = yes (will be used for BiDi)
   ( 1 << CVbit_SvMode_ADJ      ) |             // Bit 4: manual adjustment   0 = disabled  1 = enabled
   ( 0 << CVbit_SvMode_KeepOn   ) |             // Bit 5: pulses when no move 0 = off       1 = pulses even when not moving
   ( 0 << CVbit_SvMode_PowCtrl  ) |             // Bit 6: servo power         0 = always on 1 = turn off power after move
   ( 0 << CVbit_SvMode_Stretch  ),              // Bit 7: extend range        0 = 1ms ..2ms 1 = 0.5ms - 2.5ms
   0,           //  Sv2_Repeat  568  56  -      Servo 2 Repeat
   0,           //  Sv2_Loc     569  57         Servo 2 Location = 0 = pre Move A; 1=pre move B
   7,           //  Sv2_CurveA  570  58  -      Servo 2 Curve Movement A
   8,           //  Sv2_TimeA   571  59  -      Servo 2 Curve Time stretch A
   8,           //  Sv2_CurveB  572  60  -      Servo 2 Curve Movement B
   8,           //  Sv2_TimeB   573  61  -      Servo 2 Curve Time stretch B
   0,           //  Sv2_res     574  62  -      Servo 2 reserved

   0,           //  cv575       575  63  -      reserved

// Variables for multiposition servo control 

   0b111,       //  Pos_Mode    576  64  -      Position Mode
                // #define BIT_EE_POSMOD_SMTH    0        //                - Smooth operation
                // #define BIT_EE_POSMOD_ADJ     1        //                - Adjustment via DCC
                // #define BIT_EE_POSMOD_MAN     2        //                - Manual Operation
   5,           //  Pos_Time    577  65  -      Position Time stretch

   0,           //  Last_Pos    578  66  -      Last index (read only)
   1,           //  ManIncr     579  67  -        1 = 4096 steps for full range
                                          //                255 = 16 step from min to max

   0,           //  Pos1_L      580  68  -      Servo 1 Position A, low part
   10,          //  Pos1_H      581  69  -      Servo 1 Position A, high part
   0,           //  Pos2_L      582  70  -      Servo 1 Position B, low part
   50,          //  Pos2_H      583  71  -      Servo 1 Position B, high part
   0,           //  Pos3_L      584  72  -      Servo 1 Position C, low part
   100,         //  Pos3_H      585  73  -      Servo 1 Position C, high part
   0,           //  Pos4_L      586  74  -      Servo 1 Position D, low part
   150,         //  Pos4_H      587  75  -      Servo 1 Position D, high part
   0,           //  Pos5_L      588  76  -      Servo 1 Position E, low part
   200,         //  Pos5_H      589  77  -      Servo 1 Position E, high part
   0,           //  Pos6_L      590  78  -      Servo 1 Position F, low part
   210,         //  Pos6_H      591  79  -      Servo 1 Position F, high part
   0,           //  Pos7_L      592  80  -      Servo 1 Position G, low part
   220,         //  Pos7_H      593  81  -      Servo 1 Position G, high part
   0,           //  Pos8_L      594  82  -      Servo 1 Position H, low part
   250,         //  Pos8_H      595  83  -      Servo 1 Position H, high part


   0,           //  cv596       596  84  -      reserved
   0,           //  cv597       597  85  -      reserved
   0,           //  cv598       598  86  -      reserved
   0,           //  cv599       599  87  -      reserved


//  servo_curve1[48] EEMEM;  // these are pairs of time/positions - see servo.c
//  servo_curve2[48] EEMEM;
//  servo_curve3[48] EEMEM;
//  servo_curve4[48] EEMEM;

