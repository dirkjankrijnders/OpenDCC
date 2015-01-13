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
//            2007-09-18 V0.2 kw CV554 bis CV559 ergänzt, damit TP File
//                               und OpenDecoder3 konsistent (war vergessen) 
//            2008-09-03 V0.3 kw CVbit_SvMode_PowCtrl dazu
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            This is the cv-structure definition for the project
//            cv_data_xxxxx.h will contain the actual data.
//
//------------------------------------------------------------------------
//
// content:   1. ....
//
//========================================================================
#ifndef _CV_DEFINE_SERVO_H_
#define _CV_DEFINE_SERVO_H_
////-----------------------------------------------------------------------------
// data in EEPROM:
// Note: the order of these data corresponds to physical CV-Address
//       CV1 is coded at #00
//       see RP 9.2.2 for more information

// Bit defines for SERVO_MODE
#define CVbit_SvMode_MOVMOD    0        //  move mode (1=permanent)
#define CVbit_SvMode_OUT_CTRL  1        //  relais control
#define CVbit_SvMode_MAN       2        //  manual tracers allowed
#define CVbit_SvMode_FEEDBACK  3        //  feedback installed
#define CVbit_SvMode_ADJ       4        //  manual adjustment
#define CVbit_SvMode_KeepOn    5        //  servo pulses even when not moving
#define CVbit_SvMode_PowCtrl   6        //  turn off servo power after movement (only V2.5)
#define CVbit_SvMode_Stretch   7        //  expand range from [1ms..2ms] to 0,5..2.5ms


#define CVbit_PosMode_SMTH    0        //                - Smooth operation
#define CVbit_PosMode_ADJ     1        //                - Adjustment via DCC
#define CVbit_PosMode_MAN     2        //                - Manual Operation
    

// Bit defines for DMX_MODE
#define CVbit_DMX_MODE_CALL         7
#define CVbit_DMX_MODE_INIT_STATE   6
#define CVbit_DMX_MODE_WATCHDOG     5
#define CVbit_DMX_MODE_WATCH_REL    4


typedef struct
  {
    //            Name              default    CV  -alt  type    comment
    unsigned char myAddrL;     //513   1  M       Decoder Adresse low, 6 bits
    unsigned char auxActiv;    //514   2  O       Auxiliary activation of outputs: 0=no aux activation
    unsigned char T_on_F1;     //515   3  O       Time on F1
    unsigned char T_on_F2;     //516   4  O       Time on F2
    unsigned char T_on_F3;     //517   5  O       Time on F3
    unsigned char T_on_F4;     //518   6  O       Time on F4
    unsigned char version;     //519   7  M       Version
    unsigned char VID    ;     //520   8  M       Vendor ID (0x0D = DIY Decoder, 0x12 = JMRI, 0x3E = TAMS)
    unsigned char myAddrH;     //521   9  M       Decoder Adresse high (3 bits)
    unsigned char cv522  ;     //522  10  -       reserved
    unsigned char cv523    ;   //523  11  -       reserved
    unsigned char cv524    ;   //524  12  -       reserved
    unsigned char cv525    ;   //525  13  -       reserved
    unsigned char cv526    ;   //526  14  -       reserved
    unsigned char cv527    ;   //527  15  -       reserved
    unsigned char cv528    ;   //528  16  -       reserved
    unsigned char cv529    ;   //529  17  -       reserved
    unsigned char cv530    ;   //530  18  -       reserved
    unsigned char cv531    ;   //531  19  -       reserved
    unsigned char cv532    ;   //532  20  -       reserved
    unsigned char cv533    ;   //533  21  -       reserved
    unsigned char cv534    ;   //534  22  -       reserved
    unsigned char cv535    ;   //535  23  -       reserved
    unsigned char cv536    ;   //536  24  -       reserved
    unsigned char cv537    ;   //537  25  -       reserved
    unsigned char cv538    ;   //538  26  -       reserved
    unsigned char cv539    ;   //539  27  -       reserved
    unsigned char BiDi   ;     //540  28  -       Bi-Directional Communication Config - keep at 0
    unsigned char Config ;     //541  29  -       similar to CV#29; for acc. decoders
    unsigned char cv542    ;   //542  30  -       reserved
    unsigned char cv543    ;   //543  31  -       reserved
    unsigned char LastState;   //544  32  -       saved last port state (was reserved)
    //
    // 545-593 - Manufacturer Unique
    //
    unsigned char MODE;        //545  33  -      global decoder mode
                                                                // 00 = turnout decoder with feedback
                                                                // 01 = dual servo decoder
                                                                // 02 = multiposition servo decoder
                                                                // 03 = relais direct
                                                                // 04 = reserved
                                                                // ...
                                                                // 08 = dmx decoder
                                                                // 10 = signal decoder (tbd.)
                                                                // 16 = kirmes
                                                                // 17 = neon action
                                                                // 32 = sodium
                                                                // 33 = direct RGB-control
                                                                // 34 = RGB-profiles + servo decoder

    unsigned char FM ;         //546  34  -      global feedback mode
                                                                // 00 = no feedback
                                                                // 01 = pos. feedback (ack)
                                                                // 02 = neg. feedback (emergency stop)
                                                                // for NEON: Opmode
    unsigned char FBM_F1;      //547  35  -      feedback mode Func 1
                                                                // 00 = command acknowledge
                                                                // 01 = turnout feedback with intrinsic endswitches
                                                                // 02 = feedback with extra detector
                                                                // 03 = feedback with extra detector
    unsigned char FBM_F2;       //548  36  -      feedback mode Func 2
    unsigned char FBM_F3;       //549  37  -      feedback mode Func 3
    unsigned char FBM_F4;       //550  38  -      feedback mode Func 4

    #if (SERVO_ENABLED == TRUE)

    unsigned char Sv1_minL   ; //551  39  -      Servo 1 Min low
    unsigned char Sv1_min    ; //552  40  -      Servo 1 Min high
    unsigned char Sv1_maxL   ; //553  41  -      Servo 1 Max low
    unsigned char Sv1_max    ; //554  42  -      Servo 1 Max high
    unsigned char Sv1_Mode   ; //555  43  -      Servo 1 Mode: see Mode bits above
    unsigned char Sv1_Repeat ; //556  44  -      Servo 1 Repeat: 0=forever
    unsigned char Sv1_Loc    ; //557  45         Servo 1 Location: 0 = pre Move A; 1=pre move B
    unsigned char Sv1_CurveA ; //558  46  -      Servo 1 Curve Movement A
    unsigned char Sv1_TimeA  ; //559  47  -      Servo 1 Curve Time stretch A
    unsigned char Sv1_CurveB ; //560  48  -      Servo 1 Curve Movement B
    unsigned char Sv1_TimeB  ; //561  49  -      Servo 1 Curve Time stretch B
    unsigned char Sv1_res    ; //562  50  -      Servo 1 reserved

    unsigned char Sv2_minL   ; //563  51  -      Servo 2 Min low
    unsigned char Sv2_min    ; //564  52  -      Servo 2 Min high
    unsigned char Sv2_maxL   ; //565  53  -      Servo 2 Max low
    unsigned char Sv2_max    ; //566  54  -      Servo 2 Max high
    unsigned char Sv2_Mode   ; //567  55  -      Servo 2 Mode = 0=A-B 1=permanent
    unsigned char Sv2_Repeat ; //568  56  -      Servo 2 Repeat
    unsigned char Sv2_Loc    ; //569  57         Servo 2 Location = 0 = pre Move A; 1=pre move B
    unsigned char Sv2_CurveA ; //570  58  -      Servo 2 Curve Movement A
    unsigned char Sv2_TimeA  ; //571  59  -      Servo 2 Curve Time stretch A
    unsigned char Sv2_CurveB ; //572  60  -      Servo 2 Curve Movement B
    unsigned char Sv2_TimeB  ; //573  61  -      Servo 2 Curve Time stretch B
    unsigned char Sv2_res    ; //574  62  -      Servo 2 reserved

    unsigned char cv575      ; //575  63  -      reserved

    // Variables for multiposition servo control 

    unsigned char Pos_Mode   ; //576  64  -      Position Mode
    // #define BIT_POSMOD_SMTH    0        //                - Smooth operation
    // #define BIT_POSMOD_ADJ     1        //                - Adjustment via DCC
    // #define BIT_POSMOD_MAN     2        //                - Manual Operation
    unsigned char Pos_Time   ; //577  65  -      Position Time stretch

    unsigned char Last_Pos   ; //578  ??  -      Last index (read only)
    unsigned char ManIncr    ; //579  ??  -        1 = 4096 steps for full range
                                              //                255 = 16 step from min to max

    unsigned char Pos1_L     ; //580  ??  -      Servo 1 Position A, low part
    unsigned char Pos1_H     ; //581  ??  -      Servo 1 Position A, high part
    unsigned char Pos2_L     ; //582  ??  -      Servo 1 Position B, low part
    unsigned char Pos2_H     ; //583  ??  -      Servo 1 Position B, high part
    unsigned char Pos3_L     ; //584  ??  -      Servo 1 Position C, low part
    unsigned char Pos3_H     ; //585  ??  -      Servo 1 Position C, high part
    unsigned char Pos4_L     ; //586  ??  -      Servo 1 Position D, low part
    unsigned char Pos4_H     ; //587  ??  -      Servo 1 Position D, high part
    unsigned char Pos5_L     ; //588  ??  -      Servo 1 Position E, low part
    unsigned char Pos5_H     ; //589  ??  -      Servo 1 Position E, high part
    unsigned char Pos6_L     ; //590  ??  -      Servo 1 Position F, low part
    unsigned char Pos6_H     ; //591  ??  -      Servo 1 Position F, high part
    unsigned char Pos7_L     ; //592  ??  -      Servo 1 Position G, low part
    unsigned char Pos7_H     ; //593  ??  -      Servo 1 Position G, high part
    unsigned char Pos8_L     ; //594  ??  -      Servo 1 Position H, low part
    unsigned char Pos8_H     ; //595  ??  -      Servo 1 Position H, high part


    unsigned char cv596      ; //596  ??  -      reserved
    unsigned char cv597      ; //597  ??  -      reserved
    unsigned char cv598      ; //598  ??  -      reserved
    unsigned char cv599      ; //599  ??  -      reserved

    #endif


    #if (DMX_ENABLED == TRUE)
    unsigned char DMX_MODE;     //551  39  -      DMX Mode
                                                            // Bit 7: 1 = single call to cmx_operation allowed
                                                            //        0 = only base address calls
                                                            // Bit 6: 1 = initial state = all on
                                                            //        0 = initial state = all off
                                                            // Bit 5  1 = Watchdog event turns on lights
                                                            //        0 = no reaction

    unsigned char WD_Time;      //552  40 -        Watchdog Intervall Time [unit 100ms]
    unsigned char Alarm_Time;   //553  41 -        Activation Time for Alarm [unit 100ms]

    unsigned char OnReact_7_0;  //554  42 -        Bitfield, a 1 marks this dmx channel to receive 255 when all on is called 
    unsigned char OnReact_15_8; //555; 43 -        Bitfield, a 1 marks this dmx channel to receive 255 when all on is called 
    unsigned char OnReact_23_16;//556; 44 -        Bitfield, a 1 marks this dmx channel to receive 255 when all on is called 
    unsigned char OnReact_31_24;//557; 45 -        Bitfield, a 1 marks this dmx channel to receive 255 when all on is called 
    unsigned char cv558;        //558; 46 -        reserved
    unsigned char PresetLoad;   //559; 47 -        a write determines the new preset to load

//    t_dmxctrl dmxctrl[ESIZE_DMXCTRL];             // 80 virtual decoder
                                                  // each 4 bytes long
                                                  // 560-563, 564-567 ... 886-889 

//    t_dmxmacro dmxmacro[ESIZE_DMXMACRO];          // 4 Macros, each 16 pairs
                                                  // macro 1: cv880-cv911
                                                  // macro 2: cv912-cv943
                                                  // macro 3: cv944-cv975
                                                  // macro 4: cv976-cv1007
    #endif
    #if (REVERSER_ENABLED == TRUE)
    unsigned char K1_Trg1_normal; //551  39
    unsigned char K1_Trg2_normal; //552  40
    unsigned char K1_Trg1_invers; //553  41
    unsigned char K1_Trg2_invers; //554  42
    unsigned char K2_Trg1_normal; //551  43
    unsigned char K2_Trg2_normal; //552  44
    unsigned char K2_Trg1_invers; //553  45
    unsigned char K2_Trg2_invers; //554  46
    #endif

    #if (RGB_ENABLED == TRUE)
    unsigned char REDmax;           //  551  39  -      1 ... 255
    unsigned char GREENmax;         //  551  39  -      1 ... 255
    unsigned char BLUEmax;          //  551  39  -      1 ... 255
    unsigned char RGB_mode;         //  noch reserved - angedacht:
                                    //  a) direkte Ansteuerung mit 3 Weichen, time macht dann die Dimmgeschwindigkeit
                                    //  b) Profilwahl
    unsigned char RGB_repeat;       //   556  44  -     RGB 1 Repeat: 0=forever
    unsigned char RGB_profile;
    unsigned char RGB_time;
    unsigned char RGB_fade1[24*4];  // 4 Bytes for each point (ein Fade-Profil)
    #endif


 } t_cv_record;


#endif  // _CV_DEFINE_SERVO_H_
