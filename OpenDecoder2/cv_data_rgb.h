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
// file:      cv_data_rgb.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2011-09-22 V0.1 kw start
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
   
    // (RGB_ENABLED == TRUE)
   100,          //  REDmax    551  39  -       Justagewert fuer ROT
   100,          //  GREENmax   552  40  -      Justagewert fuer GRUEN
   100,          //  BLUEmax    553  41  -      Justagewert fuer BLAU
                 //  RGB_mode   555  43  -      RGB Mode: reserved
   ( 0 << CVbit_SvMode_MOVMOD   ) |             // Bit 0: Move Mode:          0 = A-B;      1 = permanent
   ( 0 << CVbit_SvMode_OUT_CTRL ) |             // Bit 1: Relais control:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_MAN      ) |             // Bit 2: manual tracers:     0 = disabled, 1 = enabled
   ( 0 << CVbit_SvMode_FEEDBACK ) |             // Bit 3: feedback installed  0 = no        1 = yes (will be used for BiDi)
   ( 1 << CVbit_SvMode_ADJ      ) |             // Bit 4: manual adjustment   0 = disabled  1 = enabled
   ( 0 << CVbit_SvMode_KeepOn   ) |             // Bit 5: pulses when no move 0 = off       1 = pulses even when not moving
   ( 0 << CVbit_SvMode_PowCtrl  ) |             // Bit 6: servo power         0 = always on 1 = turn off power after move
   ( 0 << CVbit_SvMode_Stretch  ),              // Bit 7: extend range        0 = 1ms ..2ms 1 = 0.5ms - 2.5ms
   5,           //  RGB_repeat  556  44  -      RGB Repeat: 0=forever
   0,           //  RGB_profile 558  46  -      RGB Profile (Kurvenauswahl)
   50,          //  RGB_time    558  46  -      RGB Zeitstreckung des Profils
                //  RGB_fade1[24*4];  // 4 Bytes for each point (ein Fade-Profil)

   
