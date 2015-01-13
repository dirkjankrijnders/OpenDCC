//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      dcc_decode.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2006-02-14 V0.1 kw start
//            2007-04-27 V0.4 kw changed return codes
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//
//------------------------------------------------------------------------
//
// howto:     Step 1: set up a timerengine, suppling the system with an increment
//                    on "timerval" every 20ms
//
//            Step 2: call init_dcc_decode();
//
//            Step 3: init_dcc_receiver();
//
//            Step 4: check for semaphor C_Received; if so call analyze_message().
//                    This checks the received DCC message
//                    and returns a code
//                    All relevant Data are stored to globals (Received...)
//

/*
#define MAX_DCC_SIZE  6
struct message
  {
    unsigned char size;               // 3 .. 6, including XOR
    unsigned char dcc[MAX_DCC_SIZE];  // the dcc content
  };
*/

extern unsigned int  ReceivedAddr;          // last received address - gets filled by dcc_decode
extern unsigned int  ReceivedCommand;       // subaddress (starting from the first address)
                                            // or aspect
extern unsigned char  ReceivedActivate;      // coil

unsigned char analyze_message(t_message *new);        // this returns a code on the result:
                                            // 0: if void,
                                            // 1: if accessory and command type equal our mode
                                            // 2: if accessory and address equal myAddr (or broadcast)
                                            // 3: if accessory and address > myAddr (Received Command is extended)

void init_dcc_decode(void);
             





// --- Modul Internals

/* 
extern enum 
  {
    CV_NOP          = 0b00,         // CC=00 Reserved for future use
    CV_VERIFY       = 0b01,         // CC=01 Verify byte
    CV_WRITE        = 0b11,         // CC=11 Write byte
    CV_BITOPERATION = 0b10,         // CC=10 Bit manipulation
  } ReceivedOperation;

// extern unsigned char ReceivedOperation; 

extern unsigned int  ReceivedCV;           // Configuration Variable to change
extern unsigned char ReceivedData;         // CV Value
*/


