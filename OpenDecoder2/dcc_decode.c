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
// file:      dcc_decode.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-14 V0.1 kw copied from opendecoder.c
//                               split up the code in modules
//                               added routines for CV-Programming
//            2007-03-30 V0.2 kw corrected some items with Decoding
//            2007-04-16 V0.3 kw added debugswitch for service mode and
//                               feedback testing
//                               service mode is only executed after second
//                               received command
//            2007-04-27 V0.4 kw changed return codes
//            2007-08-04 V0.5 kw Bugfix in cv_is_blocked
//            2007-08-18 V0.6 kw masking of myADDRHigh with 0x7F
//                               (hidden bit: unprogrammed)
//            2007-11-22 V0.7 kw Decoder Reset added         
//
// tests:     2007-04-14 decode okay
//                       CV read/write direct mode okay, cv bitmode
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: all protocol issues with DCC
//                   we support CV-byte operations and PoM
//                   we do CV remapping - CV1 == CV513 and so on
//
// content:   A DCC-Decoder for ATmega8515/ATmega162 and other AVR
//
//             1. CV-Handling (including preset)
//             2. DCC Parser
//
// required:  a running timerengine (for timeouts)
//            this engine is currently implemented in port_engine
//            (TICK_PERIOD, timerval)
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

#include "config.h"              // general definitions the decoder, cv's
#include "myeeprom.h"            // wrapper for eeprom
#include "hardware.h"            // port definitions
#include "dcc_receiver.h"        // receiver for dcc
#include "dcc_decode.h"          // decoder for dcc


#define SERVICE_MODE_TIMEOUT   40000L    // 40ms - at least 20ms
#if ((SERVICE_MODE_TIMEOUT / TICK_PERIOD) == 0)
  #warning: TICK_PERIOD too large
#endif
#if ((SERVICE_MODE_TIMEOUT / TICK_PERIOD) > 127)
  #error: TICK_PERIOD too small
#endif


//------------------------------------------------------------------------------

#define DEBUG_FEEDBACK      FALSE       // TRUE: a call to addr+4 is mapped to "Coil Off"
                                        //       at addr. 
                                        //       set this option to test the feedback decoder
                                        //       or to use it with !grrml! intellibox.
                                        // FALSE: normal operation

#define DEBUG_PORTB7_IS_SM  FALSE       // TRUE: Port B, Bit 7 mirrors servicemode enabled
                                        // FALSE: normal operation
#if (DEBUG_FEEDBACK == TRUE)
  #warning DEBUG_FEEDBACK active
#endif

#if (DEBUG_PORTB7_IS_SM == TRUE)
  #warning DEBUG__PORTB7_IS_SM active
#endif


#ifndef CV_REMAPPING
  #error CV_REMAPPING not defined - must be set TRUE to map correctly to eeprom values
#endif

//-----------------------------------------------------------------------------------
//
// Global Data
// after decoding, the last data is stored the following global variables
// (Note: this is not really clean - but we are on micro and have to save memory ...)
//

unsigned int  ReceivedAddr;         // last received address - gets filled by dcc_decode
unsigned int  ReceivedCommand;      // format: 0000dddd dddddccc
                                    // where: ccc is the coil address like in DCC.
                                    // ddddddddd is the difference between ReceivedAddr
                                    // and MyAddr (only in case ReceivedAddr > MyAddr) 
unsigned char ReceivedActivate;     // 0: a turn OFF was received
                                    // !0: a turn ON was received (typ. 0b00001000)



//-----------------------------------------------------------------------------------
//
// -- Static, but local data

enum                                // same coding as NMRA
  {
    CV_NOP          = 0b00,         // CC=00 Reserved for future use
    CV_VERIFY       = 0b01,         // CC=01 Verify byte
    CV_WRITE        = 0b11,         // CC=11 Write byte
    CV_BITOPERATION = 0b10,         // CC=10 Bit manipulation
  } ReceivedOperation;
  

unsigned int  ReceivedCV;           // Configuration Variable to change
                                    // Range [0... ]
unsigned char ReceivedData;         // CV Value


unsigned char service_mode_state;   // bit field
#define SM_ENABLED   0              // Bit 0: 0: normal operation
                                    //        1: service mode
#define SM_RECEIVED  1              // Bit 1: 0: initial state
                                    //        1: there is already a received SM
                          
signed char last_sm_mode_received;  // timer variable to create a update grid;


//==============================================================================
//
// Host Interface (Protocol Layer)
//
//==============================================================================


//------------------------------------------------------------------------
// 
// restore all eeprom content to default and reboot

void ResetDecoder(unsigned char state)
  {
    unsigned int i;
    unsigned char default_value;
    unsigned char *eeptr;    
    const unsigned char *pgmptr;    
    unsigned char blink = 16;

    LED_ON;
    
    eeptr = (unsigned char *) &CV;
    pgmptr = (const unsigned char *) &CV_PRESET;

    for (i=0; i < sizeof(CV); i++)
      {
        default_value = pgm_read_byte(pgmptr);
        if (my_eeprom_read_byte(eeptr) != default_value)
          {
            my_eeprom_write_byte(eeptr, default_value);
            blink--;
            if (blink == 8) LED_OFF;
            if (blink == 0)
              {
                LED_ON;
                blink = 16;
              }
          }
        eeptr++;
        pgmptr++;
      }
    eeprom_busy_wait();
    LED_OFF;
  }


unsigned char cv_is_blocked(unsigned int cv)
// some CV are not allowed to be written
// block access to: CV519, CV520 (=7,8) (Version and VID)
//                  CV540, CV541 (=28,29) (config)
  {
    if (cv == (7-1)) return(TRUE);
    if (cv == (8-1)) return(TRUE);       // cv8 is coded as 7
    if (cv == (28-1)) return(TRUE);
    if (cv == (29-1)) return(TRUE);
    return(FALSE);
  }

// used static: 
//   ReceivedOperation
//   ReceivedCV
//   ReceivedData 
void cv_operation(void)
  {
    unsigned char bitmask;

    #if (CV_REMAPPING == TRUE)  // must be true - we don't have more eeprom
        ReceivedCV &= 0x1FF;
    #else
        #warning: on AVR with 512 bytes EEPROM CV Address must be remapped - address error will occur
    #endif

    switch(ReceivedOperation)
      {
        case CV_NOP:
            break;
        case CV_VERIFY:
            if (my_eeprom_read_byte(&CV.myAddrL + ReceivedCV) == ReceivedData)
              {
                activate_ACK(6);
              }
            break;
        case CV_WRITE:
            if (ReceivedCV == (8-1))    // cv8 is coded as 7
              {
                activate_ACK(6);
                ResetDecoder(ReceivedData);
                _restart();                         // really hard exit
              }
            if (cv_is_blocked(ReceivedCV)) return;
            my_eeprom_write_byte(&CV.myAddrL + ReceivedCV, ReceivedData);
            eeprom_busy_wait();
            activate_ACK(6);
            break;
        case CV_BITOPERATION:
            // Data is interpreted as 111KDBBB
            // K = 0 verify, K = 1 write
            // D = value
            // BBB = Bitpos.
            bitmask = 1 << (ReceivedData & 0b00000111);

            if (ReceivedData & 0b00010000)
              { // write bit
                unsigned char oldbyte;

                if (cv_is_blocked(ReceivedCV)) return;

                oldbyte = my_eeprom_read_byte(&CV.myAddrL + ReceivedCV);
                if (ReceivedData & 0b00001000) oldbyte |= bitmask;
                else                           oldbyte &= ~bitmask;
                
                my_eeprom_write_byte(&CV.myAddrL + ReceivedCV, oldbyte);
                eeprom_busy_wait();
                activate_ACK(6);
              }
            else
              { // verify bit
                if (ReceivedData & 0b00001000)
                  {
                    if (my_eeprom_read_byte(&CV.myAddrL + ReceivedCV) & bitmask) 
                        activate_ACK(6);
                  }
                else
                  {
                    if ((my_eeprom_read_byte(&CV.myAddrL + ReceivedCV) & bitmask) == 0)
                        activate_ACK(6);
                  }
              }
            break;
      } 
  }


//
//---------------------------------------------------------------------------------------
// analyze_message(struct message *new_dcc) checks the received DCC message
// parameters:
//      pointer to struct of message, containing size and dcc data (supplied by the dcc_receiver)
//
// returns:
//       0: if void,
//       1: if accessory command and command type equal our mode
//       2: if accessory command and address equal myAddr (or broadcast)
//       3: if accessory command and address > myAddr (Received Command is extended)
//
// side effects: 
//       a) accesses to CV are handled here.
//       b) Globals are loaded:
//              ReceivedAddress
//              ReceivedCommand
//              ReceivedActivate
//       c) Local Statics are loaded:
//              ReceivedOperation
//              ReceivedCV
//              ReceivedData
//
unsigned char analyze_message(t_message *new_dcc)
  {
    unsigned char i;
    unsigned char myxor = 0;
    unsigned int MyAddr;
    unsigned char MyConfig;

  
    for (i=0; i<new_dcc->size; i++)
      {
        myxor = myxor ^ new_dcc->dcc[i];
      }
    if (myxor)
      {
        // checksum error, ignore
        return(0);
      }

    if (service_mode_state & (1 << SM_ENABLED))
      {                                                 //// we are in Service Mode!
        if ((char)(timerval - last_sm_mode_received) >= (SERVICE_MODE_TIMEOUT / TICK_PERIOD)) 
          {
            service_mode_state = 0;                    // timeout reached, leave service mode
            #if (DEBUG_PORTB7_IS_SM == TRUE)
                PORTB &= ~(1<<7);
            #endif
          }

        if (new_dcc->dcc[0] == 0)
          {                 
            if (new_dcc->dcc[1] == 0)
              { // reset message - enter service mode
                service_mode_state = (1 << SM_ENABLED);
                #if (DEBUG_PORTB7_IS_SM == TRUE)
                    PORTB |= (1<<7);
                #endif
                last_sm_mode_received = timerval;
                return(0);
              }
          }
        else if ((new_dcc->dcc[0] >= 112) && (new_dcc->dcc[0] <= 127))
          {
            if (new_dcc->size == 4) // direct mode
              {
                service_mode_state |= (1 << SM_ENABLED);
                last_sm_mode_received = timerval;
            
                // direct mode
                // {preamble} 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
                // CC = 11: write
                // CC = 01: verify
                // CC = 10: bit op
                // {preamble} 0 0111CCAA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1
                //  K = (1=write, 0=verify) D = Bitvalue, BBB = bitpos
            
                if (service_mode_state & (1 << SM_RECEIVED))
                  {  // this is the second message
                    unsigned int tempi;
                    unsigned char tempc;
                    tempc = (new_dcc->dcc[0] & 0b00001100);// CC bits
                    tempc = tempc >> 2; // CC bits

                    tempi = ((new_dcc->dcc[0] & 0b00000011) << 8)
                               | new_dcc->dcc[1];

                    if ( (tempc == ReceivedOperation) &&
                         (tempi == ReceivedCV) &&
                         (new_dcc->dcc[2] == ReceivedData)  )
                      {
                        cv_operation();
                      }
           
                    service_mode_state &= ~(1 << SM_RECEIVED);
                  }
                else
                  {
                    service_mode_state |= (1 << SM_RECEIVED);   // we have a sm message
                    
                    ReceivedOperation = (new_dcc->dcc[0] & 0b00001100);  // CC bits
                    ReceivedOperation = ReceivedOperation >> 2;     // CC bits
                    ReceivedCV = ((new_dcc->dcc[0] & 0b00000011) << 8)
                                | new_dcc->dcc[1];
                    ReceivedData = new_dcc->dcc[2];
                  }
              }
            if (new_dcc->size == 3) // paged/register mode
              {
                service_mode_state |= (1 << SM_ENABLED);
                #if (DEBUG_PORTB7_IS_SM == TRUE)
                    PORTB |= (1<<7);
                #endif
                last_sm_mode_received = timerval;
            
                // paged/register mode
                // {preamble} 0 0111CRRR 0 DDDDDDDD 0 EEEEEEEE 1
                // C = 1: write
                // C = 0: verify
                // RRR = Register
                
                // !!!! Note: this is not supported !!!!
              }
            return(0);
          }
        else if (new_dcc->dcc[0] == 255)
          {
            last_sm_mode_received = timerval;
            return(0);
          }
      }

    service_mode_state = 0;       // anyway
    #if (DEBUG_PORTB7_IS_SM == TRUE)
        PORTB &= ~(1<<7);
    #endif
    if (new_dcc->dcc[0] == 0)
      {                                                 //// Broadcast Address
        if (new_dcc->dcc[1] == 0)
          {
            service_mode_state |= (1 << SM_ENABLED);
                 
            #if (DEBUG_PORTB7_IS_SM == TRUE)
                PORTB |= (1<<7);
            #endif
            last_sm_mode_received = timerval;
          }
      }
    else if (new_dcc->dcc[0] <= 127)
      {                                                 //// loco decoders (7 bit addr)
        // {preamble} 0 0AAAAAAA 0 01DCSSSS 0 EEEEEEEE 1 
        // C may be lsb of speed or headlight
        // D = direction: 1 = forward

        ReceivedAddr = (new_dcc->dcc[0] & 0b01111111);

        // see RP921 for more information

        switch (new_dcc->dcc[1] & 0b11100000)
          {
            case 0b00000000:            // 000 Decoder and Consist Control Instruction
            case 0b00100000:            // 001 Advanced Operation Instructions
            case 0b01000000:            // 010 Speed and Direction Instruction for reverse operation
            case 0b01100000:            // 011 Speed and Direction Instruction for forward operation
            case 0b10000000:            // 100 Function Group One Instruction
            case 0b10100000:            // 101 Function Group Two Instruction
            case 0b11000000:            // 110 Future Expansion
            case 0b11100000:            // 111 Configuration Variable Access Instruction
                break;
          }                                       
      }
    else if (new_dcc->dcc[0] <= 191)
      {                                                         //// Accessory 
        MyConfig = my_eeprom_read_byte(&CV.Config) & (1<<6);       // (0=basic, 1=extended)

        if ((new_dcc->dcc[1] >= 0b10000000) && (MyConfig == 0))
          {                                                     //// Basic Accessory (9 bit addr)
            
            // take bits 5 4 3 2 1 0 from new_dcc->dcc[0]
            // take Bits 6 5 4 from new_dcc->dcc[1] and invert

            ReceivedAddr = (new_dcc->dcc[0] & 0b00111111)
                        | ((~new_dcc->dcc[1] & 0b01110000) << 2);

            MyAddr = ((my_eeprom_read_byte(&CV.myAddrH) & 0x7F) << 6) |
                    (my_eeprom_read_byte(&CV.myAddrL));

            ReceivedActivate = new_dcc->dcc[1] & 0b00001000;
            ReceivedCommand = new_dcc->dcc[1] & 0b00000111;
            if (ReceivedAddr > MyAddr)
              {
                ReceivedCommand += (ReceivedAddr - MyAddr) * 8;   // for more than one addr
              }
            
            if (new_dcc->size == 3) // it's a command
              {
                // Format:
                // {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
                //                AAAAAA    aaa                   = Decoder Address
            
                if (ReceivedAddr == 0x01FF) 
                  {
                    ReceivedCommand &= 0x0007;
                    return(2);  // basic paket - broadcast
                  } 

                if (ReceivedAddr == MyAddr) return(2);

                #if (DEBUG_FEEDBACK == TRUE)

                    if (ReceivedAddr == (MyAddr+1) )
                      {
                        ReceivedCommand &= 0x0007;
                        ReceivedActivate = 0;   // remap this to the off command! (dirty!)
                        return(2);
                      }
                #endif

                if (ReceivedAddr > MyAddr) return(3);
                return(1);  
              }
            else if (new_dcc->size == 6) // cv-access on the main
              {
                // Format:
                // {preamble} 10AAAAAA 0 1AAACDDD 0 (1110CCAA 0 AAAAAAAA 0 DDDDDDDD) 0 EEEEEEEE 1

                ReceivedCommand = new_dcc->dcc[1] & 0b00000111;   // CDDD: 1000-1111 individual output
                                                             // 0000: all outputs
                                                             // we ignore it.

                ReceivedOperation = (new_dcc->dcc[2] & 0b00001100);// CC bits
                ReceivedOperation = ReceivedOperation >> 2;   // CC bits
                
                ReceivedCV = ((new_dcc->dcc[2] & 0b00000011) << 8)
                           | new_dcc->dcc[3];

                ReceivedData = new_dcc->dcc[4];

                if (ReceivedAddr == MyAddr) 
                  {
                    cv_operation();             // note: this is not fully correct,
                                                // we react on the first PoM-command, not on the second
                  }
              }
          }
        else if ((new_dcc->dcc[1] < 0b10000000) && (MyConfig != 0))
          {                                         //// Extended Acc. (11 bit addr)

            ReceivedAddr = (( new_dcc->dcc[1] & 0b00000110) >> 1)  // >>1
                        | (( new_dcc->dcc[0] & 0b00111111) << 2) 
                        | ((~new_dcc->dcc[1] & 0b01110000) >> 4);

            MyAddr = (((my_eeprom_read_byte(&CV.myAddrH) & 0x7F) << 8) |
                     (my_eeprom_read_byte(&CV.myAddrL))) - 1;

            if (new_dcc->size == 4) // it's a command
              { // Format:
                // {preamble} 0 10AAAAAA 0 0AAA0AA1 0 000XXXXX 0 EEEEEEEE 1
                // {preamble} 0 10111111 0 00000111 0 000XXXXX 0 EEEEEEEE 1
                // output mode

                ReceivedCommand = new_dcc->dcc[2] & 0b00011111;  // aspect

                // take bits 2 1 from new_dcc->dcc[1]
                // take bits 5 4 3 2 1 0 from new_dcc->dcc[0]
                // take Bits 6 5 4 from new_dcc->dcc[1] and invert

                if (ReceivedAddr == 0x07FF)
                  {
                    // broadcast
                    return(2);
                  }
                if (ReceivedAddr == MyAddr) return(2);
                else return(1);
              }
            else if (new_dcc->size == 6) // cv-access (on the main)
              {
                // Format:
                // {preamble} 0 10AAAAAA 0 0AAA0AA1 0 (1110CCAA 0 AAAAAAAA 0 DDDDDDDD) 0 EEEEEEEE 1

                ReceivedOperation = (new_dcc->dcc[2] & 0b00001100);// CC bits
                ReceivedOperation = ReceivedOperation >> 2; // CC bits
                
                ReceivedCV = ((new_dcc->dcc[2] & 0b00000011) << 8)
                           | new_dcc->dcc[3];

                ReceivedData = new_dcc->dcc[4];

                if (ReceivedAddr == MyAddr) 
                  {
                    cv_operation();             // note: this is not fully correct,
                                                // we react on the first PoM-command, not on the second
                  }
              }
          }
      }
    else if (new_dcc->dcc[0] <= 231)
      {
                                                        //// loco decoders (14 bit addr)
        ReceivedAddr = ((new_dcc->dcc[0] & 0b00111111) << 8)
                    |  (new_dcc->dcc[1]);
      }
    else if (new_dcc->dcc[0] <= 254)
      {                                                 //// Reserved in DCC for Future Use
      }
    else
      {                                                 //// Idle Packet
      }


    return(0);
  }


// must be called once at power up.
void init_dcc_decode(void)
  {
    service_mode_state = 0;         // all bits off
    #if (DEBUG_PORTB7_IS_SM == TRUE)
      PORTB &= ~(1<<7);
    #endif
  }


