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
// history:   2007-02-14 V0.01 kw copied from opendecoder.c
//                                split up the code in modules
//                                added routines for CV-Programming
//            2007-02-25 V0.02 kw corrected some items with Decoding
//            2007-04-10 V0.03 kw moved Timerinit to port_engine.c
//            2007-04-27 V0.04 kw all-in-one software
//            2007-05-31 V0.05 kw added Hardware 3, added Jumperquery
//                                for programming
//            2007-08-05 V0.06 kw change to CV struct 
//            2007-08-18 V0.07 kw added 5 flashes  if unprogrammed
//            2007-12-11          some minor flaws in simulation corrected
//            2008-10-01 V0.08 kw added init for OpenDecoder2.5
//            2008-12-16 V0.09 kw bugfix: do not call init_segment, when in Servo-Mode
//            2008-12-24 V0.10 kw added servo_powerup_delay to diverse power ramping
//            2009-04-14 V0.11 kw service mode alive when in servo delay
//            2009-05-03       kw added switch for servo puls init mode
//                                (CV.Sv1_Mode, Bit 6: CVbit_SvMode_PowCtrl)
//            2010-09-14 V0.12 kw added reverser (relay control for reverser)
//            2011-10-10 V0.14 kw added NEON_MODE
//			  2011-11-29 V0.13 kw added HW28 frog polarisation 2x2 relais on PortA
//                                  and key inputs;
//            2011-12-13          added manual control for Servos
//            2011-12-25 V0.14 kw added RGB, modes 33 and 34
//            2012-12-26 V0.15 kw added direct mode 3
//
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: init, mainloop, globals
//
//
// ====== >>>> config.h  is the central definition file for the project
//
//
// content:   A DCC-Decoder for ATmega8515 and other AVR
//
//             1. Defines and variable definitions
//             2. Init
//             3. DCC Parser
//             5. MAIN: analyze command, call the action, do programming
//             6. Test and Simulation
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
#include "hardware.h"            // port definitions for target
#include "dcc_receiver.h"        // receiver for dcc
#include "dcc_decode.h"          // decode dcc and handle CV access
#include "port_engine.h"         // handling of output and feedback, LED control
#include "reverser_engine.h"     // handling of relay/reverser
#include "dmxout.h"              // dmx
#include "servo.h"               // servo
#include "keyboard.h"
#include "rgb.h"                 // RGB-LED

#include "main.h"

#define SIMULATION  0               // 0: real application
                                    // 0: test receive (with stimuli)
                                    // 2: test receive and decode routine - please enable SIMULATION
                                    //                           also in dcc_receiver
                                    // 3: test timing engine
                                    // 4: test action

#if (SIMULATION != 0)
   #warning SIMULATION is on! - do not use on real hardware!
#endif
//-----------------------------------------------------------------------------------
//
// Security Checks
//
//-----------------------------------------------------------------------------------
//
// Global Data
//


unsigned char PortState;            // this is the state to be saved


//--------------------------------------------------------------------------------------

#if (TARGET_HARDWARE == OPENDECODER2)

    void init_main(void)
      {
        #if (SERVO_ENABLED == TRUE)
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (0<<JUMPER)
                  | (0<<PROGTASTER)   
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
        
            unsigned char temp;
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
             {
                PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<JUMPER)         // 1 = pullup
                      | (1<<PROGTASTER)     // 1 = pullup 
                      | (1<<SERVO1)         // output (OC1A) 
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }
            else
              {
                PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<JUMPER)         // 1 = pullup
                      | (1<<PROGTASTER)     // 1 = pullup 
                      | (0<<SERVO1)         // output (OC1A)
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }
            DDRE  = (1<<DMXDIR)         // output
                  | (1<<SERVO2);        // output (OC1B)

            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
              {
                PORTE = (1<<DMXDIR)         // 0=receive, 1=transmit
                      | (1<<SERVO2);        // output (OC1B)
              }
            else
              {
                PORTE = (1<<DMXDIR)         // 0=receive, 1=transmit
                      | (0<<SERVO2);        // output (OC1B)
              }
        #else
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (0<<JUMPER)
                  | (0<<PROGTASTER)   
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
            PORTD = (1<<RS485_RX)       // 1 = pullup
                  | (0<<RS485_TX)       // 
                  | (1<<DCCIN)          // 1 = pullup
                  | (1<<JUMPER)         // 1 = pullup
                  | (1<<PROGTASTER)     // 1 = pullup 
                  | (SERVO_INIT_PULS<<SERVO1)         // output (OC1A)  // 23.12.2008 changed to 1
                  | (0<<LED)            // LED off
                  | (0<<DCC_ACK);       // ACK off

            DDRE  = (1<<DMXDIR)         // output
                  | (1<<SERVO2);        // output (OC1B)

            PORTE = (1<<DMXDIR)         // 0=receive, 1=transmit
                  | (SERVO_INIT_PULS<<SERVO2);        // output (OC1B)
        #endif

        DDRA  = 0;                  // PORTA: inputs
        DDRB  = 0xFF;               // PortB: All Bits as Output
        DDRC  = 0;                  // PORTC: inputs
       
        PORTA = 0;                  // feedback: no pull up
        PORTB = 0;                  // output: all off     
      }

#elif (TARGET_HARDWARE == OPENDECODER25)

    void init_main(void)
      {
        #if (SERVO_ENABLED == TRUE)
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (0<<JUMPER)
                  | (1<<CRED)           // output (OC3A)
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
        
            unsigned char temp;
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
              {
                PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<JUMPER)         // 1 = pullup
                      | (0<<CRED)           //  
                      | (1<<SERVO1)         // output (OC1A)  // // puls = XXXXXXXXX_____X_____X_____X_____
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }
            else
              {
                PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<JUMPER)         // 1 = pullup
                      | (0<<CRED)           //  
                      | (0<<SERVO1)         // output (OC1A)  // // puls = ________X_____X_____X_____X_____
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }

            DDRE  = (1<<SERVO1_POWER)
                  | (1<<SERVO2_POWER)   // output
                  | (1<<SERVO2);        // output (OC1B)

            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
              {
                PORTE = (1<<SERVO1_POWER)
                      | (1<<SERVO2_POWER)   // 0=on, 1=off
                      | (1<<SERVO2);        // output (OC1B)
              }
            else
              {
                PORTE = (1<<SERVO1_POWER)
                      | (1<<SERVO2_POWER)   // 0=on, 1=off
                      | (0<<SERVO2);        // output (OC1B)
              }
        #else
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (0<<JUMPER)
                  | (1<<CRED)   
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
            DDRE  = (1<<SERVO1_POWER)
                  | (1<<SERVO2_POWER)   // output
                  | (1<<SERVO2);        // output (OC1B)
        #endif
        DDRA  = 0;                  // PORTA: inputs
        DDRB  = 0xFF;               // PortB: All Bits as Output
        DDRC  = 0;                  // PORTC: inputs
       
        PORTA = 0;                  // feedback: no pull up
        PORTB = 0;                  // output: all off     
        PORTC = 0xFF;               // pullup
      }

#elif (TARGET_HARDWARE == OPENDECODER28)

    void init_main(void)
      {
        #if (SERVO_ENABLED == TRUE)
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (1<<XPDIR)
                  | (1<<CRED)   
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
            unsigned char temp;
            temp = my_eeprom_read_byte(&CV.Sv1_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
              {
        		PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<XPDIR)          // 1 = pullup
                      | (0<<CRED)           //  
                      | (1<<SERVO1)         // output (OC1A)  // // puls = XXXXXXXXX_____X_____X_____X_____
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }
            else
              {
                PORTD = (1<<RS485_RX)       // 1 = pullup
                      | (0<<RS485_TX)       // 
                      | (1<<DCCIN)          // 1 = pullup
                      | (1<<XPDIR)          // 1 = pullup
                      | (0<<CRED)           //  
                      | (0<<SERVO1)         // output (OC1A)  // // puls = ________X_____X_____X_____X_____
                      | (0<<LED)            // LED off
                      | (0<<DCC_ACK);       // ACK off
              }

            DDRE  = (1<<SERVO1_POWER)
                  | (1<<SERVO2_POWER)   // output
                  | (1<<SERVO2);        // output (OC1B)
            temp = my_eeprom_read_byte(&CV.Sv2_Mode);
            if (temp & (1 << CVbit_SvMode_PowCtrl))
              {
                PORTE = (1<<SERVO1_POWER)
                      | (1<<SERVO2_POWER)   // 0=on, 1=off
                      | (1<<SERVO2);        // output (OC1B)
              }
            else
              {
                PORTE = (1<<SERVO1_POWER)
                      | (1<<SERVO2_POWER)   // 0=on, 1=off
                      | (0<<SERVO2);        // output (OC1B)
              }
        #else
        // bisher (03.05.2009)
            DDRD  = (0<<RS485_RX)       // input
                  | (1<<RS485_TX)       // output
                  | (0<<DCCIN) 
                  | (1<<XPDIR)
                  | (1<<CRED)   
                  | (1<<SERVO1)         // output (OC1A)
                  | (1<<LED)            // output, 1 turns on LED
                  | (1<<DCC_ACK);       // output, sending 1 makes an ACK
            PORTD = (1<<RS485_RX)       // 1 = pullup
                  | (0<<RS485_TX)       // 
                  | (1<<DCCIN)          // 1 = pullup
                  | (0<<XPDIR)          // 0 = rx ena
                  | (0<<CRED)           //  
                  | (0<<SERVO1)         // output (OC1A)  // // puls = ________X_____X_____X_____X_____
                  | (0<<LED)            // LED off
                  | (0<<DCC_ACK);       // ACK off
            DDRE  = (1<<SERVO1_POWER)
                  | (1<<SERVO2_POWER)   // output
                  | (1<<SERVO2);        // output (OC1B)
            PORTE = 0;
        #endif




        DDRA  = (1 << RELAIS1)           // outputs
              | (1 << RELAIS2) 
              | (1 << RELAIS3) 
              | (1 << RELAIS4)
              | (0 << KEY1)              // input
              | (0 << KEY2)
              | (0 << KEY3)
              | (0 << KEY4);
        PORTA = (0 << RELAIS1)           // off
              | (0 << RELAIS2) 
              | (0 << RELAIS3) 
              | (0 << RELAIS4)
              | (1 << KEY1)              // pullup
              | (1 << KEY2)
              | (1 << KEY3)
              | (1 << KEY4);
        DDRB  = (1 << CGREEN)            // outputs
              | (1 << CBLUE);
        PORTB = 0;  
        DDRC  = (0 << PROGTASTER)   //Bootloader - Proggrammiermode
              | (0 << JUMPER2)      //Freeze
              | (0 << JUMPER3)      //this is Hostmaster
              | (0 << JUMPER4)   //save last state
              | (1 << LED5)
              | (1 << LED4)
              | (1 << LED3)
              | (1 << LED2);
        PORTC = (1 << PROGTASTER)   //pullup
              | (1 << JUMPER2) 
              | (1 << JUMPER3) 
              | (1 << JUMPER4) 
              | (0 << LED5)
              | (0 << LED4)
              | (0 << LED3)
              | (0 << LED2);
      }

#elif (TARGET_HARDWARE == OPENDECODER3) 

    void init_main(void)
      {
        DDRA  = 0;                  // PORTA: nothing -> inputs
        DDRB  = (1<<LED)            // output, 1 turns on LED)
              | (1<<DMXDIR)         // 0=receive, 1=transmit
              | (0<<RS485_RX)       // input
              | (1<<RS485_TX);      // output
        DDRC  = (1<<RELAIS1)        // PORTC: 4* Relais
              | (1<<RELAIS2) 
              | (1<<RELAIS3) 
              | (1<<RELAIS4)
              | (0<<KEY1)           // input
              | (0<<KEY2)           // input
              | (0<<KEY3)           // input
              | (0<<KEY4);          // input
 
        DDRD  = (0<<LN_IN)          // input
              | (1<<RAILCOM)        // output
              | (0<<DCCIN) 
              | (0<<JUMPER2)
              | (0<<PROGTASTER)   
              | (1<<SERVO1)         // output (OC1A)
              | (1<<RAILCOM_ON)
              | (1<<DCC_ACK);       // output, sending 1 makes an ACK

        DDRE  = (0<<JUMPER1)        // input
              | (1<<SERVO2);        // output (OC1B)
       
        PORTA = 0;                  // nothing
        PORTB = (0<<LED)            // output, 1 turns on LED)
              | (1<<DMXDIR)         // 0=receive, 1=transmit
              | (1<<RS485_RX)       // input -> pullup
              | (0<<RS485_TX);      // output
        PORTC = (0<<RELAIS1)        // PORTC: 4* Relais
              | (0<<RELAIS2) 
              | (0<<RELAIS3) 
              | (0<<RELAIS4)
              | (1<<KEY1)           // input -> pullup
              | (1<<KEY2)           // input -> pullup
              | (1<<KEY3)           // input -> pullup
              | (1<<KEY4);          // input -> pullup
 
        PORTD = (1<<LN_IN)          // input -> pullup
              | (0<<RAILCOM)        // output
              | (1<<DCCIN) 
              | (1<<JUMPER2)
              | (1<<PROGTASTER)   
              | (0<<SERVO1)         // output (OC1A)
              | (0<<RAILCOM_ON)
              | (0<<DCC_ACK);       // output, sending 1 makes an ACK

        PORTE = (1<<JUMPER1)        // input
              | (0<<SERVO2);        // output (OC1B) 
      }
#endif




//------------------------------------------------------------------------
// This Routine is called when PROG is pressed
// -- manual programming and accordingly setting of CV's
//
#define DEBOUNCE  (50000L / TICK_PERIOD)
#if (DEBOUNCE == 0)
 #define DEBOUNCE  1
#endif


void DoProgramming(void)
  {
    unsigned char myCommand;
    signed char my_timerval;
    unsigned char pulsdelay;
    
       

    my_timerval = timerval;
    while(timerval - my_timerval < DEBOUNCE) ;          // wait

    if (PROG_PRESSED)                                   // still pressed?
      {
        turn_led_on();
        
        while(PROG_PRESSED) ;                           // wait for release

        my_timerval = timerval;
        while(timerval - my_timerval < DEBOUNCE) ;     // wait
        
        while(!PROG_PRESSED)
          {
            if (semaphor_get(C_Received))
              {                                         // Message
                if (analyze_message(&incoming))                  // yes, any accessory
                  {
                    my_eeprom_write_byte(&CV.myAddrL, (unsigned char) ReceivedAddr & 0b00111111  );     
                    my_eeprom_write_byte(&CV.myAddrH, (unsigned char) (ReceivedAddr >> 6) & 0b00000111);
                    
                    myCommand = ReceivedCommand & 0x07;
                    
                    if (myCommand == 0)      pulsdelay =  200000L / TICK_PERIOD; 
                    else if (myCommand == 1) pulsdelay =  500000L / TICK_PERIOD; 
                    else if (myCommand == 2) pulsdelay = 1000000L / TICK_PERIOD; 
                    else if (myCommand == 3) pulsdelay = 2000000L / TICK_PERIOD; 
                    else                     pulsdelay = 0;   // permanent; 
               
                    my_eeprom_write_byte(&CV.T_on_F1, pulsdelay);   
                    my_eeprom_write_byte(&CV.T_on_F2, pulsdelay);   
                    my_eeprom_write_byte(&CV.T_on_F3, pulsdelay);   
                    my_eeprom_write_byte(&CV.T_on_F4, pulsdelay);   
             
                    #if (NEON_ENABLED == TRUE)
                        my_eeprom_write_byte(&CV.FM, myCommand);  // save as MyOpMode
                    #endif

                    if (JUMPER_FITTED)
                      {
                        if (pulsdelay == 0)
                          {
                            my_eeprom_write_byte(&CV.LastState, 0x55);
                            my_eeprom_write_byte(&CV.FBM_F1, 0);    // 0 = COMMAND_ONLY
                            my_eeprom_write_byte(&CV.FBM_F2, 0);    // 0 = COMMAND_ONLY
                            my_eeprom_write_byte(&CV.FBM_F3, 0);    // 0 = COMMAND_ONLY
                            my_eeprom_write_byte(&CV.FBM_F4, 0);    // 0 = COMMAND_ONLY
                          }
                        else
                          {
                            my_eeprom_write_byte(&CV.LastState, 0x00);
                            my_eeprom_write_byte(&CV.FBM_F1, 1);    // 1 = END_SWITCH
                            my_eeprom_write_byte(&CV.FBM_F2, 1);    // 1 = END_SWITCH
                            my_eeprom_write_byte(&CV.FBM_F3, 1);    // 1 = END_SWITCH
                            my_eeprom_write_byte(&CV.FBM_F4, 1);    // 1 = END_SWITCH
                          }
                      }
                    
                    do {} while (!eeprom_is_ready());    // wait for write to complete
                    
                    LED_OFF;

                    // we got reprogrammed ->
                    // forget everthing running and restart decoder!                    
                    
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
                    
                    // return;  
                  }
              }
          }  // while
        turn_led_off();
        my_timerval = timerval;
        while(timerval - my_timerval < DEBOUNCE) ;     // wait    
        while(PROG_PRESSED) ;       // wait for release
      }
    return;   
  }




#define ASM_DUMMY() __asm__ __volatile__ ("" : : )

//--------------------------------------------------------------------------------------------
#if (SERVO_ENABLED == TRUE)

static void delay_1p9ms(void)
  {
    if (semaphor_query(C_Received))                     // keep DCC in alive
          {
            analyze_message(&incoming);                     // check for service mode
            semaphor_get(C_Received);                       // now take away the protection
          }
    LED_ON;
    _mydelay_us(90L);
    LED_OFF;
    _mydelay_us(1810L);
  }
  
static void delay_300ms(void)
  {
    unsigned char del;
    for (del = 0; del < 255; del++)
      {
        delay_1p9ms();       
      }
  }

static void servo_powerup_delay(void)
  {
    // a) fix delay 300ms
    // b) var. delay (dep. on address (256) 0..300ms

      unsigned char del, i;
      delay_300ms();
      del = my_eeprom_read_byte(&CV.myAddrL);
      for (i=0; i<del; i++)
        {
           delay_1p9ms();
        }
    #if (SERVO_POWER_UP == SPU_SOFT_PWM)
    #elif (SERVO_POWER_UP == SPU_PULS_ON)
        SERVO1_POWER_ON;    
        delay_300ms();
        SERVO2_POWER_ON;    
        delay_300ms();
    #elif (SERVO_POWER_UP == SPU_PULS_OFF)
        SERVO1_POWER_ON;    
        delay_300ms();
        SERVO2_POWER_ON;    
        delay_300ms();
    #else
      #warning SERVO_POWER_UP method not defined 
    #endif
  }
#endif // (SERVO_ENABLED == TRUE)

//--------------------------------------------------------------------------------------------


int main(void)
  {
    unsigned char my_mode;
    #if (SEGMENT_ENABLED == TRUE)
      unsigned char Pos_Mode;
    #endif

    // #warning  rgb_action(1);    rgb_simu();


    init_main();                                        // setup hardware ports (to do!!)

    init_port_engine();                                 // setup timers and states

    init_dcc_receiver();                                // setup dcc receiver

    init_dcc_decode();

    init_keyboard();                                    // local tracers

    #if (RGB_ENABLED == TRUE)
        init_rgb();
    #endif

    sei();                                              // Global enable interrupts

    #if (SIMULATION == 0)
    #if (SERVO_ENABLED == TRUE)
       servo_powerup_delay();
    #endif
    #endif

    my_mode = my_eeprom_read_byte(&CV.MODE);
                
    if (my_eeprom_read_byte(&CV.myAddrH) & 0x80)
      {
        flash_led_fast(5);                              // warning - we are unprogrammed
      }

    #if (SERVO_ENABLED == TRUE)
       if (my_mode==1) init_servo();                    // setup servos and recovers old position
    #endif

    #if ((SERVO_ENABLED == TRUE) && (RGB_ENABLED == TRUE))
       if (my_mode==34) init_servo();                   // setup servos and recovers old position
    #endif

    #if (SEGMENT_ENABLED == TRUE)
       if (my_mode==2) init_segment();                  // setup multiposition (requires servos)
       Pos_Mode = my_eeprom_read_byte(&CV.Pos_Mode);
    #endif



    #if (REVERSER_ENABLED == TRUE)
       init_reverser_engine();                          // setup last states for reverser
    #endif



    #if (DMX_ENABLED == TRUE)
        init_dmxout();
    #endif


    #if (SIMULATION == 3)
        rgb_action(3);
    #endif

    sei();                                              // Global enable interrupts

    while(1)
      {
        if (semaphor_query(C_Received))
          {
            if (analyze_message(&incoming) >= 2)                 // MyAdr or greater received
              {
                switch (my_mode)
                  {
                    #if (PORT_ENABLED == TRUE)
                        case 0:
                            port_action(ReceivedCommand, ReceivedActivate);   // standard accessory decoder
                            break;
                        case 3:
                            direct_action(ReceivedCommand);
                            break;
                    #endif
                    #if (SERVO_ENABLED == TRUE)
                        case 1:
                            if (ReceivedActivate)
                              {
                                servo_action(ReceivedCommand);  // servo decoder
                              }
                            break;
                    #endif
                    #if (SEGMENT_ENABLED == TRUE)
                        case 2:
                            if (ReceivedActivate)
                              {
                                servo_action2(ReceivedCommand);        // multiposition
                              }
                            break;
                    #endif
                    #if (REVERSER_ENABLED == TRUE)
                        case 5:
                            reverser_action(ReceivedCommand, ReceivedActivate);
                            break;
                    #endif
                    #if (DMX_ENABLED == TRUE)
                        case 8:
                            if (ReceivedActivate)
                              {
                                dmx_action(ReceivedCommand);        // dmx
                              }
                            break;
                    #endif
                    #if (NEON_ENABLED == TRUE)
                        case 17:
                            if (ReceivedActivate)
                              {
                                neon_action(ReceivedCommand);
                              }
                            break;
                    #endif
                    #if (RGB_ENABLED == TRUE)
                        case 33:
                            if (ReceivedActivate)
                              {
                                rgb_direct_action(ReceivedCommand);
                              }
                            break;
                        case 34:
                            if (ReceivedActivate)
                              {
                                rgb_action(ReceivedCommand);        // RGB-Fader + Servo
                              }
                            break;
                    #endif
                    default:
                        flash_led_fast(6);                  		// Error code
                        break;
                  }
              }
            semaphor_get(C_Received);                       // now take away the protection
          }

        if (semaphor_get(C_DoSave) )
          {
            if (JUMPER_FITTED)
              {
                my_eeprom_write_byte(&CV.LastState, PortState);  // i.e. PORTB   
              }
          } 

        #if (SIMULATION == 0)
            if (PROG_PRESSED) DoProgramming();
        #endif
    

        #if (SERVO_ENABLED == TRUE)
            run_servo();                                      // update servo positions    
            #if (SEGMENT_ENABLED)
                if (Pos_Mode & (1<<CVbit_PosMode_MAN))
                  {
                    #ifdef KEY_MASK
                        unsigned char mkey;
                        PORTA |= KEY_MASK;                          // all Pullup
                        mkey = keyboard();
                        if (mkey != 0XFF) servo_action2(mkey);
                    #endif
                  }
            #else
                  {
                    #ifdef KEY_MASK
                        unsigned char mkey;
                        PORTA |= KEY_MASK;                          // all Pullup
                        mkey = keyboard();
                        if (mkey != 0XFF) servo_key_action(mkey);
                    #endif
                  }
            #endif
        #endif

        #if (RGB_ENABLED == TRUE)
            run_rgb_fader();
        #endif

        #if (DMX_ENABLED == TRUE)
            run_dmxkey();                                   // tracers
            run_watchdog();
            run_dmxout();
        #endif

      }
  }


//===============================================================================
//
// Simulationen:
//
// 0: keine Simulation
// 1: Test der Empfangrootine; hierzu ein DCC Generator (aus opendcc/dccout.c)
// 2: Test der Portengine
//
#if (SIMULATION == 2)

//                                        Adressbyte   Datenbyte 
//                                         10AAAAAA    1aaaSCCR    // note: aaa = ~AAA
unsigned char message_adr001_out0_g[] = {0b10000001, 0b11111001};
unsigned char message_adr005_out1_g[] = {0b10000101, 0b11111011};
unsigned char message_adr380_out0_g[] = {0b10111100, 0b10101001};  // 380 = 0x17C = 0b101111100


// upstream interface:
struct
  {
    unsigned char size;
    unsigned char dcc[5];
  } next_message;

volatile unsigned char next_message_count;

enum do_states
  {                            // actual state
     dos_idle,
     dos_send_preamble,
     dos_send_bstart,
     dos_send_byte,
     dos_send_xor
  };

struct
  {
    enum do_states state;

    unsigned char ibyte;                            // current index of byte in message
    unsigned char bits_in_state;                    // Bits to output in this state
    unsigned char cur_byte;                         // current byte
    unsigned char xor_byte;                              // actual xor
    unsigned char current_dcc[5];                  // current message in output processing
    unsigned char bytes_in_message;                 // current size of message (decremented)
    unsigned char phase;
  } doi;


unsigned char dccbit;

void do_send(unsigned char mydccbit)
  {
    dccbit = mydccbit;
  }

void dcc_bit_generator(void)
  {
    switch (doi.state)
      {
        case dos_idle:
            do_send(1);
            if (next_message_count > 0)
              {
                memcpy(doi.current_dcc, next_message.dcc, sizeof(doi.current_dcc));
                doi.bytes_in_message = next_message.size;
                // no size checking - if (doi.cur_size > 5) doi.cur_size = 5;
                next_message_count--;
                doi.ibyte = 0;
                doi.xor_byte = 0;
                doi.bits_in_state = 14;
                doi.state = dos_send_preamble;
              }
            break;

        case dos_send_preamble:
            do_send(1);
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
                 doi.state = dos_send_bstart;
            break;

        case dos_send_bstart:
            do_send(0);
            if (doi.bytes_in_message == 0)
              { // message done, goto xor
                doi.cur_byte = doi.xor_byte;
                doi.state = dos_send_xor;
                doi.bits_in_state = 8;
              }
            else
              { // get next addr or data
                doi.bytes_in_message--;
                doi.cur_byte = doi.current_dcc[doi.ibyte++];
                doi.xor_byte ^= doi.cur_byte;
                doi.state = dos_send_byte;
                doi.bits_in_state = 8;
              }
            break;

        case dos_send_byte:
            if (doi.cur_byte & 0x80) do_send(1);
            else                    do_send(0);
            doi.cur_byte <<= 1;
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
              {
                doi.state = dos_send_bstart;
              }
            break;

        case dos_send_xor:
            if (doi.cur_byte & 0x80) do_send(1);
            else                    do_send(0);
            doi.cur_byte <<= 1;
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
              {
                doi.state = dos_idle;
              }
            break;
     }
  }

void dcc_generate_init(void)
  {
    doi.state = dos_idle;
  }


void simulat_receive(void)
  {
    dcc_generate_init();
    dcc_bit_generator();
    dcc_bit_generator();
    memcpy(next_message.dcc, message_adr001_out0_g, sizeof(doi.current_dcc));
    next_message.size = 2;
    next_message_count = 2;
    while(1)
      {
        dcc_bit_generator();
        dcc_receive();

        if (semaphor_query(C_Received) )
          {
            if (analyze_message(&incoming) == 2)     // MyAdr empfangen
              {
                port_action(ReceivedCommand, ReceivedActivate);
              }
            semaphor_get(C_Received);
          }
        if (semaphor_query(C_DoSave) )
          {
            semaphor_get(C_DoSave);
            if (JUMPER_FITTED)
              {
                my_eeprom_write_byte(&CV.LastState, PORTB);   
              } 
          } 
        // if (PROG_PRESSED) DoProgramming();         
      }
 
  }


#endif
//------------------------------------------------------------------------
