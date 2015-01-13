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
// file:      hardware.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2006-02-14 V0.1 kw copied from opendecoder.c
//            2007-05-21 V0.2 kw added OpenDecoder3
//            2008-09-28 V0.3 kw added OpenDecoder25
//            2011-11-30 V0.4 kw added OpenDecoder28
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: hardware definitions
//
//
//------------------------------------------------------------------------
#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#ifndef TARGET_HARDWARE
#warning: SEVERE: no target hardware defined! Code will not work!
#endif


//========================================================================

#if (TARGET_HARDWARE == OPENDECODER2) 
//---------------------------------------------------------------------
// CPU Definitions:
//

#if (__AVR_ATmega8515__)
  // if changed: check timings, timer settings, baudrate
  // atmega8515:   512 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    512
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATmega162__)
  // if changed: check timings, timer settings, baudrate
  // atmega162:   1024 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    1024
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATtiny84__)
	#define SRAM_SIZE    512
	#define EEPROM_SIZE  512
	#define EEPROM_BASE  0x810000L // Not used...
#elif (__AVR_ATmega328P__)
	#define SRAM_SIZE    2048
	#define EEPROM_SIZE  1024
	#define EEPROM_BASE  0x810000L // Not used...
#else
  #warning Processor
#endif


#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined", set default to 8MHz 
   # define F_CPU 8000000UL
   // if changed: check every place where it is used
   // (possible range underflow/overflow in preprocessor!)
#endif


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
#define FEEDBACK_PULLUP PORTA
#define FEEDBACK_IN     PINA

// PORTB:
#define OUTPUT_PORT     PORTB    // this is the main output port
#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais

// PORTC:
// unused on OpenDecoder2, V1.1 and V2.1 - all ports are routed to vias



// PORTD:
#define RS485_RX        0
#define RS485_TX        1
#define DCCIN           2       // must be located on INT0
#define JUMPER          3       // if Jumper fitted: state is saved in EEPROM
#define PROGTASTER      4
#define SERVO1          5       // output (OC1A)
#define LED             6       // output, 1 turns on LED
#define DCC_ACK         7       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define PROG_PRESSED    (!(PIND & (1<<PROGTASTER)))
#define JUMPER_FITTED   (!(PIND & (1<<JUMPER)))
#define LED_OFF         PORTD &= ~(1<<LED)
#define LED_ON          PORTD |= (1<<LED)
#define DCC_ACK_OFF     PORTD &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTD |= (1<<DCC_ACK)

// Note LEDs is active high -> state on == pin high!
#define LED_STATE       ((PIND & (1<<LED)))

#define DMX_OUT         1
#define DMX_PORT        PORTD


// PORTE:
#define DMXDIR          0
#define SERVO2          2      // output (OC1B)

#define DMXDIR_SEND     PORTE |= (1<<DMXDIR)
#define DMXDIR_REC      PORTE &= ~(1<<DMXDIR)

// Note: there is no Servo power switch on opendecoder2

#define SERVO2_POWER    0
#define SERVO1_POWER    1

#define SERVO2_POWER_ON   PORTE &= ~(1<<SERVO2_POWER)
#define SERVO2_POWER_OFF  PORTE |= (1<<SERVO2_POWER)
#define SERVO1_POWER_ON   PORTE &= ~(1<<SERVO1_POWER)
#define SERVO1_POWER_OFF  PORTE |= (1<<SERVO1_POWER)

// #endif // (TARGET_HARDWARE == OPENDECODER2) 

//========================================================================
#elif (TARGET_HARDWARE == OPENDECODER25) 
//---------------------------------------------------------------------
// CPU Definitions:
//

#if (__AVR_ATmega8515__)
  // if changed: check timings, timer settings, baudrate
  // atmega8515:   512 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    512
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATmega162__)
  // if changed: check timings, timer settings, baudrate
  // atmega162:   1024 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    1024
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#else
  #warning Processor
#endif


#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined", set default to 8MHz 
   # define F_CPU 8000000UL
   // if changed: check every place where it is used
   // (possible range underflow/overflow in preprocessor!)
#endif


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
#define FEEDBACK_PULLUP PORTA
#define FEEDBACK_IN     PINA
#define KEY_PORT        PINA     // This is the port where we read bits
#define KEY_MASK        0xF0     // the keyboard task runs only on
                                 // those ports defined in the mask
                                 // Ports not in the mask are ignored
#define KEY_OFFSET      -4       // shift keystroke result with this number
#define OUTPUT_PORT     PORTA    // this is the main output port
#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais


// PORTB:
#define CGREEN          1
#define CBLUE           4


// PORTC:
// unused on OpenDecoder2, V1.1 and V2.1 - all ports are routed to vias
#define PROGTASTER      7
#define PROG_PRESSED    (!(PINC & (1<<PROGTASTER)))


// PORTD:
#define RS485_RX        0
#define RS485_TX        1
#define DCCIN           2       // must be located on INT0
#define JUMPER          3       // if Jumper fitted: state is saved in EEPROM
#define CRED            4       // output (OC3A)
#define SERVO1          5       // output (OC1A)
#define LED             6       // output, 1 turns on LED
#define DCC_ACK         7       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define JUMPER_FITTED   (!(PIND & (1<<JUMPER)))
#define LED_OFF         PORTD &= ~(1<<LED)
#define LED_ON          PORTD |= (1<<LED)
#define DCC_ACK_OFF     PORTD &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTD |= (1<<DCC_ACK)

// Note LEDs is active high -> state on == pin high!
#define LED_STATE       ((PIND & (1<<LED)))


// PORTE:
#define SERVO2          2      // output (OC1B)

// used on OpenDecoder2, V2.5 - switch Servopower
#define SERVO2_POWER    0
#define SERVO1_POWER    1

#define SERVO2_POWER_ON   PORTE &= ~(1<<SERVO2_POWER)
#define SERVO2_POWER_OFF  PORTE |= (1<<SERVO2_POWER)
#define SERVO1_POWER_ON   PORTE &= ~(1<<SERVO1_POWER)
#define SERVO1_POWER_OFF  PORTE |= (1<<SERVO1_POWER)

// #endif // (TARGET_HARDWARE == OPENDECODER25) 

//========================================================================
#elif (TARGET_HARDWARE == OPENDECODER28) 
//---------------------------------------------------------------------
// CPU Definitions:
//

#if (__AVR_ATmega8515__)
  // if changed: check timings, timer settings, baudrate
  // atmega8515:   512 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    512
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATmega162__)
  // if changed: check timings, timer settings, baudrate
  // atmega162:   1024 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    1024
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#else
  #warning Processor
#endif


#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined", set default to 8MHz 
   # define F_CPU 8000000UL
   // if changed: check every place where it is used
   // (possible range underflow/overflow in preprocessor!)
#endif


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
//#define FEEDBACK_PULLUP PORTA
//#define FEEDBACK_IN     PINA

#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais
#define KEY1            4       // input, requires pull up
#define KEY2            5       // input, requires pull up
#define KEY3            6       // input, requires pull up
#define KEY4            7       // input, requires pull up

#define OUTPUT_PORT     PORTA    // this is the main output port
#define KEY_PORT        PINA     // This is the port where we read bits
#define KEY_MASK        0xF0     // the keyboard task runs only on
                                 // those ports defined in the mask
                                 // Ports not in the mask are ignored
#define KEY_OFFSET      -4       // shift keystroke result with this number



// PORTB:
//#define OUTPUT_PORT     PORTB    // this is the main output port
//#define RELAIS1         0       // output, 1 turn on relais
//#define RELAIS2         1       // output, 1 turn on relais
//#define RELAIS3         2       // output, 1 turn on relais
//#define RELAIS4         3       // output, 1 turn on relais
#define CGREEN          1
#define CBLUE           4


// PORTC:
// unused on OpenDecoder2, V1.1 and V2.1 - all ports are routed to vias
#define PROGTASTER      7      //Bootloader - Proggrammiermode
#define JUMPER2         6      //Freeze
#define JUMPER3         5      //this is Hostmaster
#define JUMPER4         4      //save last state
#define LED5            3
#define LED4            2
#define LED3            1
#define LED2            0

#define PROG_PRESSED    (!(PINC & (1<<PROGTASTER)))
#define JUMPER2_FITTED  (!(PINC & (1<<JUMPER2)))
#define JUMPER3_FITTED  (!(PINC & (1<<JUMPER3)))
#define JUMPER4_FITTED  (!(PINC & (1<<JUMPER4)))  
#define JUMPER_FITTED   JUMPER4_FITTED              // map standard Jumper to Jumper4   
#define LED2_OFF         PORTC &= ~(1<<LED2)
#define LED2_ON          PORTC |= (1<<LED2)
#define LED3_OFF         PORTC &= ~(1<<LED3)
#define LED3_ON          PORTC |= (1<<LED3)
#define LED4_OFF         PORTC &= ~(1<<LED4)
#define LED4_ON          PORTC |= (1<<LED4)
#define LED5_OFF         PORTC &= ~(1<<LED5)
#define LED5_ON          PORTC |= (1<<LED5)


// PORTD:
#define RS485_RX        0
#define RS485_TX        1
#define DCCIN           2       // must be located on INT0
#define XPDIR           3       // output, 1 = send
#define CRED            4
#define SERVO1          5       // output (OC1A)
#define LED             6       // output, 1 turns on LED
#define DCC_ACK         7       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define XPDIR_SEND      PORTD |= (1<<XPDIR)
#define XPDIR_REC       PORTD &= ~(1<<XPDIR)
#define XP_OUT          1
#define XP_PORT         PORTD

#define LED_OFF         PORTD &= ~(1<<LED)
#define LED_ON          PORTD |= (1<<LED)
#define DCC_ACK_OFF     PORTD &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTD |= (1<<DCC_ACK)

// Note LEDs is active high -> state on == pin high!
#define LED_STATE       ((PIND & (1<<LED)))


// PORTE:
#define SERVO2          2      // output (OC1B)

// used on OpenDecoder2, V2.5 - switch Servopower
#define SERVO2_POWER    0
#define SERVO1_POWER    1

#define SERVO2_POWER_ON   PORTE &= ~(1<<SERVO2_POWER)
#define SERVO2_POWER_OFF  PORTE |= (1<<SERVO2_POWER)
#define SERVO1_POWER_ON   PORTE &= ~(1<<SERVO1_POWER)
#define SERVO1_POWER_OFF  PORTE |= (1<<SERVO1_POWER)

// #endif // (TARGET_HARDWARE == OPENDECODER25) 


//========================================================================
#elif (TARGET_HARDWARE == OPENDECODER3) 
//---------------------------------------------------------------------
// CPU Definitions:
//

#if (__AVR_ATmega162__)
  // if changed: check timings, timer settings, baudrate
  // atmega162:   1024 Byte SRAM, 512 Byte EEPROM
  #define SRAM_SIZE    1024
  #define EEPROM_SIZE  512
  #define EEPROM_BASE  0x810000L
#else
  #warning Processor
#endif


#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined", set default to 8MHz 
   # define F_CPU 8000000UL
   // if changed: check every place where it is used
   // (possible range underflow/overflow in preprocessor!)
#endif

//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
//#define FEEDBACK_PULLUP PORTA
//#define FEEDBACK_IN     PINA

// PORTB:
#define LED             0       // output, 1 turns on LED
#define DMXDIR          1       // output, 1 = send
#define RS485_RX        2
#define RS485_TX        3


#define LED_OFF         PORTB &= ~(1<<LED)      // use only these macros to access LED
#define LED_ON          PORTB |= (1<<LED)
#define LED_STATE       ((PINB & (1<<LED)))

#define DMXDIR_SEND     PORTB |= (1<<DMXDIR)
#define DMXDIR_REC      PORTB &= ~(1<<DMXDIR)

#define DMX_OUT         3
#define DMX_PORT        PORTB


// PORTC:
#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais
#define KEY1            4       // input, requires pull up
#define KEY2            5       // input, requires pull up
#define KEY3            6       // input, requires pull up
#define KEY4            7       // input, requires pull up

#define OUTPUT_PORT     PORTC    // this is the main output port
#define KEY_PORT        PINC     // This is the port where we read bits
#define KEY_MASK        0xF0     // the keyboard task runs only on
                                 // those ports defined in the mask
                                 // Ports not in the mask are ignored
#define KEY_OFFSET      -4       // shift keystroke result with this number



// PORTD:
#define LN_IN           0
#define RAILCOM         1       // output (tx0)
#define DCCIN           2       // must be located on INT0
#define JUMPER2         3       // if Jumper fitted: state is saved in EEPROM
#define PROGTASTER      4
#define SERVO1          5       // output (OC1A)
#define RAILCOM_ON      6       // output, 1 turns on LED
#define DCC_ACK         7       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define PROG_PRESSED    (!(PIND & (1<<PROGTASTER)))
#define JUMPER2_FITTED  (!(PIND & (1<<JUMPER2)))
#define DCC_ACK_OFF     PORTD &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTD |= (1<<DCC_ACK)


// PORTE:
#define RESERV_E0       0       // input, pullup, external resistor
#define JUMPER1         1       // 
#define SERVO2          2       // output (OC1B)
#define JUMPER1_FITTED  (!(PINE & (1<<JUMPER1)))


//----------------------------------------------------------------------------
//
// add Your Target Hardware here!
//
#elif (TARGET_HARDWARE == ARDUINOPROMINI)
//---------------------------------------------------------------------
// CPU Definitions:
//


#ifndef F_CPU
// prevent compiler error by supplying a default
# warning "F_CPU not defined", set default to 16MHz
# define F_CPU 16000000UL
// if changed: check every place where it is used
// (possible range underflow/overflow in preprocessor!)
#endif


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
//#define FEEDBACK_PULLUP PORTA
//#define FEEDBACK_IN     PINA

#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais
#define KEY1            4       // input, requires pull up
#define KEY2            5       // input, requires pull up
#define KEY3            6       // input, requires pull up
#define KEY4            7       // input, requires pull up

#define OUTPUT_PORT     PORTA    // this is the main output port
#define KEY_PORT        PINA     // This is the port where we read bits
#define KEY_MASK        0xF0     // the keyboard task runs only on
// those ports defined in the mask
// Ports not in the mask are ignored
#define KEY_OFFSET      -4       // shift keystroke result with this number



// PORTB:
//#define OUTPUT_PORT     PORTB    // this is the main output port
//#define RELAIS1         0       // output, 1 turn on relais
//#define RELAIS2         1       // output, 1 turn on relais
//#define RELAIS3         2       // output, 1 turn on relais
//#define RELAIS4         3       // output, 1 turn on relais
#define CGREEN          1
#define CBLUE           4


// PORTC:
// unused on OpenDecoder2, V1.1 and V2.1 - all ports are routed to vias
#define PROGTASTER      7      //Bootloader - Proggrammiermode
#define JUMPER2         6      //Freeze
#define JUMPER3         5      //this is Hostmaster
#define JUMPER4         4      //save last state
#define LED5            3
#define LED4            2
#define LED3            1
#define LED2            0

#define PROG_PRESSED    (!(PINC & (1<<PROGTASTER)))
#define JUMPER2_FITTED  (!(PINC & (1<<JUMPER2)))
#define JUMPER3_FITTED  (!(PINC & (1<<JUMPER3)))
#define JUMPER4_FITTED  (!(PINC & (1<<JUMPER4)))
#define JUMPER_FITTED   JUMPER4_FITTED              // map standard Jumper to Jumper4
#define LED2_OFF         PORTC &= ~(1<<LED2)
#define LED2_ON          PORTC |= (1<<LED2)
#define LED3_OFF         PORTC &= ~(1<<LED3)
#define LED3_ON          PORTC |= (1<<LED3)
#define LED4_OFF         PORTC &= ~(1<<LED4)
#define LED4_ON          PORTC |= (1<<LED4)
#define LED5_OFF         PORTC &= ~(1<<LED5)
#define LED5_ON          PORTC |= (1<<LED5)


// PORTD:
#define RS485_RX        0
#define RS485_TX        1
#define DCCIN           2       // must be located on INT0
#define XPDIR           3       // output, 1 = send
#define CRED            4
#define SERVO1          5       // output (OC1A)
#define LED             PB5       // output, 1 turns on LED
#define DCC_ACK         PB2       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define XPDIR_SEND      PORTD |= (1<<XPDIR)
#define XPDIR_REC       PORTD &= ~(1<<XPDIR)
#define XP_OUT          1
#define XP_PORT         PORTD

#define LED_OFF         PORTB &= ~(1<<LED)
#define LED_ON          PORTB |= (1<<LED)
#define DCC_ACK_OFF     PORTB &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTB |= (1<<DCC_ACK)

// Note LEDs is active high -> state on == pin high!
#define LED_STATE       ((PIND & (1<<LED)))


// PORTE:
#define SERVO2          2      // output (OC1B)

// used on OpenDecoder2, V2.5 - switch Servopower
#define SERVO2_POWER    0
#define SERVO1_POWER    1

#define SERVO2_POWER_ON   PORTE &= ~(1<<SERVO2_POWER)
#define SERVO2_POWER_OFF  PORTE |= (1<<SERVO2_POWER)
#define SERVO1_POWER_ON   PORTE &= ~(1<<SERVO1_POWER)
#define SERVO1_POWER_OFF  PORTE |= (1<<SERVO1_POWER)

// #endif // (TARGET_HARDWARE == ARDUINOPROMIMI)


//========================================================================
#elif (TARGET_HARDWARE == CSMD)
//---------------------------------------------------------------------
// CPU Definitions:
//


#ifndef F_CPU
// prevent compiler error by supplying a default
# warning "F_CPU not defined", set default to 16MHz
# define F_CPU 16000000UL
// if changed: check every place where it is used
// (possible range underflow/overflow in preprocessor!)
#endif


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTA:
//#define FEEDBACK_PULLUP PORTA
//#define FEEDBACK_IN     PINA

#define RELAIS1         0       // output, 1 turn on relais
#define RELAIS2         1       // output, 1 turn on relais
#define RELAIS3         2       // output, 1 turn on relais
#define RELAIS4         3       // output, 1 turn on relais
#define KEY1            4       // input, requires pull up
#define KEY2            5       // input, requires pull up
#define KEY3            6       // input, requires pull up
#define KEY4            7       // input, requires pull up

#define OUTPUT_PORT     PORTA    // this is the main output port
#define KEY_PORT        PINA     // This is the port where we read bits
#define KEY_MASK        0xF0     // the keyboard task runs only on
// those ports defined in the mask
// Ports not in the mask are ignored
#define KEY_OFFSET      -4       // shift keystroke result with this number



// PORTB:
//#define OUTPUT_PORT     PORTB    // this is the main output port
//#define RELAIS1         0       // output, 1 turn on relais
//#define RELAIS2         1       // output, 1 turn on relais
//#define RELAIS3         2       // output, 1 turn on relais
//#define RELAIS4         3       // output, 1 turn on relais
#define CGREEN          1
#define CBLUE           4


// PORTC:
// unused on OpenDecoder2, V1.1 and V2.1 - all ports are routed to vias
#define PROGTASTER      7      //Bootloader - Proggrammiermode
#define JUMPER2         6      //Freeze
#define JUMPER3         5      //this is Hostmaster
#define JUMPER4         4      //save last state
#define LED5            3
#define LED4            2
#define LED3            1
#define LED2            0

#define PROG_PRESSED    (!(PINC & (1<<PROGTASTER)))
#define JUMPER2_FITTED  (!(PINC & (1<<JUMPER2)))
#define JUMPER3_FITTED  (!(PINC & (1<<JUMPER3)))
#define JUMPER4_FITTED  (!(PINC & (1<<JUMPER4)))
#define JUMPER_FITTED   JUMPER4_FITTED              // map standard Jumper to Jumper4
#define LED2_OFF         PORTC &= ~(1<<LED2)
#define LED2_ON          PORTC |= (1<<LED2)
#define LED3_OFF         PORTC &= ~(1<<LED3)
#define LED3_ON          PORTC |= (1<<LED3)
#define LED4_OFF         PORTC &= ~(1<<LED4)
#define LED4_ON          PORTC |= (1<<LED4)
#define LED5_OFF         PORTC &= ~(1<<LED5)
#define LED5_ON          PORTC |= (1<<LED5)


// PORTD:
#define RS485_RX        0
#define RS485_TX        1
#define DCCIN           2       // must be located on INT0
#define XPDIR           3       // output, 1 = send
#define CRED            4
#define SERVO1          5       // output (OC1A)
#define LED             6       // output, 1 turns on LED
#define DCC_ACK         PB2       // output, sending 1 makes an ACK

#define DCC_PORT        PORTD   // must be defined to have portable code
#define DCC_PORT_IN     PIND    // must be defined to have portable code

#define DCCIN_STATE     (PIND & (1<<DCCIN))

#define XPDIR_SEND      PORTD |= (1<<XPDIR)
#define XPDIR_REC       PORTD &= ~(1<<XPDIR)
#define XP_OUT          1
#define XP_PORT         PORTD

#define LED_OFF         PORTD &= ~(1<<LED)
#define LED_ON          PORTD |= (1<<LED)
#define DCC_ACK_OFF     PORTB &= ~(1<<DCC_ACK)
#define DCC_ACK_ON      PORTB |= (1<<DCC_ACK)

// Note LEDs is active high -> state on == pin high!
#define LED_STATE       ((PIND & (1<<LED)))


// PORTE:
#define SERVO2          2      // output (OC1B)

// used on OpenDecoder2, V2.5 - switch Servopower
#define SERVO2_POWER    0
#define SERVO1_POWER    1

#define SERVO2_POWER_ON   PORTE &= ~(1<<SERVO2_POWER)
#define SERVO2_POWER_OFF  PORTE |= (1<<SERVO2_POWER)
#define SERVO1_POWER_ON   PORTE &= ~(1<<SERVO1_POWER)
#define SERVO1_POWER_OFF  PORTE |= (1<<SERVO1_POWER)

// #endif // (TARGET_HARDWARE == CSMD)


//========================================================================
#else
   #warning: SEVERE: unknown target hardware defined! Code will not work!
#endif // TARGET_HARDWARE

#endif // _HARDWARE_
