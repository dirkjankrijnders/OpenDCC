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
// file:      dcc_receiver.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-14 V0.1 kw copied from opendecoder.c
//            2007-02-26 V0.2 kw added config.h
//            2007-03-01 V0.3 kw added code for sampling receiver with
//                               lowpass filter
//                               (not yet tested)
//            2007-03-30 V0.4 kw added local memory for message, copy to
//                               global messagebuffer at end of incoming
//                               dcc-message -> this releases timing!
//                               T87us now 77us
//            2007-05-21 V0.5 kw added some comments
//            2007-02-07 V0.6 kw changed preamble detection limit 
//                               from 11 to 10 'one' bits
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: the dcc receive parts
//
// content:   A DCC-Decoder for ATmega8515 and other AVR
//
//             1. Defines and variable definitions
//             2. DCC receive routine
//             3. Test and Simulation
//
// note:      for railcom a precise detection of the packet end is
//            required to find out the cutout point!
//            this requires either:
//            a) using the old isr and correct wiring
//            b) usinge the new isr and detecting both sides of DCC
//------------------------------------------------------------------------
// used hw resources:
//
//      INT0:   DCCIN
//      Timer0: for T87us Delay 
//      Overflow Interrupt Timer0: (evaluating DCCIN Level)
//      DCC_ACK (for acknowledge)

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/pgmspace.h>        // put var to program memory
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>

#include "config.h"
#include "hardware.h"            // Port and CPU definitions
#include "dcc_receiver.h"



#define SIMULATION  0            // 0: real application
                                 // 1: test receive routine

                                 
#define ALTERNATE_RECEIVE  0     // 0: standard receiver
                                 // 1: add code for sampling receiver 
                                 


//---------------------------------------------------------------------------

// Note: ACK is done with busy waiting - will be stretched when interrupts occur.

void activate_ACK(unsigned char time)
  {
    unsigned char i;
    // set ACK for  time [ms]
    DCC_ACK_ON;
    for (i=0; i<time; i++) _mydelay_us(1000);
    DCC_ACK_OFF;    
  }


/// This are DCC timing definitions from NMRA
#define PERIOD_1   116L          // 116us for DCC 1 pulse - do not change
#define PERIOD_0   232L          // 232us for DCC 0 pulse - do not change


//
//---------------------------------------------------------------------------------
// Some tricks to optmize size and speed:
//
// 1. Save global flags (like Communicate or dcc.state) in unused IOs
// 2. Speed up port access by declaring port access as inline code
// 3. Cast logical operation down to char, whenever possible.
//    use assign to local variable and compare this var - smaller size
// 4. Same code in different cases should be identically ordered -
//    gcc reuses this code!
//
// Trick 1: Using IO as variables - what are unused IOs on ATtiny2313:
//   GPIOR0, GPIOR1, GPIOR2 General Purpose I/O Register 
//   UBRRL Baudrate, (USIDR USI Data Register)
//
//   We use on ATtiny2313:
//      GPIOR0 for general communication between processes
//      GPIOR1 for DCC ISR
//
//   unused IOs on ATmega8515:
//      SPDR. DDRC, PORTC, OCR0 



#define OPTIMIZE_VARS 0

    
#if (OPTIMIZE_VARS == 1)
    #define Recstate SPDR
#else
    unsigned char Recstate;         
#endif

t_message incoming;

volatile t_message local;


struct
    {
        unsigned char state;                    // current state
        unsigned char bitcount;                 // current bit
        unsigned char bytecount;                // pointer to current byte
        unsigned char accubyte;                 // location for bit stuffing
        signed char dcc_time;                   // integration time for dcc (only sampling code)
                                                // we start with -7 -> all values >= indicate a zero
        unsigned char filter_data;              // bitfield for low pass data
    } dccrec;

// some states:
#define RECSTAT_WF_PREAMBLE  0
#define RECSTAT_WF_LEAD0     1
#define RECSTAT_WF_BYTE      2
#define RECSTAT_WF_TRAILER   3
#define RECSTAT_WF_SECOND_H  4                  // nur bei der neuen!!!


#define RECSTAT_DCC          7   



#if (ALTERNATE_RECEIVE == 0)

void init_dcc_receiver(void)
  {

    // Init Timer0

    #define T0_PRESCALER   8    // may be 1, 8, 64, 256, 1024
    #if   (T0_PRESCALER==1)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(0<<CS01)|(1<<CS00))
    #elif (T0_PRESCALER==8)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(0<<CS00))
    #elif (T0_PRESCALER==64)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(1<<CS00))
    #elif (T0_PRESCALER==256)
        #define T0_PRESCALER_BITS   ((1<<CS02)|(0<<CS01)|(0<<CS00))
    #elif (T0_PRESCALER==1024)
        #define T0_PRESCALER_BITS   ((1<<CS02)|(0<<CS01)|(1<<CS00))
    #endif

//    #define T87US (F_CPU * PERIOD_1 * 3 / 4 / T0_PRESCALER / 1000000L)

    #define T87US (F_CPU * 77L / T0_PRESCALER / 1000000L)


    #if (T87US > 254)
      #warning T87US too big, use either larger prescaler or slower processor
    #endif
    #if (T87US < 32)
      #warning T87US too small, use either smaller prescaler or faster processor
    #endif


    TCCR0 = (0 << FOC0)        // Timer0: normal mode, but stopped
          | (0 << WGM00)       // wgm = 00: normal mode, top=0xff
          | (0 << COM01)       // com = 00: normal mode, pin operates as usual
          | (0 << COM00) 
          | (0 << WGM01)       // 
          | (0 << CS02)        // cs02.01.00
          | (0 << CS01)        //    0  0  0 = stop
          | (0 << CS00);       //    0  0  1 = run 1:1
                               //    0  1  0 = run with prescaler 8


    TCNT0 = 256L - T87US;  

    // OCR0 is unused -> Flags!

    semaphor_get(C_Received);

    TIMSK |= (1<<TOIE0);       // Timer0 Overflow

    // Init Interrupt 0

    MCUCR |= (1<<ISC01)
           | (1<<ISC00);    //  The rising edge of INT0 generates an interrupt request.

    GICR |= (1<<INT0);       // Enable INT0
          
  }



//==============================================================================
//
// Section 2
//
// DCC Receive Routine
//
// Howto:    uses two interrupt: a rising edge in DCC polarity triggers INT0;
//           in INT0, Timer0 with a delay of 87us is started.
//           On Timer0 Overflow Match the level of DCC is evaluated and
//           parsed.
//
//                           |<-----116us----->|
//
//           DCC 1: _________XXXXXXXXX_________XXXXXXXXX_________
//                           ^-INT0
//                           |----87us--->|
//                                        ^Timer-INT: reads zero
//
//           DCC 0: _________XXXXXXXXXXXXXXXXXX__________________
//                           ^-INT0
//                           |----------->|
//                                        ^Timer-INT: reads one
//           
// Result:   1. The received message is collected in the struct "local"
//           2. After receiving a complete message, data is copied to
//              "incoming".
//           3. The flag C_Received is set.
//

// here just a repetition of the defines in dcc_receiver.h
// #define MAX_DCC_SIZE  6
// typedef struct
//   {
//     unsigned char size;               // 3 .. 6, including XOR
//     unsigned char dcc[MAX_DCC_SIZE];  // the dcc content
//   } t_message;




// ISR(INT0) loads only a register and stores this register to IO.
// this influences no status flags in SREG.
// therefore we define a naked version of the ISR with
// no compiler overhead.

#define ISR_INT0_OPTIMIZED

#ifdef ISR_INT0_OPTIMIZED
    #ifdef ISR_NAKED
        #undef ISR_NAKED
    #endif

    #define ISR_NAKED(vector) \
        void vector (void) __attribute__ ((signal, naked)); \
        void vector (void)

    ISR_NAKED(INT0_vect) 
      {
         __asm__ __volatile 
          (
            "push r16"  "\n\t"
            "ldi r16, %1"  "\n\t"
            "out %0, r16" "\n\t"
            "pop r16"  "\n\t"
         :                         // no output section
         : "M" (_SFR_IO_ADDR (TCCR0)),
           "M" ( (0 << FOC0) 
               | (0 << WGM00)       // wgm = 00: normal mode, top=0xff
               | (0 << COM01)       // com = 00: normal mode, pin operates as usual
               | (0 << COM00) 
               | (0 << WGM01)       // 
               | (T0_PRESCALER_BITS)       // cs02.01.00
               )
          );
        asm volatile ( "reti" ); 
      }
#else
    ISR(INT0_vect) 
      {
        TCCR0  = (0 << FOC0)        // force output: 0=not
               | (0 << WGM00)       // wgm = 00: normal mode, top=0xff
               | (0 << COM01)       // com = 00: normal mode, pin operates as usual
               | (0 << COM00) 
               | (0 << WGM01)       // 
               | (T0_PRESCALER_BITS);    //   = run 
      }

#endif

const unsigned char copy[] PROGMEM = {"OpenDecoder2 v0.12 (c) Kufer 2010"};


ISR(TIMER0_OVF_vect)
  {
    #define mydcc (Recstate & (1<<RECSTAT_DCC))

    // read asap to keep timing!
    if (DCCIN_STATE) Recstate &= ~(1<<RECSTAT_DCC);  // if high -> mydcc=0
    else             Recstate |= 1<<RECSTAT_DCC;    

    TCCR0 = (0 << FOC0)        // force output: 0=not
           | (0 << WGM00)       // wgm = 00: normal mode, top=0xff
           | (0 << COM01)       // com = 00: normal mode, pin operates as usual
           | (0 << COM00) 
           | (0 << WGM01)       // 
           | (0 << CS02)        // cs02.01.00
           | (0 << CS01)        //    0  0  0 = stop
           | (0 << CS00);       //    0  0  1 = run 1:1

    // Interrupt occurs at MAX+1 (=256)
    // set Timer Value to 256 - (3/4 of period of a one) -> this is a time window of 116*0,75=87us
    
    TCNT0 = 256L - T87US;  

    dccrec.bitcount++;

    if (Recstate & (1<<RECSTAT_WF_PREAMBLE))            // wait for preamble
      {                                       
        if (mydcc)
          {
            if (dccrec.bitcount >= 10) 
              {
                Recstate = 1<<RECSTAT_WF_LEAD0;            
              }
          }
        else
          {
            dccrec.bitcount=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_LEAD0))          // wait for leading 0
      {
        if (mydcc)
          {                                             // still 1, wait again
          }
        else
          {
            dccrec.bytecount=0;
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_BYTE))           // wait for byte
      {
        unsigned char my_accubyte;
        my_accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            my_accubyte |= 1;
          }
        dccrec.accubyte = my_accubyte;
        
        /* dccrec.accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            dccrec.accubyte |= 1;
          }
         */
        if (dccrec.bitcount==8)
          {
            if (dccrec.bytecount == MAX_DCC_SIZE)       // too many bytes
              {                                         // ignore message
                Recstate = 1<<RECSTAT_WF_PREAMBLE;   
              }
            else
              {
                local.dcc[dccrec.bytecount++] = dccrec.accubyte;
                Recstate = 1<<RECSTAT_WF_TRAILER; 
              }
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_TRAILER))        // wait for 0 (next byte) 
      {                                                 // or 1 (eof message)
        if (mydcc)
          {  // trailing "1" received
            Recstate = 1<<RECSTAT_WF_PREAMBLE;
            dccrec.bitcount=1;

            if (semaphor_query(C_Received))
              {
                // panic - nobody is reading the messages :-((
              }
            else
              {                                         // copy from local to global
                unsigned char i;
                for (i=0; i<MAX_DCC_SIZE; i++)
                  {
                     incoming.dcc[i] = local.dcc[i];
                  }
                incoming.size = dccrec.bytecount;
                semaphor_set(C_Received);                   // ---> tell the main prog!
              }
            
          }
        else
          {
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else
      {
        Recstate = 1<<RECSTAT_WF_PREAMBLE;
      }
  }


#endif   // ALTERNATE_RECEIVE

#if (SIMULATION == 1)

unsigned char dccbit;
void simulat_receive(void);

void dcc_receive(void)
  {
    #define mydcc dccbit

    dccrec.bitcount++;

    if (Recstate & (1<<RECSTAT_WF_PREAMBLE))            // wait for preamble
      {                                       
        if (mydcc)
          {
            if (dccrec.bitcount >= 10) 
              {
                Recstate = 1<<RECSTAT_WF_LEAD0;            
              }
          }
        else
          {
            dccrec.bitcount=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_LEAD0))          // wait for leading 0
      {
        if (mydcc)
          {                                             // still 1, wait again
          }
        else
          {
            dccrec.bytecount=0;
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_BYTE))           // wait for byte
      {
        dccrec.accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            dccrec.accubyte |= 1;
          }
        if (dccrec.bitcount==8)
          {
            if (dccrec.bytecount == MAX_DCC_SIZE)        // too many bytes
              {                                         // ignore message
                Recstate = 1<<RECSTAT_WF_PREAMBLE;   
              }
            else
              {
                local.dcc[dccrec.bytecount++] = dccrec.accubyte;
                Recstate = 1<<RECSTAT_WF_TRAILER; 
              }
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_TRAILER))        // wait for 0 (next byte) 
      {                                                 // or 1 (eof message)
        if (mydcc)
          {  // trailing "1" received
            Recstate = 1<<RECSTAT_WF_PREAMBLE;
            dccrec.bitcount=1;
            
            if (semaphor_query(C_Received))
              {
                // panic - nobody is reading the messages :-((
              }
            else
              {                                         // copy from local to global
                unsigned char i;
                for (i=0; i<MAX_DCC_SIZE; i++)
                  {
                     incoming.dcc[i] = local.dcc[i];
                  }
                incoming.size = dccrec.bytecount;
                semaphor_set(C_Received);                   // ---> tell the main prog!
              }
          }
        else
          {
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else
      {
        Recstate = 1<<RECSTAT_WF_PREAMBLE;
      }
  }

#endif   // SIMULATION == 1


#if (ALTERNATE_RECEIVE == 1)

//===========================================================================
//
// DCC Routine with Sampling
// Sampling Period: 10us
// Filter Length: 5 Samples, lowpass
//
// Howto:
//  1. Filtering (using a traditional lowpass design with 5 taps,
//     thus having about 50us integration time.
//  2. Sampling the filter output with 10us period
//  3. Tracking all polarity changes and getting the message.
//

// static inline int Acceleration(int pwm)  __attribute__((always_inline));



void init_dcc_receiver(void)
  {

    // Init Timer0

    #define T0_PRESCALER   1    // may be 1, 8, 64, 256, 1024
    #if   (T0_PRESCALER==1)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(0<<CS01)|(1<<CS00))
    #elif (T0_PRESCALER==8)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(0<<CS00))
    #elif (T0_PRESCALER==64)
        #define T0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(1<<CS00))
    #elif (T0_PRESCALER==256)
        #define T0_PRESCALER_BITS   ((1<<CS02)|(0<<CS01)|(0<<CS00))
    #elif (T0_PRESCALER==1024)
        #define T0_PRESCALER_BITS   ((1<<CS02)|(0<<CS01)|(1<<CS00))
    #endif



    TCCR0 = (0 << FOC0)        // Timer0: ctc
          | (0 << WGM00)       // wgm = 00: ctc mode, top=ocr0
          | (0 << COM01)       // com = 00: normal mode, pin operates as usual
          | (0 << COM00) 
          | (1 << WGM01)       // 
          | T0_PRESCALER_BITS; 


    TCNT0 = 0;  

    OCR0 = (F_CPU * 10L / T0_PRESCALER / 1000000L);

    semaphor_get(C_Received);

    TIMSK |= (1<<OCIE0);       // Timer0 compare
  }





static inline 
  void save_bit_to_T(unsigned char myreg, unsigned char mypos) 
  __attribute__((always_inline));

void save_bit_to_T(unsigned char myreg, unsigned char mypos)
  {
    asm volatile ("bst %0,%1" :: "r" (myreg), "i" (mypos)); 
  }

static inline 
  void restore_bit_from_T(unsigned char myreg, unsigned char mypos) 
  __attribute__((always_inline));

void restore_bit_from_T(unsigned char myreg, unsigned char mypos)
  {
    asm volatile ("bld %0,%1" :: "r" (myreg), "i" (mypos)); 
  }



unsigned char dcc_filter_data;



ISR(TIMER0_COMP_vect)
  {
    unsigned char temp;
    signed char filter_val = 0;

    temp = dccrec.filter_data << 1;

    if (DCC_PORT_IN  & (1<<DCCIN))  temp |= 1;    // shift incoming data to filter

    dccrec.filter_data = temp;

    if (temp & (1<<0)) filter_val++;              // lowpass over 5 samples
    if (temp & (1<<1)) filter_val++;
    if (temp & (1<<2)) filter_val++;
    if (temp & (1<<3)) filter_val++;
    if (temp & (1<<4)) filter_val++;

    filter_val -= 3;                               // filter_val now: [-3...2]

    restore_bit_from_T(temp, 7);

    temp = temp ^ filter_val;                 // exor of sign bits

    save_bit_to_T(filter_val, 7);                 // save MSB of filter_val


    if (temp & (1<<7))                  // test sign -> if yes, we have a polarity change
      {
        // #define olddcc (Recstate & (1<<RECSTAT_DCC))
        #define olddcc (dccrec.accubyte & (1<<0)) 
        #define newdcc (dccrec.dcc_time < 0)       


        if (Recstate & (1<<RECSTAT_WF_SECOND_H))            // wait for mirror bit
          { // must have same polarity as previous
            
            if (olddcc)
              {
                if (newdcc) Recstate &= ~(1<<RECSTAT_WF_SECOND_H);
                else        Recstate = 1<<RECSTAT_WF_PREAMBLE; // restart
              }
            else
              {
                if (newdcc) Recstate = 1<<RECSTAT_WF_PREAMBLE; // restart
                else        Recstate &= ~(1<<RECSTAT_WF_SECOND_H);
              }
          }
        else
          {
            dccrec.bitcount++;

            if (Recstate & (1<<RECSTAT_WF_PREAMBLE))            // wait for preamble
              {                                       
                if (newdcc)
                  {
                    if (dccrec.bitcount >= 20) 
                      {
                        Recstate = 1<<RECSTAT_WF_LEAD0;            
                      }
                  }
                else
                  {
                    dccrec.bitcount=0;
                  }
              }
            else if (Recstate & (1<<RECSTAT_WF_LEAD0))          // wait for leading 0
              {
                if (newdcc)
                  {                                             // still 1, wait again
                  }
                else
                  {
                    dccrec.bytecount=0;
                    Recstate = (1<<RECSTAT_WF_BYTE)
                             | (1<<RECSTAT_WF_SECOND_H);
                             
                    dccrec.bitcount=0;
                    dccrec.accubyte=0;                            // expect 0 as mirror value
                  }
              }
            else if (Recstate & (1<<RECSTAT_WF_BYTE))           // wait for byte
              {
                Recstate = (1<<RECSTAT_WF_BYTE)
                         | (1<<RECSTAT_WF_SECOND_H);

                unsigned char my_accubyte;
                my_accubyte = dccrec.accubyte << 1;
                if (newdcc)
                  {
                    my_accubyte |= 1;
                  }
                dccrec.accubyte = my_accubyte;

                if (dccrec.bitcount==8)
                  {
                    if (dccrec.bytecount == MAX_DCC_SIZE)        // too many bytes
                      {                                         // ignore message
                        Recstate = 1<<RECSTAT_WF_PREAMBLE;   
                      }
                    else
                      {
                        local.dcc[dccrec.bytecount++] = dccrec.accubyte;
                        dccrec.accubyte = 0;
                        Recstate = (1<<RECSTAT_WF_TRAILER)
                                 | (1<<RECSTAT_WF_SECOND_H);
                      }
                  }
              }
            else if (Recstate & (1<<RECSTAT_WF_TRAILER))        // wait for 0 (next byte) 
              {                                                 // or 1 (eof message)
                if (newdcc)
                  {  // trailing "1" received
                    Recstate = 1<<RECSTAT_WF_PREAMBLE;
                    dccrec.bitcount=1;
                    if (semaphor_query(C_Received))
                      {
                        // panic - nobody is reading the messages :-((
                      }
                    else
                      {                                         // copy from local to global
                        unsigned char i;
                        for (i=0; i<MAX_DCC_SIZE; i++)
                          {
                             incoming.dcc[i] = local.dcc[i];
                          }
                        incoming.size = dccrec.bytecount;
                        semaphor_set(C_Received);                   // ---> tell the main prog!
                      }
                  }
                else
                  {
                    Recstate = (1<<RECSTAT_WF_BYTE)
                             | (1<<RECSTAT_WF_SECOND_H);
                    dccrec.bitcount=0;
                    dccrec.accubyte=0;
                  }
              }
            else
              {
                Recstate = 1<<RECSTAT_WF_PREAMBLE;
              }
          }
        dccrec.dcc_time = -7;                                   // restart
      }
    else
      {
        // no polarity change
        if(dccrec.dcc_time < 0) dccrec.dcc_time++;              // incr only if < 0
                                                                // this prevents overflow with zero stretched bits
      }
  }

#endif   // ALTERNATE_RECEIVE == 1

