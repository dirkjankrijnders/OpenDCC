//----------------------------------------------------------------
//
// OpenDCC - OpenDecoder2 / OpenDecoder3 / OpenDCC
//
// Copyright (c) 2006-2007 Wolfgang Kufer
//               2003 LowLevel BitBang Routines by Mathias Dzionsko (madz@gmx.de)
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      dmxout.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-06-02 V0.01 kw started
//            2006-06-14 V0.02 kw Raumlichtsteuerung mit macros!
//            2006-09-27 V0.03 kw Documentation added
//            2007-01-15 V0.04 kw added LED Control, Bugfix in EEPROM Handling
//            2007-01-18 V0.05 kw Bugfix in do_dmx_operation
//            2007-04-24 V0.06 kw added now lowlevel mode (with UART)
//            2007-04-26 V0.07 kw div by 0 with dimmtime replaced by 1
//                                added watchdog code
//            2007-05-13 V0.08 kw corrected some init problems
//            2007-07-30 V0.09 kw added compileswitches to run on OpenDecoder3
//            2007-08-05 V0.10 kw added watch CV's
//            2007-10-25 V0.11 kw added FEEDBACK Relais Output
//            2007-11-02 V0.12 kw bei dmx_all_off werden auch die Macros gestoppt.
//                                bei dmx_all_on werden abhängig von CV.OnReact_xx 
//                                entweder 0 oder 255 ausgegeben.
//                                Bei dmx_all_on oder _off werden alle laufenden
//                                Macros und virtuellen Decoder gestoppt.
//            2007-11-07 V0.13 kw added preset-thread; access to PresetLoad
//                                loads a preset - see dmx_presets.h
//                                added RELAIS control: another 4 DMX channels
//                                these map to direkt control of Relais.
//                                (see Compileswitch RELAIS_BY_DMX)
//            2007-11-08 V0.14 kw kw MODE bit to control watchdog access to relays
//            2007-11-09 V0.15 kw added BITmode Virtual Decoder
//                                TimeH+TimeL is a bitmask, read from MSB to LSB
//                                readstep: 20ms; when 1: output TARGET, else old value
//                                these decoders are from 100 to 128.
//            2007-11-23 V0.16 kw move to OpenDecoder2, some code size optimizations
//                                Pullup on PORTA 2 and 3
//            2008-01-27 V0.17 kw default auf 72 / 18 for DMX
//
// tests:     2006-06-14 kw Test des Dämmerungsübergang via Macros -> okay
//            2007-05-13 kw Test in OpenDecoder2
//            2007-08-05 kw Test in OpenDecoder3 (TAMS)
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   genereates dmx output and calculates dimm processes
//
// howto:     DMX is generated with:
//            a) inline assembler, tuned to 16MHz
//            b) UART
//            Specs of DMX:
//            see http://www.soundlight.de/techtips/dmx512/dmx2000a.htm
//
// Note:      Inside a decoder it is necessary to allocate the
//            DMX virtual decoder space and macros space inside the CV area.
//            (Reasons: decoder reset and memory address assignment)
//            See 'DMX_MEM_LOC'
//
// interface upstream:
//            init_dmxout(void)        // init
//            run_dmxout(void)         // multitask replacement
//            run_dmxkey(void)         // tracers for DMX mode
//            set_dmxcv(entry, local_index, data)
//            do_dmx_operation(ctrl_i) // activates a virtual decoder
//            do_dmx_macro(ctrl_i)     // activates a virtual macro
//
// interface downstream:
//            a) direct mode
//               direct HW access via DMX_PORT (PORTB) and Pin DMX_OUX
//               this disables interrupt for 44us
//               (do not use on decoders)
//            b) uart driven 
//-----------------------------------------------------------------


#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>         // put var to program memory
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "config.h"                // general structures and definitions
#include "hardware.h"              // general structures and definitions
#include "port_engine.h"              // led control routines
#include "myeeprom.h"            // wrapper for eeprom



// #include "dccout.h"                // next message
// #include "parser.h"                // pc_send
// #include "status.h"                // timeout engine, set_state, keyboard
// #include "organizer.h"
// #include "rs232.h"
// #include "s88.h"

// #define SIMULAT                      // if defined: extra test code

#if (DMX_ENABLED == TRUE)

//-- system configuration

#define DMX_LOCAL_TRACERS   2           // 0: no local tracers
                                        // 1: like OpenDCC
                                        // 2: like OpenDecoder (not yet implemented)

#define DMX_INTERFACE       2           // 1: Direct BitBang Interface (do not use on decoders, timing conflicts may occur)
                                        // 2: UART (with polling) - on Atmega8515
                                        // 3: UART1 (with polling) - on Atmega162

#define DMX_DECODER         1           // 0: no special code added
                                        // 1: code for dcc-decoder added (timerval is char!)

#define WATCHDOG_ENABLED    1           // 0: no watchdog
                                        // 1: watchdog

#define RELAIS_BY_DMX       1           // 0: nothing special
                                        // 1: additional 4 DMX channels are used to control
                                        //    onboard relais for OpenDecoder3


#if (RELAIS_BY_DMX == 0)
  #define SIZE_DMX_RELAIS 0
#else
  #define SIZE_DMX_RELAIS 4             // we got four relais on OpenDecoder3
#endif

#ifndef DMX_PORT
 #warning: please define a target hardware
 #warning: I need:  DMX_OUT on DMX_PORT (which may be any IO-PORT) 
 #warning: i.e. OpenDCC:      #define DMX_OUT 4, #define DMX_PORT PORTB
 #warning: i.e. OpenDecoder2: #define DMX_OUT 1, #define DMX_PORT PORTD
 #warning: i.e. OpenDecoder3: #define DMX_OUT 3, #define DMX_PORT PORTB
#endif

#if ((TARGET_HARDWARE == OPENDECODER3) && (DMX_INTERFACE != 3))
    #warning: possible configuration mismatch - are you sure? 
#endif


#if (TARGET_HARDWARE == OPENDECODER2)
  #if (DMX_INTERFACE != 2)
    #warning: possible configuration mismatch - are you sure?
  #endif 
#endif

#define _IN_RAM         0              // only for test purposes
#define _IN_EEPROM      1              // define value to be in eeprom (no garanteed location)
#define _IN_EEPROM_FIX  2              // define value to be in eeprom (section .ee_dmx)
#define _IN_CV          3              // allocate inside CV structure


#define DMX_MEM_LOC _IN_EEPROM         // if _IN_CV: allocate DXM-Ctrl and DMX-macro to CV structure
                                       // if _IN_RAM: allocate DXM-Ctrl and DMX-macro to Data (SRAM)
                                       // if _INEEPROM: allocate DXM-Ctrl and DMX-macro to EEPROM
                                       // if _INEEPROM_FIX: allocate DXM-Ctrl and DMX-macro to EEDMX
                                       // note: due to a bug in the makefile creator of avr-studio
                                       //       the code ist not correctly allocated.
                                       //
                                       // --->>>>   please use _IN_CV only (for decoders)! 

#if (DMX_MEM_LOC == _IN_EEPROM)
  #define EEMEM_DMX EEMEM
#endif
#if (DMX_MEM_LOC == _IN_EEPROM_FIX)
  #define EEMEM_DMX __attribute__((section(".ee_dmx")))
#endif


//------------------------------------------------------------------------------
// Timing-Definitionen zur DMX Programmierung

// DMX_UPDATE_PERIOD: damit wird festgelegt, wie oft der DMX Datenstrom erzeugt wird.
// DMX_RESOLUTION:    das ist die Zeiteinheit der einzelnen virtuellen Decoder für die
//                    Helligkeitsrampe: diese haben 16 bit, so daß bei 0,1s Resolution
//                    sich bis zu 100min lange Rampen programmieren lassen.
//                    (DMX_RESOLUTION sollte ein ganzzahliges Vielfaches der
//                    DMX_UPDATE_PERIOD sein. 
// DMX_MACRO_PERIOD:  das ist die Zeiteinheit bei der Macroprogrammierung, in der
//                    die einzelnen virtuellen Decoder nacheinander aufgerufen werden.
//                    1s bedeutet max. Verzögerung von 4,2 min.
//
// Erfahrungsgemäß dauern Modellbahndämmerungen etwa 1-3min, so daß folgende Definitionen
// passen müssten:

#define  DMX_UPDATE_PERIOD    20000L    // 20ms -> 50Hz
#define  DMX_RESOLUTION      100000L
#define  DMX_MACRO_PERIOD   1000000L    // 1s -> 1Hz


// time is given in 0.1s resolution; we do DMX_UPDATE_PERIOD, therefore we have:
#define DMX_OVERSAMPL  (DMX_RESOLUTION / DMX_UPDATE_PERIOD)


// some security checks
#if (DMX_OVERSAMPL == 0)
#warning DMX - wrong timings
#endif
//------------------------------------------------------------------------------
// internal, but static:
enum dmxout_states
  {                                 // actual state
     IDLE,
     WF_TIMESLOT,                   // wait for next active slot
	 WF_PREAMBLE,                   // send preamble
	 WF_DMX_BYTES                   // send bytes
  } dmxout_state;


//-------------------------------------------------------------------------------
//
//   Data fields 
//
//   dmxvar:    'live' array for calculating data
//   dmxctrl:   array with control values (virtual decoders) -> resides in eeprom
//   dmxmacro:  list of virtual decoders to call             -> resides in eeprom
//
//--------------------------------------------------------------------------------
//
#ifndef SIZE_DMX                    // should be defined in config.h
  #warning local defines of SIZE_DMX
  #warning I am setting reasonable defaults: 24 DMX channels, 72 virtual decs., 4 macros each 18 entries
  #define SIZE_DMX        24        // number of dmx channels (max 250)
                                    // 9 bytes of RAM for each entry
  #define ESIZE_DMXCTRL   72        // number of control fields (virtual decoders)
                                    // 4 bytes of EEPROM for each entry
  #define ESIZE_DMXMACRO   4        // number of macro lists
                                    //
  #define ESIZE_MACROLIST 18        // number of entries in one macro
                                    // 2 bytes each entry
#endif

struct s_dmxvar
  {
    unsigned char dimm;             // final dimm value
    unsigned char dtype;            // 0: dimming,
                                    // 1: bitblinking: LSB of act = bitaddr, delta = pattern
    int32_t actual;                 // actual dimm value
    int32_t delta;                  // actual delta to dimm value
  };

struct s_dmxvar dmxvar[SIZE_DMX + SIZE_DMX_RELAIS];       // array for calculating data
//  {{0, (100L << 22), 0, 0   },
//   {0, (100L << 22), 0, 0   }
//  };

// Scaling: actual and delta are 32 bit value; these are signed and scaled
// 2^22; dmx-values are 8 bit; the scaling is neccesary to achieve a good
// resolution when dimming; signed is used because of the easy up-down calculation.
// one spare bit is reserved, because of overrun checks when the target is not
// met.


//--------------------------------------------------------------------------------------


typedef struct                      // virtual decoder 
  {
    unsigned char target;           // dmx channel this control acts on
	unsigned char dimm;             // final dimm value
	unsigned char time_l;           // time to reach this value, unit: 0.1s; lower 8 bits
	unsigned char time_h;           // time to reach this value, unit: 0.1s; upper 8 bits
  } t_dmxctrl;


typedef struct
  {
    struct                           // each macro consists of 16 call
      {
         unsigned char decoder;      // virtual decoder to call; 
                                     // offset of 1; void entries are 0
	     unsigned char time;         // time, when to call decoder, unit: 1s; 0..254
      }  entry[ESIZE_MACROLIST];
  }  t_dmxmacro;


#if (SIZE_DMX > 24)
 #warning: please adjust CVs for OnReact_xxx to further DMX channels
#endif



/*
struct s_dmxctrl                    // goes to EEPROM
  {
    unsigned char target;           // dmx channel this control acts on
	unsigned char dimm;             // final dimm value
	unsigned char time_l;           // time to reach this value, unit: 0.1s; lower 8 bits
	unsigned char time_h;           // time to reach this value, unit: 0.1s; upper 8 bits
  };
*/

// --------------- the folling initializing is used for following lights:
// dmx in room:
// 
//                 day east: 0        day west: 1
//
//  red morning:2                                       read evening: 6
//
//                 day east: 4        day west: 5
//
// night: 3 and 7
//
// There are 4 macros:
// macro 0  -> enter night in approx. 60s. uses virtual decoders 0..17
// macro 1  -> enter day in approx. 60s. uses virtual decoders 0..17
// macro 2  -> enter night in approx. 180s.  uses virtual decoders 20..37
// macro 3  -> enter day in approx. 180s. uses virtual decoders 20..37

#if (DMX_MEM_LOC == _IN_CV)

#else
#if (DMX_MEM_LOC == _IN_RAM)
// struct s_dmxctrl dmxctrl[ESIZE_DMXCTRL]  =      
t_dmxctrl dmxctrl[ESIZE_DMXCTRL]  =      
#else
// struct s_dmxctrl dmxctrl[ESIZE_DMXCTRL] EEMEM_DMX =      // array with control value (virtual decoder)
t_dmxctrl dmxctrl[ESIZE_DMXCTRL] EEMEM_DMX =      // array with control value (virtual decoder)
#endif
  { {   0,   0,  400 & 0xFF,  400 / 256 },    //  0: dmx 0 off    in 40 sec
    {   0, 255,  400 & 0xFF,  400 / 256 },    //  1: dmx 0 on     in 40 sec
    {   1,   0,  400 & 0xFF,  400 / 256 },    //  2: dmx 1 off    in 40 sec
    {   1, 255,  400 & 0xFF,  400 / 256 },    //  3: dmx 1 on     in 40 sec
    {   2,   0,  100 & 0xFF,  100 / 256 },    //  4: dmx 2 off    in 10 sec
    {   2, 255,  100 & 0xFF,  100 / 256 },    //  5: dmx 2 on     in 10 sec
    {   3,   0,   50 & 0xFF,   50 / 256 },    //  6: dmx 3 off    in  5 sec
    {   3, 255,   50 & 0xFF,   50 / 256 },    //  7: dmx 3 on     in  5 sec
    {   4,  30,  400 & 0xFF,  400 / 256 },    //  8: dmx 4 off    in 40 sec
    {   4, 255,  400 & 0xFF,  400 / 256 },    //  9: dmx 4 on     in 40 sec
    {   5,  30,  400 & 0xFF,  400 / 256 },    // 10: dmx 5 off    in 40 sec
    {   5, 255,  400 & 0xFF,  400 / 256 },    // 11: dmx 5 on     in 40 sec
    {   6,   0,  100 & 0xFF,  100 / 256 },    // 12: dmx 6 off    in 10 sec
    {   6, 255,  100 & 0xFF,  100 / 256 },    // 13: dmx 6 on     in 10 sec
    {   7,   0,   50 & 0xFF,   50 / 256 },    // 14: dmx 7 off    in  5 sec
    {   7, 255,   50 & 0xFF,   50 / 256 },    // 15: dmx 7 on     in  5 sec
    {   3, 130,  100 & 0xFF,  100 / 256 },    // 16: dmx 3 middle in 10 sec
    {   7, 130,  100 & 0xFF,  100 / 256 },    // 17: dmx 7 middle in 10 sec
    {   0,   0,    0 & 0xFF,    0 / 256 },    // 18: void
    {   0,   0,    0 & 0xFF,    0 / 256 },    // 19: void
    
    {   0,   0, 1200 & 0xFF, 1200 / 256 },    // 20: dmx 0 off    in 120 sec
    {   0, 255, 1200 & 0xFF, 1200 / 256 },    // 21: dmx 0 on     in 120 sec
    {   1,   0, 1200 & 0xFF, 1200 / 256 },    // 22: dmx 1 off    in 120 sec
    {   1, 255, 1200 & 0xFF, 1200 / 256 },    // 23: dmx 1 on     in 120 sec
    {   2,   0,  300 & 0xFF,  300 / 256 },    // 24: dmx 2 off    in  30 sec
    {   2, 255,  300 & 0xFF,  300 / 256 },    // 25: dmx 2 on     in  30 sec
    {   3,   0,  150 & 0xFF,  150 / 256 },    // 26: dmx 3 off    in  15 sec
    {   3, 255,  150 & 0xFF,  150 / 256 },    // 27: dmx 3 on     in  15 sec
    {   4,  30, 1200 & 0xFF, 1200 / 256 },    // 28: dmx 4 off    in 120 sec
    {   4, 255, 1200 & 0xFF, 1200 / 256 },    // 29: dmx 4 on     in 120 sec
    {   5,  30, 1200 & 0xFF, 1200 / 256 },    // 30: dmx 5 off    in 120 sec
    {   5, 255, 1200 & 0xFF, 1200 / 256 },    // 31: dmx 5 on     in 120 sec
    {   6,   0,  300 & 0xFF,  300 / 256 },    // 32: dmx 6 off    in  30 sec
    {   6, 255,  300 & 0xFF,  300 / 256 },    // 33: dmx 6 on     in  30 sec
    {   7,   0,  150 & 0xFF,  150 / 256 },    // 34: dmx 7 off    in  15 sec
    {   7, 255,  150 & 0xFF,  150 / 256 },    // 35: dmx 7 on     in  15 sec
    {   3, 130,  300 & 0xFF,  300 / 256 },    // 36: dmx 3 middle in  30 sec
    {   7, 130,  300 & 0xFF,  300 / 256 },    // 37: dmx 7 middle in  30 sec
  };
#endif

//--------------------------------------------------------------------------------------
// dmxmacro
// This is a list of virtual decoders to call during the macro
// To activate a list: set dmxmacro_active_time[list] = 0;
// Note: only chars are allowed (see eeprom_read_... calls)
//
/*
struct s_dmxmacro
  {
    struct                           // each macro consists of 16 call
      {
         unsigned char decoder;      // virtual decoder to call; 
                                     // offset of 1; void entries are 0
	     unsigned char time;         // time, when to call decoder, unit: 1s; 0..254
      }  entry[ESIZE_MACROLIST];
  };

*/

#if (DMX_MEM_LOC == _IN_CV)

#else
#if (DMX_MEM_LOC == _IN_RAM)
    t_dmxmacro dmxmacro[ESIZE_DMXMACRO] =     // array with control macros,
#else
    t_dmxmacro dmxmacro[ESIZE_DMXMACRO] EEMEM_DMX =     // array with control macros,
#endif

                                                        // goes to EEPROM
  {
   {                                            // macro 0  -> enter night
     {{ 0+1, 0},      // decoder  0 nach  0s (dmx0 - day east off) 
      { 2+1, 5},      // decoder  2 nach  5s (dmx1 - day west off)
      { 8+1, 0},      // decoder  8 nach  0s (dmx4 - day east off) 
      {10+1, 5},      // decoder 10 nach  5s (dmx5 - day west off)
      { 7+1,30},      // decoder  7 nach 30s (dmx3 - night on)
      {15+1,30},      // decoder 15 nach 30s (dmx7 - night on)
      {13+1,20},      // decoder 13 nach 20s (dmx6 - red evening on)
      {12+1,42},      // decoder 12 nach 42s (dmx6 - red evening off)
      { 4+1, 0},      // decoder  3 nach  0s (dmx2 - red morning off)
      {16+1,50},      // decoder 16 nach 50s (dmx3 - dimm down night)
      {17+1,50},      // decoder 17 nach 50s (dmx7 - dimm down night)
     },
   },
   {                                            // macro 1  -> enter day
     {{ 1+1,17},      // decoder  1 nach 17s (dmx0 - day east on) 
      { 3+1,12},      // decoder  3 nach 12s (dmx1 - day west on)
      { 9+1,17},      // decoder  9 nach 17s (dmx4 - day east on) 
      {11+1,12},      // decoder 11 nach 12s (dmx5 - day west on)
      { 6+1,22},      // decoder  6 nach 22s (dmx3 - night off)
      {14+1,22},      // decoder 14 nach 22s (dmx7 - night off)
      { 5+1, 5},      // decoder  5 nach  5s (dmx2 - red morning on)
      { 4+1,27},      // decoder  4 nach 27s (dmx2 - red morning off)
      {12+1, 0},      // decoder 12 nach  0s (dmx6 - red evening off)
      { 7+1, 0},      // decoder  7 nach  0s (dmx3 - night on)
      {15+1, 0},       //decoder 15 nach  0s (dmx7 - night on)
     },    
   },
   {                                            // macro 2  -> enter night slowly
     {{20+1,  0},     // decoder 20 nach   0s (dmx0 - day east off) 
      {22+1, 15},     // decoder 22 nach  15s (dmx1 - day west off)
      {28+1,  0},     // decoder 28 nach   0s (dmx4 - day east off) 
      {30+1, 15},     // decoder 30 nach  15s (dmx5 - day west off)
      {27+1, 90},     // decoder 27 nach  90s (dmx3 - night on)
      {35+1, 90},     // decoder 35 nach  90s (dmx7 - night on)
      {33+1, 60},     // decoder 33 nach  60s (dmx6 - red evening on)
      {32+1,126},     // decoder 32 nach 126s (dmx6 - red evening off)
      {24+1,  0},     // decoder 23 nach   0s (dmx2 - red morning off)
      {36+1,150},     // decoder 36 nach 150s (dmx3 - dimm down night)
      {37+1,150},     // decoder 37 nach 150s (dmx7 - dimm down night)
     },
   },
   {                                            // macro 3  -> enter day slowly
     {{21+1, 51},      // decoder 21 nach 51s (dmx0 - day east on) 
      {23+1, 36},      // decoder 23 nach 36s (dmx1 - day west on)
      {29+1, 51},      // decoder 29 nach 51s (dmx4 - day east on) 
      {31+1, 36},      // decoder 31 nach 36s (dmx5 - day west on)
      {26+1, 66},      // decoder 26 nach 66s (dmx3 - night off)
      {34+1, 66},      // decoder 14 nach 66s (dmx7 - night off)
      {25+1, 15},      // decoder 25 nach 15s (dmx2 - red morning on)
      {24+1, 81},      // decoder 24 nach 81s (dmx2 - red morning off)
      {32+1,  0},      // decoder 32 nach  0s (dmx6 - red evening off)
      {27+1,  0},      // decoder 27 nach  0s (dmx3 - night on)
      {35+1,  0},       //decoder 35 nach  0s (dmx7 - night on)
     },    
   }
  } ;    // array with control macros 

#endif

// 
// dmxmacro_active_time is the elapsed time (in s) of a macro
// if set to 0: restart macro
// if 255, macro has terminated and is inactiv
unsigned char dmxmacro_active_time[ESIZE_DMXMACRO]; 
                                                            
void stop_all_macros(void)
  {
    unsigned char i;
    for (i=0; i< ESIZE_DMXMACRO; i++)
      dmxmacro_active_time[i] = 255; 
  }


//---------------------------------------------------------------------------------
// calc_dmx_byte((unsigned char dmx_index)
// dmx_index: dieser Kanal wird berechnet
//
// is called every timeslot (DMX_UPDATE_PERIOD), scans the dmxvar and builds dmxbyte

unsigned char calc_dmx_byte(unsigned char dmx_index)
  {
    register unsigned char dmx;

    dmx = dmxvar[dmx_index].actual >> 22;
	if (dmxvar[dmx_index].dtype == 0)
      {
        if (dmx != dmxvar[dmx_index].dimm)
           {
    	     // perform one step
    		 dmxvar[dmx_index].actual += dmxvar[dmx_index].delta;
		 
    		 // limit to max values
             if (dmxvar[dmx_index].actual < 0) dmxvar[dmx_index].actual = 0;
             if (dmxvar[dmx_index].actual > 0x3fff0000L) dmxvar[dmx_index].actual = 0x3fff0000L;

             // this is not just actual += delta, due to rounding issues it may
    		 // happen that the target is never met -> we check target crossing

             dmx = dmxvar[dmx_index].actual >> 22;
    		 if (dmxvar[dmx_index].delta < 0)
    		   {
    		   	 if(dmx < dmxvar[dmx_index].dimm)
    			    {
    				  // under limit -> set actual at limit
                      dmxvar[dmx_index].actual = (int32_t)dmxvar[dmx_index].dimm << 22;
    				  dmx = dmxvar[dmx_index].dimm;
    				}
    		   }
             else
    		   {
    		      if(dmx > dmxvar[dmx_index].dimm)
    			    {
    				  // above limit -> set actual at limit
                      dmxvar[dmx_index].actual = (int32_t)dmxvar[dmx_index].dimm << 22;
    				  dmx = dmxvar[dmx_index].dimm;
    				}
    		   }
	      }
      }
    else // dtype = 1 (Bitfield)
      {
        unsigned char bitpos;
        bitpos = (unsigned char) dmxvar[dmx_index].actual;    // use LSBs of actual
        if (bitpos > 0)
          {
            bitpos--;
            dmxvar[dmx_index].actual--;
            if (dmxvar[dmx_index].delta & (1L << bitpos))
                dmx = dmxvar[dmx_index].dimm;
                // else: keep actual
          }
      }
    return(dmx);
  }

void dmx_all_on(void)
  {
    unsigned char i, j = 0;
    unsigned char bf_offset = 0;
    unsigned char bf = 0xff;

    stop_all_macros();

    for (i=0; i< (SIZE_DMX + SIZE_DMX_RELAIS); i++)
	  {
	    // set all outputs
        // The bit fields in OnReact_xxx determine, whether we send 255 or 0
 
        if (j == 0)
          {
             bf = my_eeprom_read_byte(&CV.OnReact_7_0 + bf_offset);
             bf_offset++;
             j = 8;
          }
        if ((bf & 0x01) == 0)
          {
            dmxvar[i].dtype = 0;
            dmxvar[i].dimm = 0;
            dmxvar[i].actual = 0;
            dmxvar[i].delta = 0;
          }
        else
          {
            dmxvar[i].dtype = 0;
    	    dmxvar[i].dimm = 255;
    		dmxvar[i].actual = 255L << 22;
            dmxvar[i].delta = 0;
          }
        bf = bf >> 1;                       // advance bit counter
        j--;
      }
  }

void dmx_all_off(void)
  {
    unsigned char i;
    
    stop_all_macros();

    for (i=0; i < (SIZE_DMX + SIZE_DMX_RELAIS); i++)
	  {
	    // set all outputs
	    dmxvar[i].dtype = 0;
        dmxvar[i].dimm = 0;
		dmxvar[i].actual = 0L << 22;
        dmxvar[i].delta = 0;
	  }
  }



//---------------------------------------------------------------------------------
// if a new dimm value should be activated, call this routine to update
// the runtime vars. Values are read from given ctrl field and put to the 
// dmx-field
//
// delta = difference (new-old) / steps 
//
// time is given in DMX_RESOLUTION; we do DMX_UPDATE_PERIOD, therefore we have
// oversammpling; this is an addtional divider when calculating delta.

#if (DMX_MEM_LOC == _IN_CV)
 #define read_dmxctrl(index, mytype)  my_eeprom_read_byte(&CV.dmxctrl[index].mytype)
#elif (DMX_MEM_LOC == _IN_RAM)
 #define read_dmxctrl(index, mytype)  dmxctrl[index].mytype
#else
 #define read_dmxctrl(index, mytype)  my_eeprom_read_byte(&dmxctrl[index].mytype)
#endif

//----------------------------------------------------------------------------------
// do_dmx_operation(unsigned char ctrl_i)
//  activates a single dmx decoder
//  parameter: ctrl_i; this decoder is activated;

void do_dmx_operation(unsigned char ctrl_i)
  {
    unsigned char dmx_i;
    int32_t newex;
    int32_t diff;
    unsigned int dimmtime;

    dmx_i = read_dmxctrl(ctrl_i, target);                   // get target
    if (dmx_i < (SIZE_DMX + SIZE_DMX_RELAIS) )              // standard DMX operation
      {
        dmxvar[dmx_i].dtype = 0;
        dmxvar[dmx_i].dimm = read_dmxctrl(ctrl_i, dimm);    // copy dimm to target

        dimmtime = read_dmxctrl(ctrl_i, time_h)*256 + read_dmxctrl(ctrl_i, time_l);
        if (dimmtime == 0) dimmtime = 1;

        newex = ((int32_t) dmxvar[dmx_i].dimm ) << 22;
        diff = newex - dmxvar[dmx_i].actual;
        dmxvar[dmx_i].delta =
            diff / 
            ( DMX_OVERSAMPL * dimmtime );
      }
    else if (dmx_i < 100 )
      { // ignored
      }
    else if (dmx_i < (100 + SIZE_DMX + SIZE_DMX_RELAIS) )   // bitfield decoder
      {
        dmx_i -= 100;               // remove offset
        dmxvar[dmx_i].dtype = 1;
        dmxvar[dmx_i].dimm = read_dmxctrl(ctrl_i, dimm);           // copy dimm to target
        dmxvar[dmx_i].actual = dmxvar[dmx_i].actual & 0xFFFFFF00L; // clear bitcounter
        dmxvar[dmx_i].actual = dmxvar[dmx_i].actual | 16; 
        dmxvar[dmx_i].delta = read_dmxctrl(ctrl_i, time_h)*256 + read_dmxctrl(ctrl_i, time_l);
      } 
  }


//----------------------------------------------------------------------------------
// do_dmx_macro(unsigned char ctrl_i)
//  activates a macro, with contains virtual decoders

void do_dmx_macro(unsigned char ctrl_i)
  {
     dmxmacro_active_time[ctrl_i] = 0;
  }

#if (DMX_DECODER == 0)
//----------------------------------------------------------------------------------
// do_dmx(unsigned char ctrl_i)
//  ctrl_i:  macro or decoder to call
//     
//  activates a virtual decoder -> my be a macro or a single dmx control
//
//           |  Range:                           |     Type
//      -----|-----------------------------------|------------------------
//           |0 ...............ESIZE_DMXMACRO:   | macro
//           |ESIZE_DMXMACRO ..(ESIZE_DMXCTRL+   | single decoder
//           |                 ESIZE_DMXMACRO )  |


void do_dmx(unsigned char ctrl_i)
  {
    if (ctrl_i < ESIZE_DMXMACRO)
      {
        do_dmx_macro(ctrl_i);
      }
    else
      {
        ctrl_i -= ESIZE_DMXMACRO;
        if (ctrl_i < ESIZE_DMXCTRL)
          {
            do_dmx_operation(ctrl_i);
          }
        // else: out of range -> not handled, just ignored
      }
  }


//----------------------------------------------------------------------------------
// set_dmxcv(entry, local_index, data)
//  sets up the CV of a specific virtual decoder.
//
//  entry:        index of dmx_control to set.
//  local_index:  0: DMX Kanal
//                1: DIMM Wert, der ausgegeben wird
//                2: low(Zeit bis zum Erreichen des Wertes)
//                3: high(Zeit bis zum Erreichen des Wertes)
//  data:         Wert
//
// Beispiel:
//      Virteller Decoder 2 soll DMX Kanal 5 auf 50 in 30 sec steuern:
//      set_dmxcv(2, CV_DMXi, 5);
//      set_dmxcv(2, CV_DIMMi, 50);
//      set_dmxcv(2, CV_TIME_Li, 300 & 0xff);
//      set_dmxcv(2, CV_TIME_Hi, 300 / 256);
  


void set_dmxcv(unsigned char ctrl_i, unsigned char sub_i, unsigned char data)
  {
    unsigned char *myctrl;

    if ((ctrl_i < ESIZE_DMXCTRL) && (sub_i < 4 /* sizeof dmxctrl[0] */))
	  {
	    #if (DMX_MEM_LOC == _IN_RAM)
            myctrl = (unsigned char *) dmxctrl;
            myctrl[ctrl_i*sizeof dmxctrl[0] + sub_i] = data;
        #elif (DMX_MEM_LOC == _IN_CV)
            myctrl = (unsigned char *) CV.dmxctrl;
            my_eeprom_write_byte(&myctrl[ctrl_i*sizeof CV.dmxctrl[0] + sub_i], data);
        #else
            myctrl = (unsigned char *) dmxctrl;
            my_eeprom_write_byte(&myctrl[ctrl_i*sizeof dmxctrl[0] + sub_i], data);
        #endif        
	  }
  }

#endif // (DMX_DECODER == 0)

#if (DMX_MEM_LOC == _IN_CV)
 #define read_dmxmacro(mymac, myentry, mytype)  my_eeprom_read_byte(&CV.dmxmacro[mymac].entry[myentry].mytype)
#elif (DMX_MEM_LOC == _IN_RAM)
 #define read_dmxmacro(mymac, myentry, mytype)  dmxmacro[mymac].entry[myentry].mytype
#else
 #define read_dmxmacro(mymac, myentry, mytype)  my_eeprom_read_byte(&dmxmacro[mymac].entry[myentry].mytype)
#endif



//===========================================================================
//
// Macro Processor
//
// Is called every second from run_dmxout; scans the dmxmacro field for decoders
// to run at the given time point. If a valid decoder is found, the corresponing
// operation is executed. (update dmxvar)
//
// howto activate a macro? -> just set dmxmacro_active_time[macro] = 0;

void proceed_dmxmacro(void)
  {
    unsigned char mymac;
    unsigned char m;
    unsigned char decoder;

    for (mymac = 0; mymac < ESIZE_DMXMACRO; mymac++)
      {
        // search through all macro lists
        if (dmxmacro_active_time[mymac] < 255)
          {
            // found an active one
            for (m=0; m<ESIZE_MACROLIST; m++)
              {
                decoder = read_dmxmacro(mymac, m, decoder);
                if ((decoder != 0) &&
                    (read_dmxmacro(mymac, m, time) == dmxmacro_active_time[mymac]))
                  {
                    // perform this call
                    do_dmx_operation(decoder-1);
    	          }
              }
            dmxmacro_active_time[mymac]++;
          }
      }
  }


//===========================================================================
//
// Lowlevel interface
//
// we support different hardware
//
// DMX_INTERFACE = 1: BitBang Interface
//               = 2: Uart driven, polled
//               = 3: Uart1 driven, polled
//
// There are three routines:
//
//   DMXSendReady: returns true if a new char could be sent
//
//   DMXSendReset: sends the initial sequence
//                                  ___10us___
//                 ___min. 88us____/          XXX DATA XXX
//
//                 |<--- Break --->|<- Mark ->|
//
//   DMXSendByte:  sends one byte (1 Start, 8 Data, 2 Stop)
//                                                     ___ ___
//                 ___| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |   |   |
//
//===========================================================================

#if (DMX_INTERFACE == 1)

//===========================================================================
//
// Lowlevel interface  (Copyright (C) 2003 Mathias Dzionsko (madz@gmx.de))
//
// Die Lowlevel Routinen auf Bitebene sind minidmx entnommen und wurden
// mit Genehmigung von Mathias hier eingebaut. Danke!
//
// This uses  DMX_PORT  and DMX_OUT

//---------------------------------------------------------------------------
// DMXSendByte
//
// Sendet ein Datenbyte über die DMX-Leitung.
// Die benötigten Takte für ein DMX-Bit sind 64 bei 16 MHz. (DMX hat 250kbit, 2%)
// Interrupts werden für 44 µs nicht zugelassen.
//
#define DMX_CYCLES_PER_BIT  (F_CPU / 250000L)
#define DMX_ASM_CYCLES  (1 +  3 +  6 +  0 *  3 +  2 +  4  )
//                      ror+call+ bit+ CNT*cycl+ jmp+ ret = 16
#define DMX_LOOP   ((DMX_CYCLES_PER_BIT - DMX_ASM_CYCLES) / 3)


// unsigned char i = DMX_LOOP;
// if F_CPU is changed, please control Bitduration: DMX_LOOP / F_CPU != 4us
// 14.06.2006: checked in Simulator
// fine adjust with one or more rjmp DMXSendBit4

#if (F_CPU != 16000000L)
  #warning: SEVERE: wrong CPU frequency for DMX BitBang Interface
#endif

uint8_t DMXSendReady(void)
  {
    return(TRUE);            // bit bang always ready
  }

void DMXSendByte(uint8_t value) __attribute__((noinline));

void DMXSendByte(uint8_t value)
{
  asm volatile (
"                cli                    \n" // keine Ints zulassen
"                clc                    \n" // 1      - Start Bit
"                rcall DMXSendBit       \n" // +3+33=37 Takte
"                                       \n"
"                ror %2                 \n" // 1      - Bit 0
"                rcall DMXSendBit       \n" // +3+33=37 Takte
"                ror %2                 \n" //        - Bit 1
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 2
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 3
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 4
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 5
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 6
"                rcall DMXSendBit       \n"
"                ror %2                 \n" //        - Bit 7
"                rcall DMXSendBit       \n"
"                                       \n"
"                sec                    \n"  //        - Stop Bit 1
"                rcall DMXSendBit       \n"
"                sec                    \n"  //        - Stop Bit 2
"                rcall DMXSendBit       \n"
"                sei                    \n"  // Ints wieder zulassen
"                rjmp DMXSendByteExit   \n"
"                                       \n"
"DMXSendBit:     brcs DMXSendBit1       \n"
"                nop                    \n" // 1+1
"                cbi %0, %1             \n" // +2
"                rjmp DMXSendBit2       \n" // +2=6 Takte
"DMXSendBit1:    sbi %0, %1             \n" // 2+2
"                rjmp DMXSendBit2       \n" // +2=6 Takte
"                                       \n"
"DMXSendBit2:    ldi r31,16             \n" // 1  : Kufer: war 6, jetzt 16 (DMX_LOOP)
"DMXSendBit3:    dec r31                \n" // +1
"                brne DMXSendBit3       \n" // +1/+2
"                rjmp DMXSendBit4       \n" // +2
"DMXSendBit4:    ret                    \n" // +4=9+6*3=27 Takte
"DMXSendByteExit:                       \n" // neu: 54 Takte
//                          %0               %1            %2
    : : "I" (_SFR_IO_ADDR(DMX_PORT)), "I" (DMX_OUT), "r" (value) : "r31"
  );
}

//---------------------------------------------------------------------------
// DMXSendReset

// Sendet ein Reset-Signal mit anschließendem Start-Byte über die
// DMX-Leitung.

void DMXSendReset(void)
  {
    DMX_PORT &= ~(1<<DMX_OUT);          // RESET-Signal senden
    _mydelay_us(100);                   // 100µs (min. 88µs) warten

    DMX_PORT |= (1<<DMX_OUT);           // MARK-Signal senden
    _mydelay_us(10);                    // 10µs (min. 8µs) warten

    DMXSendByte(0);                     // Startbyte (0) senden
  }


// endif DMX_INTERFACE == 1

//=============================================================================

#elif (DMX_INTERFACE == 2)

uint8_t DMXSendReady(void)
  {
    return(UCSRA & (1<<UDRE));            
  }

void DMXSendByte(uint8_t value)
  {
      while ( !DMXSendReady() );
      UDR = value;
  }

void DMXSendReset(void)
  {
    while ( !DMXSendReady() );
    
    DMX_PORT |= (1<<DMX_OUT);          
      
    UCSRB &= ~(1 << TXEN);              // disable UART

    DMX_PORT &= ~(1<<DMX_OUT);          // RESET-Signal senden
    _mydelay_us(100);                   // 100µs (min. 88µs) warten

    DMX_PORT |= (1<<DMX_OUT);           // MARK-Signal senden
    _mydelay_us(10);                    // 10µs (min. 8µs) warten
    
    UCSRB |= (1 << TXEN);               // enable UART;

    DMXSendByte(0);                     // Startbyte (0) senden
  }

// #endif // DMX_INTERFACE == 2


//=============================================================================
// Atmega162 Port 1

#elif (DMX_INTERFACE == 3)

uint8_t DMXSendReady(void)
  {
    return(UCSR1A & (1<<UDRE1));            
  }

void DMXSendByte(uint8_t value)
  {
      while ( !DMXSendReady() );
      UDR1 = value;
  }

void DMXSendReset(void)
  {
    while ( !DMXSendReady() );
    
    DMX_PORT |= (1<<DMX_OUT);          
      
    UCSR1B &= ~(1 << TXEN1);              // disable UART

    DMX_PORT &= ~(1<<DMX_OUT);          // RESET-Signal senden
    _mydelay_us(100);                   // 100µs (min. 88µs) warten

    DMX_PORT |= (1<<DMX_OUT);           // MARK-Signal senden
    _mydelay_us(10);                    // 10µs (min. 8µs) warten
    
    UCSR1B |= (1 << TXEN1);               // enable UART;

    DMXSendByte(0);                     // Startbyte (0) senden
  }

#else
  #warning Unsupported DMX_Interface - Code missing

#endif // DMX_INTERFACE == 3

//==============================================================================
// Predefined Setups for DMX
//
// we react on writing to CV.PresetLoad
// If there is a new number we do a copy of the corresponding preset to
// CV eeprom
//
typedef struct
  {
    t_dmxctrl dmxctrl[ESIZE_DMXCTRL];             // 80 virtual decoder
                                                  // each 4 bytes long
                                                  // 560-563, 564-567 ... 886-889 

    t_dmxmacro dmxmacro[ESIZE_DMXMACRO];          // 4 Macros, each 16 pairs
                                                  // macro 1: cv880-cv911
                                                  // macro 2: cv912-cv943
                                                  // macro 3: cv944-cv975
                                                  // macro 4: cv976-cv1007
 } t_dmx_preset;

const t_dmx_preset preset[] PROGMEM =
  {
    #include "dmx_presets.h"
  };

enum Preset_States
  {
    Preset_INIT,               
    Preset_CHECK,              
  } Preset_State;


void init_preset(void)
  {
    Preset_State = Preset_INIT;
  }


unsigned char PowerOn_PresetLoad;

void copy_preset_to_eeprom(unsigned char index)
  {
    unsigned char *eeptr;    
    const unsigned char *pgmptr;    
    unsigned int i;
    unsigned char default_value;
    unsigned char blink = 16;

    turn_led_on();

    if (index < (sizeof(preset)/sizeof(t_dmx_preset)) )
      {
        #if (DMX_MEM_LOC == _IN_CV)
            eeptr = (unsigned char *) &CV.dmxctrl[0];
        #else
          #if (DMX_MEM_LOC == _IN_RAM)
          #else
            eeptr = (unsigned char *) &dmxctrl[0];
          #endif
        #endif
        pgmptr = (const unsigned char *) &preset[index];

        for (i=0; i < sizeof(t_dmx_preset); i++)
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
      }
    else
      {                                 // index out of range - 2 sec LED
        for (i=0; i < 1000; i++)
          {
            _mydelay_us(1000L);
          }
      }
    do {} while (!eeprom_is_ready());    // wait for write to complete
    turn_led_on();
    for (i=0; i < 1000; i++)
      {
        _mydelay_us(1000L);
      }
    turn_led_off();
  }

// Multitask replacement, must be called in a loop
void run_preset_watch(void)
  {
    unsigned new_preset;

    switch (Preset_State)
          {
            default:
            case Preset_INIT:
                PowerOn_PresetLoad = my_eeprom_read_byte(&CV.PresetLoad);
                Preset_State = Preset_CHECK;
                break;
            case Preset_CHECK:
                new_preset = my_eeprom_read_byte(&CV.PresetLoad);
                if (PowerOn_PresetLoad != new_preset)
                  {
                    // we got a new preset written ...
                    copy_preset_to_eeprom(new_preset);
                  }
                PowerOn_PresetLoad = new_preset;
                break;
          }
  }



//=================================================================================
//  Relais Control

void r_output(unsigned char port_no, unsigned char state)
  {
    if (state == 0)
      {                              
        OUTPUT_PORT &= (~(1<<port_no));
      }
    else
      {
        OUTPUT_PORT |= (1<<port_no);
      }
  }

void set_relais(unsigned char index, unsigned char state)
  {
    switch(index)
      {
        case 0:
            if (state == 0)  r_output(RELAIS1,0);
            else             r_output(RELAIS1,1);
            break;
        case 1:
            if (state == 0)  r_output(RELAIS2,0);
            else             r_output(RELAIS2,1);
            break;
        case 2:
            if (state == 0)  r_output(RELAIS3,0);
            else             r_output(RELAIS3,1);
            break;
        case 3:
            if (state == 0)  r_output(RELAIS4,0);
            else             r_output(RELAIS4,1);
            break;
        default:
            break;
      }
  }     


//=================================================================================
//
// run_dmxout: multitask replacement, must be called in loop
//
//=================================================================================

unsigned char cur_dmx_chan;

#if (DMX_DECODER == 1)      
     signed char last_dmx_run;   // timer variable to create a update grid;
#else
     signed int last_dmx_run;   // timer variable to create a update grid;
#endif

signed int last_macro_run;


void run_dmxout(void)
  {
    unsigned char mytemp;

    switch (dmxout_state)
      {
        case IDLE:
            last_dmx_run = timerval;                    // remember time 
            last_macro_run = 0;
            dmxout_state = WF_TIMESLOT;
            break;

        case WF_TIMESLOT:
            // note: cast the difference down to char, otherwise the wrap around fails
            
            mytemp = timerval - last_dmx_run;
            if ((mytemp) < (DMX_UPDATE_PERIOD / TICK_PERIOD))  return;

            // thread for preset; call at this point (not to destroy a outgoing message)

            run_preset_watch();
            
            last_dmx_run = timerval;                    // remember time 
		
            last_macro_run++;
            if (last_macro_run == (DMX_MACRO_PERIOD / DMX_UPDATE_PERIOD)) 
              {
                 proceed_dmxmacro();
                 last_macro_run = 0;
              }                   

            // 20ms passed, now calc new dmx vector
			
			dmxout_state = WF_PREAMBLE;
            break;

        case WF_PREAMBLE:
            DMXSendReset();
			dmxout_state = WF_DMX_BYTES;
			cur_dmx_chan = 0;
            break;

        case WF_DMX_BYTES:
		    if (cur_dmx_chan == (SIZE_DMX + SIZE_DMX_RELAIS))
			  {
			    // all dmx done
				dmxout_state = WF_TIMESLOT;
				return;
			  }

            if (cur_dmx_chan >= SIZE_DMX)
              {
                if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_WATCH_REL))
                  {}  // relais controlled by watchdog, we do nothing
                else
                  {
                     set_relais((cur_dmx_chan - SIZE_DMX), calc_dmx_byte(cur_dmx_chan));
                  }
                cur_dmx_chan ++;
              }
            else
			  {
    			if (DMXSendReady())
                  {
                    DMXSendByte(calc_dmx_byte(cur_dmx_chan));
                    cur_dmx_chan ++;
                  }
              }
            break;
     }
  }



//==============================================================================
// Local Traces for DMX Control without DCC
//
// only consists of two tracers STOP and GO
//
// STOP: enter night macro
// GO:   enter day macro
//       double stroked: immediate all on
//

#if (DMX_LOCAL_TRACERS == 0)                        // no keyboard handling 

void run_dmxkey(void)
  {
  }

#elif (DMX_LOCAL_TRACERS == 1)                      // keyboard handling like OpenDCC

signed int last_go_stroke;
unsigned char last_go_stroke_valid;


void run_dmxkey(void)
  {
    register unsigned char temp;
   
    if (last_go_stroke_valid)
      {
        if ((timerval - last_go_stroke) > (500000L / TICK_PERIOD))
          #if (500000L / TICK_PERIOD) > 127
            #error TICK_PERIOD too small
          #endif
          {
            last_go_stroke_valid = 0;
          }
      }

    temp=run_key();                                     // this comes from status.c


    if (temp == STOP_STROKE) 
      {  
        dmxmacro_active_time[0] = 0;                    // call night macro
        // slow blinking red led, green off
        LED_GO_OFF;
        led_pwm.go_rest = 0;
            
        LED_STOP_ON;
        led_pwm.stop_rest = 100000L / TICK_PERIOD; 
        led_pwm.stop_ontime = 700000L / TICK_PERIOD;
        led_pwm.stop_offtime = 200000L / TICK_PERIOD;
      }
    if (temp == GO_STROKE) 
      {  
        if (last_go_stroke_valid)
          {
            // doppel 'klick'
            dmx_all_on();
            LED_GO_ON;                                      // fast blinking green
            led_pwm.go_rest = 100000L / TICK_PERIOD; 
            led_pwm.go_ontime = 350000L / TICK_PERIOD;
            led_pwm.go_offtime = 100000L / TICK_PERIOD;
            
          }
        else
          {
            
            dmxmacro_active_time[1] = 0;                    // call day macro
            last_go_stroke = timerval;
            last_go_stroke_valid = 1;
            
            LED_GO_ON;
            led_pwm.go_rest = 100000L / TICK_PERIOD; 
            led_pwm.go_ontime = 700000L / TICK_PERIOD;
            led_pwm.go_offtime = 200000L / TICK_PERIOD;
            LED_STOP_OFF;
            led_pwm.stop_rest = 0;
          }
      }
  }


//--------------------------------------------------------------------------------------------
#elif (DMX_LOCAL_TRACERS == 2)                        // keyboard handling for OpenDecoder

#if (TARGET_HARDWARE == OPENDECODER2)

    #define KEY_STOP   2     // in
    #define KEY_GO     3     // in

    #define KEY_GO_PRESSED   (!(PINA & (1<<KEY_GO)))
    #define KEY_STOP_PRESSED (!(PINA & (1<<KEY_STOP)))

#elif (TARGET_HARDWARE == OPENDECODER3)

    #define KEY_STOP   4     // in
    #define KEY_GO     5     // in

    #define KEY_GO_PRESSED   (!(PINC & (1<<KEY_GO)))
    #define KEY_STOP_PRESSED (!(PINC & (1<<KEY_STOP)))

#else 
    #warning: SEVERE: no TARGET defined!
#endif


#define UNPRESSED 0
#define DEBOUNCE  1
#define PRESSED   2

#define GO_STROKE    1
#define STOP_STROKE  2

#define DEBOUNCE_TIMEOUT  40000L

unsigned char key_stop = UNPRESSED;
unsigned char key_go = UNPRESSED;

signed char key_stop_timerval;
signed char key_go_timerval;


// returns 0 if no keystroke
// returns 1 if stop
// returns 2 if run

unsigned char run_key(void)
  {
    switch(key_stop)
      {
        case UNPRESSED:
            if (KEY_STOP_PRESSED)
              {
                key_stop = DEBOUNCE;
                key_stop_timerval = timerval;  
              }
            break;
        case DEBOUNCE:
            if (!KEY_STOP_PRESSED)
              {
                key_stop = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                if ((char)(timerval - key_stop_timerval) > (DEBOUNCE_TIMEOUT / TICK_PERIOD))
                  {
                    key_stop = PRESSED;
                    return(STOP_STROKE);     // exit here
                  } 
              }
            break;
        case PRESSED:
            if (!KEY_STOP_PRESSED)
              {
                key_stop = UNPRESSED;  // nochmal von vorn
              }
            break;
      }
    switch(key_go)
      {
        case UNPRESSED:
            if (KEY_GO_PRESSED)
              {
                key_go = DEBOUNCE;
                key_go_timerval = timerval;  
              }
            break;
        case DEBOUNCE:
            if (!KEY_GO_PRESSED)
              {
                key_go = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                if ((char)(timerval - key_go_timerval) > (DEBOUNCE_TIMEOUT / TICK_PERIOD))
                  {
                    key_go = PRESSED;
                    return(GO_STROKE);
                  }
                
              }
            break;
        case PRESSED:
            if (!KEY_GO_PRESSED)
              {
                key_go = UNPRESSED;  // nochmal von vorn
              }
            break;
      }
    return(UNPRESSED);
  }

signed int last_go_stroke;
unsigned char last_go_stroke_valid;


void run_dmxkey(void)
  {
    register unsigned char temp;
   
    if (last_go_stroke_valid)
      {
        if ((char)(timerval - last_go_stroke) > (500000L / TICK_PERIOD))
          #if (500000L / TICK_PERIOD) > 127
            #error TICK_PERIOD too small
          #endif
          {
            last_go_stroke_valid = 0;
          }
      }

    temp=run_key();


    if (temp == STOP_STROKE) 
      {  
        dmxmacro_active_time[0] = 0;                    // call night macro
      }
    if (temp == GO_STROKE) 
      {  
        if (last_go_stroke_valid)
          {
            // doppel 'klick'
            dmx_all_on();
          }
        else
          {
            dmxmacro_active_time[1] = 0;                    // call day macro
            last_go_stroke = timerval;
            last_go_stroke_valid = 1;
          }
      }
  }

#endif   // DMX_LOCAL_TRACERS


//===========================================================================================


void init_dmxout(void)
  {
    unsigned char i;
    dmxout_state = IDLE;

    init_preset();
    
    #if (DMX_DECODER == 1)      
        if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_INIT_STATE))
          {
            dmx_all_on();
          }
        else
          {   
            dmx_all_off();
          }
    #else
        dmx_all_off();
    #endif

    #if (TARGET_HARDWARE == OPENDECODER2)
        PORTA = (1 << KEY_STOP)   |
                (1 << KEY_GO);   // Pullup for Tracers
    #endif

    for (i=0; i<ESIZE_DMXMACRO; i++)
	  {
	    // clear all outputs
	    dmxmacro_active_time[i] = 255;     // time over, macro inactiv
      }

    #if (DMX_INTERFACE == 1)
        // no UART to initialize
    #elif (DMX_INTERFACE == 2)

        UCSRB = 0;                  // stop everything

        uint16_t ubrr;

        ubrr = (uint16_t) ((uint32_t) F_CPU/(16*250000L) - 1);      // DMX runs at 250kBaud
        UBRRH = (uint8_t) (ubrr>>8);
        UBRRL = (uint8_t) (ubrr);
        UCSRA = (1 << RXC) | (1 << TXC);
   
        // Init UART: Data mode 8N2, asynchron
    
        UCSRB = (0 << RXEN) | (0 << TXEN) | (0 << RXCIE);   // Ints not yet enabled
        UCSRC = (1 << URSEL)        // must be one
              | (0 << UMSEL)        // 0 = asyn mode
              | (0 << UPM1)         // 0 = parity disabled
              | (0 << UPM0)         // 0 = parity disabled
              | (1 << USBS)         // 1 = tx with 2 stop bits
              | (1 << UCSZ1)        // 00 = 8 or 9 bits
              | (1 << UCSZ0);

        UCSRB |= (1<<TXB8);         // set 9th bit to 1 (second Stopbit)
        UCSRB |= (1<<UCSZ2);        // set frame length to 9bit
        
        UCSRA = (1 << TXC);         // Rücksetzen von Transmit Complete-Flags
    
        UDR;

    #elif (DMX_INTERFACE == 3)

        UCSR1B = 0;                  // stop everything

        uint16_t ubrr;

        ubrr = (uint16_t) ((uint32_t) F_CPU/(16*250000L) - 1);      // DMX runs at 250kBaud
        UBRR1H = (uint8_t) (ubrr>>8);
        UBRR1L = (uint8_t) (ubrr);
        UCSR1A = (1 << RXC1) | (1 << TXC1);
   
        // Init UART Data mode 8N2, asynchron
    
        UCSR1B = (0 << RXEN1) | (0 << TXEN1) | (0 << RXCIE1);   // Ints not yet enabled
        UCSR1C = (1 << URSEL1)        // must be one
              | (0 << UMSEL1)        // 0 = asyn mode
              | (0 << UPM11)         // 0 = parity disabled
              | (0 << UPM10)         // 0 = parity disabled
              | (1 << USBS1)         // 1 = tx with 2 stop bits
              | (1 << UCSZ11)        // 00 = 8 or 9 bits
              | (1 << UCSZ10)
              | (0 << UCPOL1);

        UCSR1B |= (1<<TXB81);         // set 9th bit to 1 (second Stopbit)
        UCSR1B |= (1<<UCSZ12);        // set frame length to 9bit
        
        UCSR1A = (1 << TXC1);        // Rücksetzen von Transmit Complete-Flags
    
        UDR1;

    #else
        #warning Unsupported DMX_Interface - Code missing
    #endif
  }


//===============================================================================
//
// Watchdog
//
// PORTB:
// B0: Watchdogrelais: normally on, turns off in case of watchdog event
// B1: Alarm Output: turns on for a short time in case of watchdog event
//
// D3: JP1: Trigger Input
//
// 
//===============================================================================


#if (WATCHDOG_ENABLED == 1)  

void watchdog_trigger(void);
void watchdog_off(void);

enum Watchdog_States
  {
    Watchdog_OFF,               // turned off, after Power Up
    Watchdog_ARMED,             // armed = scharf
    Watchdog_TRIGGERED,         // triggered = ausgelöst
    Watchdog_TRIGGERED2,        // triggered and alarm time elapsed
  } Watchdog_State;


signed char wd_timerval;         // timer variable to devide down timertick to 100ms;

signed char last_watchdog_run;   // timer variable to create a update grid;
signed char alarm_time;
signed char wd_tick;

unsigned char WD_Time;            // trigger delay - unit 100ms (WATCHDOG_PERIOD)
unsigned char Alarm_Duration;     // ALARM_DURATION


#if (TARGET_HARDWARE == OPENDECODER2)
    // This is the Jumper 
    #define TRIGGER_IN    3          // applying low: emergency stop 
    #define TRIGGER_ACTIVE (!(PIND & (1<<TRIGGER_IN)))

#elif (TARGET_HARDWARE == OPENDECODER3)
    // This is Conn. 14 (key3)
    #define TRIGGER_IN    6          // applying low: emergency stop
    #define TRIGGER_ACTIVE (!(PINC & (1<<TRIGGER_IN)))
#else 
    #warning: SEVERE: no TARGET defined!
#endif


void init_watchdog(void)
  {
    WD_Time = my_eeprom_read_byte(&CV.WD_Time);
    if (WD_Time == 0) WD_Time = 1;
    if (WD_Time > 127) WD_Time = 127;
    
    Alarm_Duration = my_eeprom_read_byte(&CV.Alarm_Time);
    if (Alarm_Duration == 0) Alarm_Duration = 1;
    if (Alarm_Duration > 127) Alarm_Duration = 127;
    
    wd_timerval = timerval;

    watchdog_off();    // init ports and state
  }


// Multitask replacement, must be called in a loop
void run_watchdog(void)
  {
    if ((char)(timerval - wd_timerval) >= (100000L / TICK_PERIOD))
      {
        wd_timerval = timerval;
        wd_tick++;

        switch (Watchdog_State)
          {
            case Watchdog_OFF:
                if (TRIGGER_ACTIVE)  watchdog_trigger();
                break;

            case Watchdog_ARMED:
                if (TRIGGER_ACTIVE)
                  {
                    watchdog_trigger();
                  }
                
                // note: cast the difference down to char, otherwise the wrap around fails
                if ((char)(wd_tick - last_watchdog_run) >= WD_Time)
                  {
                    // watchdog timeout passed, now give alarm
                    alarm_time = wd_tick;              // remember time 
                    watchdog_trigger();
                  }
                break;
       
           case Watchdog_TRIGGERED:
            
                if ((char)(wd_tick - alarm_time) >= Alarm_Duration)
                  {
                    // alarm over
                    Watchdog_State = Watchdog_TRIGGERED2;
                    if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_WATCH_REL))
                      {
                        r_output(RELAIS2,0);                    // deactivate ALARM relais
                      }    
                  }
                break;

           case Watchdog_TRIGGERED2:
                break;  
            
         }
      }
  }


void watchdog_trigger(void)                 // Trigger Watchdog!
  {
    alarm_time = wd_tick;                   // remember time 
    Watchdog_State = Watchdog_TRIGGERED;

    unsigned char mode;
    mode = my_eeprom_read_byte(&CV.DMX_MODE);

    if (mode & (1 << CVbit_DMX_MODE_WATCH_REL))
      {
        r_output(RELAIS1,0);                    // deactivate DCC relais
        r_output(RELAIS2,1);                    // activate ALARM relais
        #if (TARGET_HARDWARE == OPENDECODER3)
            r_output(RELAIS3,1);                // activate FEEDBACK relais
        #endif
      }

    // check Light control
    if (mode & (1 << CVbit_DMX_MODE_WATCHDOG))
      {
        dmx_all_on();
      }
  }

void watchdog_off(void)
  {
    Watchdog_State = Watchdog_OFF;
    if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_WATCH_REL))
      {
        r_output(RELAIS1,1);                    // activate DCC relais
        r_output(RELAIS2,0);                    // deactivate ALARM relais  
        #if (TARGET_HARDWARE == OPENDECODER3)
            r_output(RELAIS3,0);                // deactivate FEEDBACK relais
        #endif
      }
  }


void watchdog_on(void)
  {
    if ((Watchdog_State == Watchdog_TRIGGERED) || (Watchdog_State == Watchdog_TRIGGERED2))
      {
                                                // do nothing, we are already triggered
      }
    else
      {
        last_watchdog_run = wd_tick;            // retrigger timing
        Watchdog_State = Watchdog_ARMED;

        if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_WATCH_REL))
          {
            r_output(RELAIS1,1);               // activate DCC relais
            r_output(RELAIS2,0);               // deactivate ALARM relais
            #if (TARGET_HARDWARE == OPENDECODER3)
                r_output(RELAIS3,0);           // deactivate FEEDBACK relais
            #endif
          }
      }
  }

#else

void init_watchdog(void) {}             // just empty calls
void run_watchdog(void) {}

#endif // (WATCHDOG_ENABLED == 1)  



#if (DMX_DECODER == 1) 

void dmx_action(unsigned int Command)
  {
    unsigned int myCommand;
    switch(Command)
      {
        case 0:
            dmx_all_off();
            break;
        case 1:
            dmx_all_on();
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            #if (ESIZE_DMXMACRO != 4)
                #warning SIZE mismatch - please check here!
            #endif
            do_dmx_macro(Command-2);
            break;
        case 6:
            #if (WATCHDOG_ENABLED == 1)
              watchdog_off();
            #endif
            break;
        case 7:
            #if (WATCHDOG_ENABLED == 1)
              watchdog_on();
            #endif
            break;
        default:
            if (my_eeprom_read_byte(&CV.DMX_MODE) & (1 << CVbit_DMX_MODE_CALL))
              {
                myCommand = Command - 8;
                if (myCommand < ESIZE_DMXCTRL)
                  {
                    do_dmx_operation(myCommand);
                  }
              }
            // else: out of range -> not handled, just ignored           
            break;
      }
  }

#endif // DMX_DECODER

#endif // (DMX_ENABLED == TRUE)


//=================================================================================
//
// Test und Simulation
//
//=================================================================================

#ifdef SIMULAT
#warning extra test code added -> turn off SIMULAT for real software

// This simulates two ramps - first up, then down;
// note: assignment to PORTx, otherwise the compiler would eat the useless code ;-)

void simu_dmxrampe(void)
  {
     int i;

     do_dmx_operation(1);
	 do_dmx_operation(3);

     for (i=0; i< 70; i++)
	   {	  
         PORTA = calc_dmx_byte(0);
         PORTB = calc_dmx_byte(1);
		 PORTC = i;
       }
     do_dmx_operation(0);
	 do_dmx_operation(2);

     for (i=0; i< 120; i++)
	   {	  
         PORTA = calc_dmx_byte(0);
         PORTB = calc_dmx_byte(1);
       }
     while(1);
  }

#define CV_DMXi     0
#define CV_DIMMi    1
#define CV_TIME_Li  2
#define CV_TIME_Hi  3


void delay20ms(void)
  {
    int i;
    for (i=0; i< 2; i++)  
	  { // each 10ms 
      _mydelay_us(10000);
      }
  }

void simu_dmxout(void)
  {
    int i, j;


 
    init_dmxout();
    // Decoder 0: DMX:0, DIMM: 50, Rampe 30s 
    set_dmxcv(0, CV_DMXi, 0);
    set_dmxcv(0, CV_DIMMi, 50);
	set_dmxcv(0, CV_TIME_Li, 300 & 0xff);
 	set_dmxcv(0, CV_TIME_Hi, 300 / 256);
    // Decoder 1: DMX:0, DIMM: 250, Rampe 2s 
    set_dmxcv(1, CV_DMXi, 0);
	set_dmxcv(1, CV_DIMMi, 250);
	set_dmxcv(1, CV_TIME_Li, 20 & 0xff);
 	set_dmxcv(1, CV_TIME_Hi, 20 / 256);


    while(1)
      {
    	do_dmx_operation(0);
    	for (j=0; j < 35*50; j++)
          {
            dmxout_state = IDLE;
            for (i=0; i< 11; i++) 
              {
                last_dmx_run = timerval - (DMX_UPDATE_PERIOD-TICK_PERIOD);
                run_dmxout(); 
              }
            delay20ms();    
          }

     	do_dmx_operation(1);
    	for (j=0; j < 10*50; j++)
          {
    		dmxout_state = IDLE;
            for (i=0; i< 11; i++) 
              {
                last_dmx_run = timerval - (DMX_UPDATE_PERIOD-TICK_PERIOD);
                run_dmxout(); 
              }
            delay20ms(); 
          }
      }
  }

#endif   // SIMULAT

