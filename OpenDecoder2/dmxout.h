//----------------------------------------------------------------
//
// OpenDCC - OpenDecoder2
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      dmxout.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-06-13 V0.1 started

// 2do:       not completed
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   builds service mode

extern unsigned char dmxmacro_active_time[];

void run_dmxout(void);     // light control

void run_dmxkey(void);     // keyboard special for dmx

void init_dmxout(void);

void dmx_action(unsigned int Command);

void run_watchdog(void);

void do_dmx(unsigned char ctrl_i);   //  activates a virtual decoder -> my be a macro or a single dmx control

// debug / simulat

void simu_dmxrampe(void);

void simu_dmxout(void);

