//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder
//
// Copyright (c) 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      port_engine.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-02-14 V0.1 kw copied from opendecoder.c
//            2011-12-12         added key_action
//
//------------------------------------------------------------------------
//
// purpose:   flexible general purpose decoder for dcc
//            here: interface to servo engine
//
//------------------------------------------------------------------------

void init_servo(void);

void do_servo(unsigned char nr, unsigned char move);    // perform a move (A or B) on Servo "nr"

void servo_action(unsigned int Command);               // execute the decoded command

void init_segment(void);

void servo_action2(unsigned int Command);               // execute the decoded command

void servo_key_action(unsigned int Command);            // execute the key command

void run_servo(void);                                   // timertask, must be called in a loop
