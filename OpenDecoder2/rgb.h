//----------------------------------------------------------------
//
// OpenDCC - OpenDecoder2 / OpenDecoder3 / OpenDCC
//
// Copyright (c) 2011 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      rgb.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2011-09-20 V0.01 kw started
//
//-----------------------------------------------------------------


void init_rgb_timer(void);  // only timer part 

void init_rgb(void);        // complete init, to be called at power up


void rgb_direct_action(unsigned int Command);   // execute the decoded command

void rgb_action(unsigned int Command);   // execute the decoded command

void run_rgb_fader(void);                 // must be called in a loop

// internal only

void set_R(unsigned char red_value);

void set_G(unsigned char green_value);

void set_B(unsigned char blue_value);

