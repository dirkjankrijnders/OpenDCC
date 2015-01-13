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
// file:      dmx_presets.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2007-11-06 V0.1 kw start
//
//------------------------------------------------------------------------
//
// purpose:   dmx decoder for dcc
//            This file contains the data for the dmx presets
//
//------------------------------------------------------------------------
//
// Content:
// Preset:
// 0:       all empty
// 1:       direct mode, rgb
// 2:       3 bulb setup; dusk, dawn, rainy day, thunderstorm


//=========================================================================
// PRESET 0: all empty
{
  {                                            // virtual decoders
    {   0,   0,    0 & 0xFF,    0 / 256 },          //  0: void
    {   0,   0,    0 & 0xFF,    0 / 256 },          //  0: void
    {   0,   0,    0 & 0xFF,    0 / 256 },          //  0: void
  },
  {
   {                                            // macro 0  
     {{   0, 0},      // void
     },
   },
   {                                            // macro 1  
     {{   0, 0},      // void 
     },    
   },
   {                                            // macro 2  
     {{  0,  0},     //  void
     },
   },
   {                                            // macro 3  
     {{  0,  0},      // void
     },    
   }
  },
},
//=========================================================================
// PRESET 1: assign each pair of decoders to a dmx on off
// ramp time default 4s
// macros: 1: 8 lamps on after each other
//         2: 8 lamps off after each other
//         3: rgb circular
#define RAMP_TIME1  40          // unit 100ms
{
  {
    {   0,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  0: dmx 0 off    in  4 sec
    {   0, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  1: dmx 0 on     in  4 sec
    {   1,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  2: dmx 1 off    in  4 sec
    {   1, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  3: dmx 1 on     in  4 sec
    {   2,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  4: dmx 2 off    in  4 sec
    {   2, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  5: dmx 2 on     in  4 sec
    {   3,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  6: dmx 3 off    in  4 sec
    {   3, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  7: dmx 3 on     in  4 sec
    {   4,  30,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  8: dmx 4 off    in  4 sec
    {   4, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    //  9: dmx 4 on     in  4 sec
    {   5,  30,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 10: dmx 5 off    in  4 sec
    {   5, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 11: dmx 5 on     in  4 sec
    {   6,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 12: dmx 6 off    in  4 sec
    {   6, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 13: dmx 6 on     in  4 sec
    {   7,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 14: dmx 7 off    in  4 sec
    {   7, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 15: dmx 7 on     in  4 sec
    {   8,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 16: dmx 8 off    in  4 sec
    {   8, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 17: dmx 8 on     in  4 sec
    {   9,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 18: dmx 9 off    in  4 sec
    {   9, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 19: dmx 9 on     in  4 sec
    {  10,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 20: dmx10 off    in  4 sec
    {  10, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 21: dmx10 on     in  4 sec
    {  11,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 22: dmx11 off    in  4 sec
    {  11, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 23: dmx11 on     in  4 sec
    {  12,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 24: dmx12 off    in  4 sec
    {  12, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 25: dmx12 on     in  4 sec
    {  13,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 26: dmx13 off    in  4 sec
    {  13, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 27: dmx13 on     in  4 sec
    {  14,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 28: dmx14 off    in  4 sec
    {  14, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 29: dmx14 on     in  4 sec
    {  15,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 30: dmx15 off    in  4 sec
    {  15, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 31: dmx15 on     in  4 sec
    {  16,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 32: dmx16 off    in  4 sec
    {  16, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 33: dmx16 on     in  4 sec
    {  17,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 34: dmx17 off    in  4 sec
    {  17, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 35: dmx17 on     in  4 sec
    {  18,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 36: dmx18 off    in  4 sec
    {  18, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 37: dmx18 on     in  4 sec
    {  19,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 38: dmx19 off    in  4 sec
    {  19, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 39: dmx19 on     in  4 sec
    {  20,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 40: dmx20 off    in  4 sec
    {  20, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 41: dmx20 on     in  4 sec
    {  21,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 42: dmx21 off    in  4 sec
    {  21, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 43: dmx21 on     in  4 sec
    {  22,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 44: dmx22 off    in  4 sec
    {  22, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 45: dmx22 on     in  4 sec
    {  23,   0,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 46: dmx23 off    in  4 sec
    {  23, 255,   RAMP_TIME1 & 0xFF,   RAMP_TIME1 / 256 },    // 47: dmx23 on     in  4 sec
  },
  {
   {                                            // macro 0  -> light column off
     {{ 0+1, 0*(RAMP_TIME1/20)},      // decoder  0 nach  0s (dmx0 - off) 
      { 2+1, 2*(RAMP_TIME1/20)},      // decoder  2 nach  2s (dmx1 - off)
      { 4+1, 4*(RAMP_TIME1/20)},      // decoder  4 nach  4s (dmx2 - off) 
      { 6+1, 6*(RAMP_TIME1/20)},      // decoder  6 nach  6s (dmx3 - off)
      { 8+1, 8*(RAMP_TIME1/20)},      // decoder  8 nach  8s (dmx4 - off)
      {10+1,10*(RAMP_TIME1/20)},      // decoder 10 nach 10s (dmx5 - off)
      {12+1,12*(RAMP_TIME1/20)},      // decoder 12 nach 12s (dmx6 - off)
      {14+1,14*(RAMP_TIME1/20)},      // decoder 14 nach 14s (dmx7 - off)
      {16+1,16*(RAMP_TIME1/20)},      // decoder 16 nach 16s (dmx8 - off)
     },    
   },
   {                                            // macro 1  -> light column on
     {{ 1+1, 0*(RAMP_TIME1/20)},      // decoder  1 nach  0s (dmx0 - on) 
      { 3+1, 2*(RAMP_TIME1/20)},      // decoder  3 nach  2s (dmx1 - on)
      { 5+1, 4*(RAMP_TIME1/20)},      // decoder  5 nach  4s (dmx2 - on) 
      { 7+1, 6*(RAMP_TIME1/20)},      // decoder  7 nach  6s (dmx3 - on)
      { 9+1, 8*(RAMP_TIME1/20)},      // decoder  9 nach  8s (dmx4 - on)
      {11+1,10*(RAMP_TIME1/20)},      // decoder 11 nach 10s (dmx5 - on)
      {13+1,12*(RAMP_TIME1/20)},      // decoder 13 nach 12s (dmx6 - on)
      {15+1,14*(RAMP_TIME1/20)},      // decoder 15 nach 14s (dmx7 - on)
      {17+1,16*(RAMP_TIME1/20)},      // decoder 17 nach 16s (dmx8 - on)
     },    
   },
   {                                            // macro 2  -> rgb lights
     {{ 1+1, 0*(RAMP_TIME1/10)},      // decoder  1 nach  0s (dmx0 - on) 
      { 0+1, 1*(RAMP_TIME1/10)},      // decoder  0 nach  4s (dmx0 - off)
      { 3+1, 1*(RAMP_TIME1/10)},      // decoder  3 nach  4s (dmx1 - on) 
      { 2+1, 2*(RAMP_TIME1/10)},      // decoder  2 nach  8s (dmx1 - off) 
      { 5+1, 2*(RAMP_TIME1/10)},      // decoder  5 nach  8s (dmx2 - on)
      { 4+1, 3*(RAMP_TIME1/10)},      // decoder  4 nach 12s (dmx2 - off)
      { 1+1, 3*(RAMP_TIME1/10)},      // decoder  1 nach  0s (dmx0 - on) 
      { 0+1, 4*(RAMP_TIME1/10)},      // decoder  0 nach  4s (dmx0 - off)
      { 3+1, 4*(RAMP_TIME1/10)},      // decoder  3 nach  4s (dmx1 - on) 
      { 2+1, 5*(RAMP_TIME1/10)},      // decoder  2 nach  8s (dmx1 - off) 
      { 5+1, 5*(RAMP_TIME1/10)},      // decoder  5 nach  8s (dmx2 - on)
      { 4+1, 6*(RAMP_TIME1/10)},      // decoder  4 nach 12s (dmx2 - off)
      { 1+1, 6*(RAMP_TIME1/10)},      // decoder  1 nach  0s (dmx0 - on) 
      { 0+1, 7*(RAMP_TIME1/10)},      // decoder  0 nach  4s (dmx0 - off)
      { 3+1, 7*(RAMP_TIME1/10)},      // decoder  3 nach  4s (dmx1 - on) 
      { 2+1, 8*(RAMP_TIME1/10)},      // decoder  2 nach  8s (dmx1 - off) 
      
     },    
   },
   {
     {{  0,  0},      // void
     },    
   },
  },
},
//=========================================================================
// PRESET 2: three bulb setup
// dmx 0: white
// dmx 1: red
// dmx 2: blue
#define PS2_DAWNTIME  120
{
  {
    {   0,   0, (PS2_DAWNTIME*10) & 0xFF,  (PS2_DAWNTIME*10) / 256 },    //  0: dmx 0 off    in 120 sec
    {   0, 255, (PS2_DAWNTIME*10) & 0xFF,  (PS2_DAWNTIME*10) / 256 },    //  1: dmx 0 on     in 120 sec
    {   1,   0, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    //  2: dmx 1 off    in  24 sec
    {   1, 255, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    //  3: dmx 1 on     in  24 sec
    {   2,   0, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    //  4: dmx 2 off    in  24 sec
    {   2, 255, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    //  5: dmx 2 on     in  24 sec
    {   2, 130, (PS2_DAWNTIME* 4) & 0xFF,  (PS2_DAWNTIME* 4) / 256 },    //  6: dmx 2 middle in  48 sec
    {   2, 255, (PS2_DAWNTIME* 4) & 0xFF,  (PS2_DAWNTIME* 4) / 256 },    //  7: dmx 2 on     in  48 sec
    {   0, 150, (PS2_DAWNTIME*10) & 0xFF,  (PS2_DAWNTIME*10) / 256 },    //  8: dmx 0 rainy  in 120 sec
    {   0, 100, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    //  9: dmx 0 rainy  in  24 sec
    {   0, 255, (PS2_DAWNTIME* 2) & 0xFF,  (PS2_DAWNTIME* 2) / 256 },    // 10: dmx 0 on     in  24 sec
    {   0,   0,   0,  0 },                                               // 11: void
    { 100, 255,   0b00011000,  0b11100010 },                             // 12: dmx 0 lightning
    { 100, 255,   0b00000000,  0b10001111 },                             // 13: dmx 0 lightning
  },

  {
   {                                                                    // macro 0  -> enter night
     {{ 0+1,  0},      // decoder  0 nach   0s (dmx0 - day off) 
      { 3+1, 60},      // decoder  3 nach  60s (dmx1 - red evening on)
      { 2+1,120},      // decoder  2 nach 120s (dmx1 - red evening off) 
      { 5+1, 90},      // decoder  5 nach  90s (dmx2 - night on)
      { 6+1,150},      // decoder  6 nach 150s (dmx2 - dimm down night)
     },
   },
   {                                                                    // macro 1  -> enter day
     {{ 7+1,  0},      // decoder  7 nach   0s (dmx2 - night on) 
      { 4+1, 50},      // decoder  4 nach  50s (dmx2 - night off)
      { 3+1, 50},      // decoder  3 nach  50s (dmx1 - red morning on) 
      { 1+1, 74},      // decoder  1 nach  74s (dmx0 - light on)
      { 2+1, 80},      // decoder  2 nach  80s (dmx1 - red morning off)
     },    
   },
   {                                                                    // macro 2  -> enter rainy day
     {{ 7+1,  0},      // decoder  7 nach   0s (dmx2 - night on) 
      { 4+1, 50},      // decoder  4 nach  50s (dmx2 - night off)
      { 8+1, 40},      // decoder  1 nach  40s (dmx0 - light on (to 150)
     },
   },
   {                                                                    // macro 3  -> thunderstorm
     {{ 9+1,  0},      // decoder  9 nach  0s (dmx0 - dimm down) 
      {12+1, 30},      // decoder 12 nach 30s (dmx0 - lightning)
      {13+1, 42},      // decoder 13 nach 42s (dmx0 - lightning) 
      {10+1, 55},      // decoder 10 nach 55s (dmx0 - dimm up again)
     },    
   },
  },
},


/*

//=========================================================================
// PRESET 3: assign each pair of decoders to a dmx on off
// see www.opendcc.de, 8 lamp setup for mefm.de

{
  {
    {   0,   0,  400 & 0xFF,  400 / 256 },    //  0: dmx 0 off    in 40 sec
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
  },


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
   },
  },
},

*/

//=========================================================================
// PRESET x: enter your preset here
// be aware of braces - copy the empty scenario and keep them!

