/* Copyright (C)
* 2020 - John Melton, G0ORX/N6LYT
  *
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*/

// Rewrite to use gpiod rather than wiringPi
// Note that all pin numbers are now the Broadcom GPIO


#include <gtk/gtk.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>

#ifdef GPIO
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#endif

#include "band.h"
#include "channel.h"
#include "discovered.h"
#include "mode.h"
#include "filter.h"
#include "bandstack.h"
#include "toolbar.h"
#include "radio.h"
#include "toolbar.h"
#include "main.h"
#include "property.h"
#include "vfo.h"
#include "wdsp.h"
#include "new_menu.h"
#include "encoder_menu.h"
#include "diversity_menu.h"
#include "actions.h"
#include "gpio.h"
#include "i2c.h"
#include "ext.h"
#include "sliders.h"
#include "new_protocol.h"
#include "zoompan.h"
#include "iambic.h"

//
// for controllers which have spare GPIO lines,
// these lines can be associated to certain
// functions, namely
//
// CWL:      input:  left paddle for internal (iambic) keyer
// CWR:      input:  right paddle for internal (iambic) keyer
// CWKEY:    input:  key-down from external keyer
// PTT:      input:  PTT from external keyer or microphone
//
// a value <0 indicates "do not use". All inputs are active-low.
//

static int CWL_BUTTON=-1;
static int CWR_BUTTON=-1;
static int CWKEY_BUTTON=-1;
static int PTT_BUTTON=-1;

enum {
  TOP_ENCODER,
  BOTTOM_ENCODER
};

enum {
  A,
  B
};

// encoder state table
#define R_START 0x0
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

guchar encoder_state_table[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

#ifdef GPIO
char *consumer="pihpsdr";

char *gpio_device="/dev/gpiochip0";

static struct gpiod_chip *chip=NULL;
static GMutex encoder_mutex;
static GThread *monitor_thread_id;
#endif

int I2C_INTERRUPT=15;

#define MAX_LINES 32
unsigned int monitor_lines[MAX_LINES];
int lines=0;

long settle_time=50;  // ms

// VFO Encoder is always last

ENCODER encoders_no_controller[MAX_ENCODERS]={
  {FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  {FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  {FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  {FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  {FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  };

ENCODER encoders_controller1[MAX_ENCODERS]={
  {TRUE,TRUE,20,1,26,1,0,AF_GAIN,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,25,MENU_BAND,0L},
  {TRUE,TRUE,16,1,19,1,0,AGC_GAIN,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,8,MENU_BANDSTACK,0L},
  {TRUE,TRUE,4,1,21,1,0,DRIVE,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,7,MENU_MODE,0L},
  {TRUE,TRUE,18,1,17,1,0,VFO,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  {FALSE,TRUE,0,1,0,0,1,0,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  };

ENCODER encoders_controller2_v1[MAX_ENCODERS]={
  {TRUE,TRUE,20,1,26,1,0,AF_GAIN,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,22,MENU_BAND,0L},
  {TRUE,TRUE,4,1,21,1,0,AGC_GAIN,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,27,MENU_BANDSTACK,0L},
  {TRUE,TRUE,16,1,19,1,0,IF_WIDTH,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,23,MENU_MODE,0L},
  {TRUE,TRUE,25,1,8,1,0,RIT,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,TRUE,TRUE,24,MENU_FREQUENCY,0L},
  {TRUE,TRUE,18,1,17,1,0,VFO,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},
  };

ENCODER encoders_controller2_v2[MAX_ENCODERS]={
  {TRUE,TRUE,5,1,6,1,0,DRIVE,R_START,TRUE,TRUE,26,1,20,1,0,AF_GAIN,R_START,TRUE,TRUE,22,MENU_BAND,0L},          //ENC2
  {TRUE,TRUE,9,1,7,1,0,ATTENUATION,R_START,TRUE,TRUE,21,1,4,1,0,AGC_GAIN,R_START,TRUE,TRUE,27,MENU_MODE,0L},    //ENC3
  {TRUE,TRUE,11,1,10,1,0,DIV_GAIN,R_START,TRUE,TRUE,19,1,16,1,0,DIV_PHASE,R_START,TRUE,TRUE,23,DIV,0L},         //ENC4
  {TRUE,TRUE,13,1,12,1,0,XIT,R_START,TRUE,TRUE,8,1,25,1,0,RIT,R_START,TRUE,TRUE,24,MENU_FREQUENCY,0L},          //ENC5
  {TRUE,TRUE,18,1,17,1,0,VFO,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},                         //ENC1/VFO
  };

ENCODER encoders_g2_frontpanel[MAX_ENCODERS]={
  {TRUE,TRUE,5,1,6,1,0,DRIVE,R_START,TRUE,TRUE,26,1,20,1,0,MIC_GAIN,R_START,TRUE,TRUE,22,PS,0L},               //ENC1
  {TRUE,TRUE,9,1,7,1,0,AGC_GAIN,R_START,TRUE,TRUE,21,1,4,1,0,AF_GAIN,R_START,TRUE,TRUE,27,MUTE,0L},            //ENC3
  {TRUE,TRUE,11,1,10,1,0,DIV_GAIN,R_START,TRUE,TRUE,19,1,16,1,0,DIV_PHASE,R_START,TRUE,TRUE,23,DIV,0L},        //ENC7
  {TRUE,TRUE,13,1,12,1,0,XIT,R_START,TRUE,TRUE,8,1,25,1,0,RIT,R_START,TRUE,TRUE,24,MENU_FREQUENCY,0L},         //ENC5
  {TRUE,TRUE,18,1,17,1,0,VFO,R_START,FALSE,TRUE,0,0,0,0,0,0,R_START,FALSE,TRUE,0,0,0L},                        //VFO
  };

ENCODER *encoders=encoders_no_controller;

SWITCH switches_controller1[MAX_FUNCTIONS][MAX_SWITCHES]={
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,MENU_BAND,0L},
   {TRUE,TRUE,12,MENU_BANDSTACK,0L},
   {TRUE,TRUE,6,MENU_MODE,0L},
   {TRUE,TRUE,5,MENU_FILTER,0L},
   {TRUE,TRUE,24,MENU_NOISE,0L},
   {TRUE,TRUE,23,MENU_AGC,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,LOCK,0L},
   {TRUE,TRUE,12,CTUN,0L},
   {TRUE,TRUE,6,A_TO_B,0L},
   {TRUE,TRUE,5,B_TO_A,0L},
   {TRUE,TRUE,24,A_SWAP_B,0L},
   {TRUE,TRUE,23,SPLIT,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,MENU_FREQUENCY,0L},
   {TRUE,TRUE,12,MENU_MEMORY,0L},
   {TRUE,TRUE,6,RIT_ENABLE,0L},
   {TRUE,TRUE,5,RIT_PLUS,0L},
   {TRUE,TRUE,24,RIT_MINUS,0L},
   {TRUE,TRUE,23,RIT_CLEAR,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,MENU_FREQUENCY,0L},
   {TRUE,TRUE,12,MENU_MEMORY,0L},
   {TRUE,TRUE,6,XIT_ENABLE,0L},
   {TRUE,TRUE,5,XIT_PLUS,0L},
   {TRUE,TRUE,24,XIT_MINUS,0L},
   {TRUE,TRUE,23,XIT_CLEAR,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,MENU_FREQUENCY,0L},
   {TRUE,TRUE,12,SPLIT,0L},
   {TRUE,TRUE,6,DUPLEX,0L},
   {TRUE,TRUE,5,SAT,0L},
   {TRUE,TRUE,24,RSAT,0L},
   {TRUE,TRUE,23,MENU_BAND,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},
  {{TRUE,TRUE,27,MOX,0L},
   {TRUE,TRUE,13,TUNE,0L},
   {TRUE,TRUE,12,TUNE_FULL,0L},
   {TRUE,TRUE,6,TUNE_MEMORY,0L},
   {TRUE,TRUE,5,MENU_BAND,0L},
   {TRUE,TRUE,24,MENU_MODE,0L},
   {TRUE,TRUE,23,MENU_FILTER,0L},
   {TRUE,TRUE,22,FUNCTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L},
   {FALSE,FALSE,0,NO_ACTION,0L}},

  };

SWITCH switches_controller2_v1[MAX_SWITCHES]={
  {FALSE,FALSE,0,MOX,0L},
  {FALSE,FALSE,0,TUNE,0L},
  {FALSE,FALSE,0,PS,0L},
  {FALSE,FALSE,0,TWO_TONE,0L},
  {FALSE,FALSE,0,NR,0L},
  {FALSE,FALSE,0,A_TO_B,0L},
  {FALSE,FALSE,0,B_TO_A,0L},
  {FALSE,FALSE,0,MODE_MINUS,0L},
  {FALSE,FALSE,0,BAND_MINUS,0L},
  {FALSE,FALSE,0,MODE_PLUS,0L},
  {FALSE,FALSE,0,BAND_PLUS,0L},
  {FALSE,FALSE,0,XIT_ENABLE,0L},
  {FALSE,FALSE,0,NB,0L},
  {FALSE,FALSE,0,SNB,0L},
  {FALSE,FALSE,0,LOCK,0L},
  {FALSE,FALSE,0,CTUN,0L}
  };

SWITCH switches_controller2_v2[MAX_SWITCHES]={
  {FALSE,FALSE,0,MOX,0L},       //GPB7 SW2
  {FALSE,FALSE,0,TUNE,0L},      //GPB6 SW3
  {FALSE,FALSE,0,PS,0L},        //GPB5 SW4
  {FALSE,FALSE,0,TWO_TONE,0L},  //GPB4 SW5
  {FALSE,FALSE,0,NR,0L},        //GPA3 SW6
  {FALSE,FALSE,0,NB,0L},        //GPB3 SW14
  {FALSE,FALSE,0,SNB,0L},       //GPB2 SW15
  {FALSE,FALSE,0,XIT_ENABLE,0L},//GPA7 SW13
  {FALSE,FALSE,0,BAND_PLUS,0L}, //GPA6 SW12
  {FALSE,FALSE,0,MODE_PLUS,0L}, //GPA5 SW11
  {FALSE,FALSE,0,BAND_MINUS,0L},//GPA4 SW10
  {FALSE,FALSE,0,MODE_MINUS,0L},//GPA0 SW9
  {FALSE,FALSE,0,A_TO_B,0L},    //GPA2 SW7
  {FALSE,FALSE,0,B_TO_A,0L},    //GPA1 SW8
  {FALSE,FALSE,0,LOCK,0L},      //GPB1 SW16
  {FALSE,FALSE,0,CTUN,0L}       //GPB0 SW17
  };

SWITCH switches_g2_frontpanel[MAX_SWITCHES]={
  {FALSE,FALSE,0,XIT_ENABLE,0L},//GPB7 SW22
  {FALSE,FALSE,0,RIT_ENABLE,0L},//GPB6 SW21
  {FALSE,FALSE,0,FUNCTION,0L},  //GPB5 SW20
  {FALSE,FALSE,0,SPLIT,0L},     //GPB4 SW19
  {FALSE,FALSE,0,LOCK,0L},      //GPA3 SW9
  {FALSE,FALSE,0,B_TO_A,0L},    //GPB3 SW18
  {FALSE,FALSE,0,A_TO_B,0L},    //GPB2 SW17
  {FALSE,FALSE,0,MODE_MINUS,0L},//GPA7 SW13
  {FALSE,FALSE,0,BAND_PLUS,0L}, //GPA6 SW12
  {FALSE,FALSE,0,FILTER_PLUS,0L},//GPA5 SW11
  {FALSE,FALSE,0,MODE_PLUS,0L}, //GPA4 SW10
  {FALSE,FALSE,0,MOX,0L},       //GPA0 SW6
  {FALSE,FALSE,0,CTUN,0L},      //GPA2 SW8
  {FALSE,FALSE,0,TUNE,0L},      //GPA1 SW7
  {FALSE,FALSE,0,BAND_MINUS,0L},//GPB1 SW16
  {FALSE,FALSE,0,FILTER_MINUS,0L}//GPB0 SW15
  };

SWITCH *switches=switches_controller1[0];

#ifdef GPIO

static GThread *rotary_encoder_thread_id;
static uint64_t epochMilli;

static void initialiseEpoch() {
  struct timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000L) ;
}

static unsigned int millis () {
  uint64_t now ;
  struct  timespec ts ;
  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
  return (uint32_t)(now - epochMilli) ;
}

static gpointer rotary_encoder_thread(gpointer data) {
  int i;
  enum ACTION action;
  enum ACTION_MODE mode;
  gint val;

  usleep(250000);
  g_print("%s\n",__FUNCTION__);
  while(TRUE) {
    g_mutex_lock(&encoder_mutex);
    for(i=0;i<MAX_ENCODERS;i++) {
      if(encoders[i].bottom_encoder_enabled && encoders[i].bottom_encoder_pos!=0) {
        //g_print("%s: BOTTOM encoder %d pos=%d\n",__FUNCTION__,i,encoders[i].bottom_encoder_pos);
        action=encoders[i].bottom_encoder_function;
        mode=RELATIVE;
        if(action==VFO && vfo_encoder_divisor>1) {
          val=encoders[i].bottom_encoder_pos/vfo_encoder_divisor;
          encoders[i].bottom_encoder_pos=encoders[i].bottom_encoder_pos-(val*vfo_encoder_divisor);
        } else {
          val=encoders[i].bottom_encoder_pos;
          encoders[i].bottom_encoder_pos=0;
        }
        if(val!=0) schedule_action(action, mode, val);
      }
      if(encoders[i].top_encoder_enabled && encoders[i].top_encoder_pos!=0) {
        //g_print("%s: TOP encoder %d pos=%d\n",__FUNCTION__,i,encoders[i].top_encoder_pos);
        action=encoders[i].top_encoder_function;
        mode=RELATIVE;
        if(action==VFO && vfo_encoder_divisor>1) {
          val=encoders[i].top_encoder_pos/vfo_encoder_divisor;
          encoders[i].top_encoder_pos=encoders[i].top_encoder_pos-(val*vfo_encoder_divisor);
        } else {
          val=encoders[i].top_encoder_pos;
          encoders[i].top_encoder_pos=0;
        }
        if(val!=0) schedule_action(action, mode, val);
      }
    }
    g_mutex_unlock(&encoder_mutex);
    usleep(100000); // sleep for 100ms
  }
  return NULL;
}

static void process_encoder(int e,int l,int addr,int val) {
  guchar pinstate;
  //g_print("%s: encoder=%d level=%d addr=0x%02X val=%d\n",__FUNCTION__,e,l,addr,val);
  g_mutex_lock(&encoder_mutex);
  switch(l) {
    case BOTTOM_ENCODER:
      switch(addr) {
        case A:
          encoders[e].bottom_encoder_a_value=val;
          pinstate=(encoders[e].bottom_encoder_b_value<<1) | encoders[e].bottom_encoder_a_value;
          encoders[e].bottom_encoder_state=encoder_state_table[encoders[e].bottom_encoder_state&0xf][pinstate];
          //g_print("%s: state=%02X\n",__FUNCTION__,encoders[e].bottom_encoder_state);
          switch(encoders[e].bottom_encoder_state&0x30) {
            case DIR_NONE:
              break;
            case DIR_CW:
              encoders[e].bottom_encoder_pos++;
              break;
            case DIR_CCW:
              encoders[e].bottom_encoder_pos--;
              break;
            default:
              break;
          }

          //g_print("%s: %d BOTTOM pos=%d\n",__FUNCTION__,e,encoders[e].bottom_encoder_pos);
          break;
        case B:
          encoders[e].bottom_encoder_b_value=val;
          pinstate=(encoders[e].bottom_encoder_b_value<<1) | encoders[e].bottom_encoder_a_value;
          encoders[e].bottom_encoder_state=encoder_state_table[encoders[e].bottom_encoder_state&0xf][pinstate];
          //g_print("%s: state=%02X\n",__FUNCTION__,encoders[e].bottom_encoder_state);
          switch(encoders[e].bottom_encoder_state&0x30) {
            case DIR_NONE:
              break;
            case DIR_CW:
              encoders[e].bottom_encoder_pos++;
              break;
            case DIR_CCW:
              encoders[e].bottom_encoder_pos--;
              break;
            default:
              break;
          }

          //g_print("%s: %d BOTTOM pos=%d\n",__FUNCTION__,e,encoders[e].bottom_encoder_pos);

          break;
      }
      break;
    case TOP_ENCODER:
      switch(addr) {
        case A:
          encoders[e].top_encoder_a_value=val;
          pinstate=(encoders[e].top_encoder_b_value<<1) | encoders[e].top_encoder_a_value;
          encoders[e].top_encoder_state=encoder_state_table[encoders[e].top_encoder_state&0xf][pinstate];
          //g_print("%s: state=%02X\n",__FUNCTION__,encoders[e].top_encoder_state);
          switch(encoders[e].top_encoder_state&0x30) {
            case DIR_NONE:
              break;
            case DIR_CW:
              encoders[e].top_encoder_pos++;
              break;
            case DIR_CCW:
              encoders[e].top_encoder_pos--;
              break;
            default:
              break;
          }
          //g_print("%s: %d TOP pos=%d\n",__FUNCTION__,e,encoders[e].top_encoder_pos);
          break;
        case B:
          encoders[e].top_encoder_b_value=val;
          pinstate=(encoders[e].top_encoder_b_value<<1) | encoders[e].top_encoder_a_value;
          encoders[e].top_encoder_state=encoder_state_table[encoders[e].top_encoder_state&0xf][pinstate];
          //g_print("%s: state=%02X\n",__FUNCTION__,encoders[e].top_encoder_state);
          switch(encoders[e].top_encoder_state&0x30) {
            case DIR_NONE:
              break;
            case DIR_CW:
              encoders[e].top_encoder_pos++;
              break;
            case DIR_CCW:
              encoders[e].top_encoder_pos--;
              break;
            default:
              break;
          }
          //g_print("%s: %d TOP pos=%d\n",__FUNCTION__,e,encoders[e].top_encoder_pos);

          break;
      }
      break;
  }
  g_mutex_unlock(&encoder_mutex);
}

static void process_edge(int offset,int value) {
  gint i;
  unsigned int t;
  gboolean found;

  //g_print("%s: offset=%d value=%d\n",__FUNCTION__,offset,value);
  found=FALSE;

  //
  // Priority 1 (highst): check encoder
  //
  for(i=0;i<MAX_ENCODERS;i++) {
    if(encoders[i].bottom_encoder_enabled && encoders[i].bottom_encoder_address_a==offset) {
      //g_print("%s: found %d encoder %d bottom A\n",__FUNCTION__,offset,i);
      process_encoder(i,BOTTOM_ENCODER,A,value==PRESSED?1:0);
      found=TRUE;
      break;
    } else if(encoders[i].bottom_encoder_enabled && encoders[i].bottom_encoder_address_b==offset) {
      //g_print("%s: found %d encoder %d bottom B\n",__FUNCTION__,offset,i);
      process_encoder(i,BOTTOM_ENCODER,B,value==PRESSED?1:0);
      found=TRUE;
      break;
    } else if(encoders[i].top_encoder_enabled && encoders[i].top_encoder_address_a==offset) {
      //g_print("%s: found %d encoder %d top A\n",__FUNCTION__,offset,i);
      process_encoder(i,TOP_ENCODER,A,value==PRESSED?1:0);
      found=TRUE;
      break;
    } else if(encoders[i].top_encoder_enabled && encoders[i].top_encoder_address_b==offset) {
      //g_print("%s: found %d encoder %d top B\n",__FUNCTION__,offset,i);
      process_encoder(i,TOP_ENCODER,B,value==PRESSED?1:0);
      found=TRUE;
      break;
    } else if(encoders[i].switch_enabled && encoders[i].switch_address==offset) {
      //g_print("%s: found %d encoder %d switch\n",__FUNCTION__,offset,i);
      t=millis();
      //g_print("%s: found %d encoder %d switch value=%d t=%u\n",__FUNCTION__,offset,i,value,t);
      if (t<encoders[i].switch_debounce) {
        return;
      }
      encoders[i].switch_debounce=t+settle_time;
      schedule_action(encoders[i].switch_function, value, 0);
      found=TRUE;
      break;
    }
  }
  if(found) return;

  //
  // Priority 2: check "non-controller" inputs
  // take care for "external" debouncing!
  //
  if (offset == CWL_BUTTON) {
    schedule_action(CW_LEFT, value, 0);
    found=TRUE;
  }
  if (offset == CWR_BUTTON) {
    schedule_action(CW_RIGHT, value, 0);
    found=TRUE;
  }
  if (offset == CWKEY_BUTTON) {
    schedule_action(CW_KEYER_KEYDOWN, value, 0);
    found=TRUE;
  }
  if (offset == PTT_BUTTON) {
    schedule_action(CW_KEYER_PTT, value, 0);
    found=TRUE;
  }
  if (found) return;

  //
  // Priority 3: handle i2c interrupt
  //
  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2 || controller==G2_FRONTPANEL) {
    if(I2C_INTERRUPT==offset) {
      if(value==PRESSED) {
        i2c_interrupt();
      }
      found=TRUE;
    }
  }
  if(found) return;

  //
  // Priority 4: handle "normal" switches
  //
  for(i=0;i<MAX_SWITCHES;i++) {
    if(switches[i].switch_enabled && switches[i].switch_address==offset) {
      t=millis();
      //g_print("%s: found %d switch %d value=%d t=%u\n",__FUNCTION__,offset,i,value,t);
      found=TRUE;
      if(t<switches[i].switch_debounce) {
        return;
    }
//g_print("%s: switches=%p function=%d (%s)\n",__FUNCTION__,switches,switches[i].switch_function,sw_string[switches[i].switch_function]);
      switches[i].switch_debounce=t+settle_time;
      schedule_action(switches[i].switch_function, value, 0);
      break;
    }
  }
  if(found) return;

  g_print("%s: could not find %d\n",__FUNCTION__,offset);
}

static int interrupt_cb(int event_type, unsigned int line, const struct timespec *timeout, void* data) {
  //g_print("%s: event=%d line=%d\n",__FUNCTION__,event_type,line);
  switch(event_type) {
    case GPIOD_CTXLESS_EVENT_CB_TIMEOUT:
      // timeout - ignore
      //g_print("%s: Ignore timeout\n",__FUNCTION__);
      break;
    case GPIOD_CTXLESS_EVENT_CB_RISING_EDGE:
      //g_print("%s: Ignore RISING EDGE\n",__FUNCTION__);
      process_edge(line,RELEASED);
      break;
    case GPIOD_CTXLESS_EVENT_CB_FALLING_EDGE:
      //g_print("%s: Process FALLING EDGE\n",__FUNCTION__);
      process_edge(line,PRESSED);
      break;
  }
  return GPIOD_CTXLESS_EVENT_CB_RET_OK;
}
#endif

//
// If there is non-standard hardware at the GPIO lines
// the code below in the NO_CONTROLLER section must
// be adjusted such that "occupied" GPIO lines are not
// used for CW or PTT.
// For CONTROLLER1 and CONTROLLER2_V1, GPIO
// lines 9,10,11,14 are "free" and can be
// used for CW and PTT.
//
void gpio_set_defaults(int ctrlr) {
  g_print("%s: %d\n",__FUNCTION__,ctrlr);
  switch(ctrlr) {
    case NO_CONTROLLER:
      CWL_BUTTON=7;
      CWR_BUTTON=21;
      PTT_BUTTON=14;
      CWKEY_BUTTON=10;
      encoders=encoders_no_controller;
      switches=switches_controller1[0];
      break;
    case CONTROLLER1:
      CWL_BUTTON=9;
      CWR_BUTTON=11;
      PTT_BUTTON=14;
      CWKEY_BUTTON=10;
      encoders=encoders_controller1;
      switches=switches_controller1[0];
      break;
    case CONTROLLER2_V1:
      CWL_BUTTON=9;
      CWR_BUTTON=11;
      PTT_BUTTON=14;
      CWKEY_BUTTON=10;
      encoders=encoders_controller2_v1;
      switches=switches_controller2_v1;
      break;
    case CONTROLLER2_V2:
      //
      // no GPIO lines available for CW etc.
      //
      encoders=encoders_controller2_v2;
      switches=switches_controller2_v2;
      break;
    case G2_FRONTPANEL:
      //
      // no GPIO lines available for CW etc.
      //
      encoders=encoders_g2_frontpanel;
      switches=switches_g2_frontpanel;
      break;
  }
}

void gpio_restore_state() {
  char* value;
  char name[80];

  loadProperties("gpio.props");

  value=getProperty("controller");
  if(value) controller=atoi(value);
  gpio_set_defaults(controller);

  for(int i=0;i<MAX_ENCODERS;i++) {
    sprintf(name,"encoders[%d].bottom_encoder_enabled",i);
    value=getProperty(name);
    if(value) encoders[i].bottom_encoder_enabled=atoi(value);
    sprintf(name,"encoders[%d].bottom_encoder_pullup",i);
    value=getProperty(name);
    if(value) encoders[i].bottom_encoder_pullup=atoi(value);
    sprintf(name,"encoders[%d].bottom_encoder_address_a",i);
    value=getProperty(name);
    if(value) encoders[i].bottom_encoder_address_a=atoi(value);
    sprintf(name,"encoders[%d].bottom_encoder_address_b",i);
    value=getProperty(name);
    if(value) encoders[i].bottom_encoder_address_b=atoi(value);
    sprintf(name,"encoders[%d].top_encoder_enabled",i);
    value=getProperty(name);
    if(value) encoders[i].top_encoder_enabled=atoi(value);
    sprintf(name,"encoders[%d].top_encoder_pullup",i);
    value=getProperty(name);
    if(value) encoders[i].top_encoder_pullup=atoi(value);
    sprintf(name,"encoders[%d].top_encoder_address_a",i);
    value=getProperty(name);
    if(value) encoders[i].top_encoder_address_a=atoi(value);
    sprintf(name,"encoders[%d].top_encoder_address_b",i);
    value=getProperty(name);
    if(value) encoders[i].top_encoder_address_b=atoi(value);
    sprintf(name,"encoders[%d].switch_enabled",i);
    value=getProperty(name);
    if(value) encoders[i].switch_enabled=atoi(value);
    sprintf(name,"encoders[%d].switch_pullup",i);
    value=getProperty(name);
    if(value) encoders[i].switch_pullup=atoi(value);
    sprintf(name,"encoders[%d].switch_address",i);
    value=getProperty(name);
    if(value) encoders[i].switch_address=atoi(value);
  }

  for(int f=0;f<MAX_FUNCTIONS;f++) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d,%d].switch_enabled",f,i);
      value=getProperty(name);
      if(value) switches_controller1[f][i].switch_enabled=atoi(value);
      sprintf(name,"switches[%d,%d].switch_pullup",f,i);
      value=getProperty(name);
      if(value) switches_controller1[f][i].switch_pullup=atoi(value);
      sprintf(name,"switches[%d,%d].switch_address",f,i);
      value=getProperty(name);
      if(value) switches_controller1[f][i].switch_address=atoi(value);
    }
  }

  if(controller!=CONTROLLER1) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d].switch_enabled",i);
      value=getProperty(name);
      if(value) switches[i].switch_enabled=atoi(value);
      sprintf(name,"switches[%d].switch_pullup",i);
      value=getProperty(name);
      if(value) switches[i].switch_pullup=atoi(value);
      sprintf(name,"switches[%d].switch_address",i);
      value=getProperty(name);
      if(value) switches[i].switch_address=atoi(value);
    }
  }
}

void gpio_save_state() {
  char value[80];
  char name[80];

  clearProperties();
  sprintf(value,"%d",controller);
  setProperty("controller",value);

  for(int i=0;i<MAX_ENCODERS;i++) {
    sprintf(name,"encoders[%d].bottom_encoder_enabled",i);
    sprintf(value,"%d",encoders[i].bottom_encoder_enabled);
    setProperty(name,value);
    sprintf(name,"encoders[%d].bottom_encoder_pullup",i);
    sprintf(value,"%d",encoders[i].bottom_encoder_pullup);
    setProperty(name,value);
    sprintf(name,"encoders[%d].bottom_encoder_address_a",i);
    sprintf(value,"%d",encoders[i].bottom_encoder_address_a);
    setProperty(name,value);
    sprintf(name,"encoders[%d].bottom_encoder_address_b",i);
    sprintf(value,"%d",encoders[i].bottom_encoder_address_b);
    setProperty(name,value);
    sprintf(name,"encoders[%d].bottom_encoder_address_b",i);
    sprintf(value,"%d",encoders[i].bottom_encoder_address_b);
    setProperty(name,value);
    sprintf(name,"encoders[%d].top_encoder_enabled",i);
    sprintf(value,"%d",encoders[i].top_encoder_enabled);
    setProperty(name,value);
    sprintf(name,"encoders[%d].top_encoder_pullup",i);
    sprintf(value,"%d",encoders[i].top_encoder_pullup);
    setProperty(name,value);
    sprintf(name,"encoders[%d].top_encoder_address_a",i);
    sprintf(value,"%d",encoders[i].top_encoder_address_a);
    setProperty(name,value);
    sprintf(name,"encoders[%d].top_encoder_address_b",i);
    sprintf(value,"%d",encoders[i].top_encoder_address_b);
    setProperty(name,value);
    sprintf(name,"encoders[%d].top_encoder_address_b",i);
    sprintf(value,"%d",encoders[i].top_encoder_address_b);
    setProperty(name,value);
    sprintf(name,"encoders[%d].switch_enabled",i);
    sprintf(value,"%d",encoders[i].switch_enabled);
    setProperty(name,value);
    sprintf(name,"encoders[%d].switch_pullup",i);
    sprintf(value,"%d",encoders[i].switch_pullup);
    setProperty(name,value);
    sprintf(name,"encoders[%d].switch_address",i);
    sprintf(value,"%d",encoders[i].switch_address);
    setProperty(name,value);
  }

  for(int f=0;f<MAX_FUNCTIONS;f++) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d,%d].switch_enabled",f,i);
      sprintf(value,"%d",switches_controller1[f][i].switch_enabled);
      setProperty(name,value);
      sprintf(name,"switches[%d,%d].switch_pullup",f,i);
      sprintf(value,"%d",switches_controller1[f][i].switch_pullup);
      setProperty(name,value);
      sprintf(name,"switches[%d,%d].switch_address",f,i);
      sprintf(value,"%d",switches_controller1[f][i].switch_address);
      setProperty(name,value);
    }
  }

  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2 || controller==G2_FRONTPANEL) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d].switch_enabled",i);
      sprintf(value,"%d",switches[i].switch_enabled);
      setProperty(name,value);
      sprintf(name,"switches[%d].switch_pullup",i);
      sprintf(value,"%d",switches[i].switch_pullup);
      setProperty(name,value);
      sprintf(name,"switches[%d].switch_address",i);
      sprintf(value,"%d",switches[i].switch_address);
      setProperty(name,value);
    }
  }

  saveProperties("gpio.props");
}

void gpio_restore_actions() {
  char *value;
  int previous_controller=NO_CONTROLLER;

  value=getProperty("controller");
  if(value) previous_controller=atoi(value);
  gpio_set_defaults(controller);

  if(controller==previous_controller) {
  char name[80];
  if(controller!=NO_CONTROLLER) {
    for(int i=0;i<MAX_ENCODERS;i++) {
      sprintf(name,"encoders[%d].bottom_encoder_function",i);
      value=getProperty(name);
      if(value) encoders[i].bottom_encoder_function=String2Action(value);
      sprintf(name,"encoders[%d].top_encoder_function",i);
      value=getProperty(name);
      if(value) encoders[i].top_encoder_function=String2Action(value);
      sprintf(name,"encoders[%d].switch_function",i);
      value=getProperty(name);
      if(value) encoders[i].switch_function=String2Action(value);
    }
  }

  //
  // Together with the functions, store the current "layer" as well
  //
  value=getProperty("switches.function");
  if (value) function=atoi(value);
  for(int f=0;f<MAX_FUNCTIONS;f++) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d,%d].switch_function",f,i);
      value=getProperty(name);
      if(value) {
        switches_controller1[f][i].switch_function=String2Action(value);
      }
    }
  }
  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2 || controller==G2_FRONTPANEL) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d].switch_function",i);
      value=getProperty(name);
      if(value) switches[i].switch_function=String2Action(value);
    }
  }
  }
}

void gpio_save_actions() {
  char value[80];
  char name[80];

  sprintf(value,"%d",controller);
  setProperty("controller",value);

  if(controller!=NO_CONTROLLER) {
    for(int i=0;i<MAX_ENCODERS;i++) {
      sprintf(name,"encoders[%d].bottom_encoder_function",i);
      Action2String(encoders[i].bottom_encoder_function, value);
      setProperty(name,value);
      sprintf(name,"encoders[%d].top_encoder_function",i);
      Action2String(encoders[i].top_encoder_function, value);
      setProperty(name,value);
      sprintf(name,"encoders[%d].switch_function",i);
      Action2String(encoders[i].switch_function, value);
      setProperty(name,value);
    }
  }

  sprintf(value,"%d", function);
  setProperty("switches.function",value);
  for(int f=0;f<MAX_FUNCTIONS;f++) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d,%d].switch_function",f,i);
      Action2String(switches_controller1[f][i].switch_function,value);
      setProperty(name,value);
    }
  }
  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2 || controller==G2_FRONTPANEL) {
    for(int i=0;i<MAX_SWITCHES;i++) {
      sprintf(name,"switches[%d].switch_function",i);
      Action2String(switches[i].switch_function,value);
      setProperty(name,value);
    }
  }
}

#ifdef GPIO
static gpointer monitor_thread(gpointer arg) {
  struct timespec t;

  // thread to monitor gpio events
  g_print("%s: start event monitor lines=%d\n",__FUNCTION__,lines);
  g_print("%s:",__FUNCTION__);
  for(int i=0;i<lines;i++) {
    g_print(" %ud",monitor_lines[i]);
  }
  g_print("\n");
  t.tv_sec=60;
  t.tv_nsec=0;

  int ret=gpiod_ctxless_event_monitor_multiple(
                        gpio_device, GPIOD_CTXLESS_EVENT_BOTH_EDGES,
                        monitor_lines, lines, FALSE,
                        consumer, &t, NULL, interrupt_cb,NULL);
  if (ret<0) {
    g_print("%s: ctxless event monitor failed: %s\n",__FUNCTION__,g_strerror(errno));
  }

  g_print("%s: exit\n",__FUNCTION__);
  return NULL;
}

static int setup_line(struct gpiod_chip *chip, int offset, gboolean pullup) {
  int ret;
  struct gpiod_line_request_config config;

  g_print("%s: %d\n",__FUNCTION__,offset);
  struct gpiod_line *line=gpiod_chip_get_line(chip, offset);
  if (!line) {
    g_print("%s: get line %d failed: %s\n",__FUNCTION__,offset,g_strerror(errno));
    return -1;
  }

  config.consumer=consumer;
  config.request_type=GPIOD_LINE_REQUEST_DIRECTION_INPUT | GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
#ifdef OLD_GPIOD
  config.flags=pullup?GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW:0;
#else
  config.flags=pullup?GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP:GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
#endif
  ret=gpiod_line_request(line,&config,1);
  if (ret<0) {
    g_print("%s: line %d gpiod_line_request failed: %s\n",__FUNCTION__,offset,g_strerror(errno));
    return ret;
  }

  gpiod_line_release(line);

  monitor_lines[lines]=offset;
  lines++;
  return 0;
}

#if 0
//unused
static int setup_output_line(struct gpiod_chip *chip, int offset, int _initial_value) {
  int ret;
  struct gpiod_line_request_config config;

  g_print("%s: %d\n",__FUNCTION__,offset);
  struct gpiod_line *line=gpiod_chip_get_line(chip, offset);
  if (!line) {
    g_print("%s: get line %d failed: %s\n",__FUNCTION__,offset,g_strerror(errno));
    return -1;
  }

  config.consumer=consumer;
  config.request_type=GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
  ret=gpiod_line_request(line,&config,1);
  if (ret<0) {
    g_print("%s: line %d gpiod_line_request failed: %s\n",__FUNCTION__,offset,g_strerror(errno));
    return ret;
  }

  // write initial value


  gpiod_line_release(line);

  return 0;
}
#endif
#endif

int gpio_init() {

#ifdef GPIO
  int ret=0;
  initialiseEpoch();

  g_mutex_init(&encoder_mutex);

  gpio_set_defaults(controller);

  chip=NULL;

//g_print("%s: open gpio 0\n",__FUNCTION__);
  chip=gpiod_chip_open_by_number(0);
  if(chip==NULL) {
    g_print("%s: open chip failed: %s\n",__FUNCTION__,g_strerror(errno));
    ret=-1;
    goto err;
  }

  // setup encoders
  g_print("%s: setup encoders\n",__FUNCTION__);
  for(int i=0;i<MAX_ENCODERS;i++) {
    if(encoders[i].bottom_encoder_enabled) {
      if(setup_line(chip,encoders[i].bottom_encoder_address_a,encoders[i].bottom_encoder_pullup)<0) {
        continue;
      }
      if(setup_line(chip,encoders[i].bottom_encoder_address_b,encoders[i].bottom_encoder_pullup)<0) {
        continue;
      }
    }

    if(encoders[i].top_encoder_enabled) {
      if(setup_line(chip,encoders[i].top_encoder_address_a,encoders[i].top_encoder_pullup)<0) {
        continue;
      }
      if(setup_line(chip,encoders[i].top_encoder_address_b,encoders[i].top_encoder_pullup)<0) {
        continue;
      }
    }

    if(encoders[i].switch_enabled) {
      if(setup_line(chip,encoders[i].switch_address,encoders[i].switch_pullup)<0) {
        continue;
      }
    }
  }

  // setup switches
  g_print("%s: setup switches\n",__FUNCTION__);
  for(int i=0;i<MAX_SWITCHES;i++) {
    if(switches[i].switch_enabled) {
      if(setup_line(chip,switches[i].switch_address,switches[i].switch_pullup)<0) {
        continue;
      }
    }
  }

  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2 || controller==G2_FRONTPANEL) {
    i2c_init();
    g_print("%s: setup i2c interrupt %d\n",__FUNCTION__,I2C_INTERRUPT);
    if((ret=setup_line(chip,I2C_INTERRUPT,TRUE))<0) {
      goto err;
    }
  }

  int have_button=0;
  if (CWL_BUTTON >= 0) {
    if((ret=setup_line(chip,CWL_BUTTON,TRUE))<0) {
      goto err;
    }
    have_button=1;
  }
  if (CWR_BUTTON >= 0) {
    if((ret=setup_line(chip,CWR_BUTTON,TRUE))<0) {
      goto err;
    }
    have_button=1;
  }
  if (CWKEY_BUTTON >= 0) {
    if((ret=setup_line(chip,CWKEY_BUTTON,TRUE))<0) {
      goto err;
    }
    have_button=1;
  }
  if (PTT_BUTTON >= 0) {
    if((ret=setup_line(chip,PTT_BUTTON,TRUE))<0) {
      goto err;
    }
    have_button=1;
  }

  if(have_button || controller != NO_CONTROLLER) {
    monitor_thread_id = g_thread_new( "gpiod monitor", monitor_thread, NULL);
    g_print("%s: monitor_thread: id=%p\n",__FUNCTION__,rotary_encoder_thread_id);
  }
  if(controller!=NO_CONTROLLER) {
    rotary_encoder_thread_id = g_thread_new( "encoders", rotary_encoder_thread, NULL);
    g_print("%s: rotary_encoder_thread: id=%p\n",__FUNCTION__,rotary_encoder_thread_id);
  }

#endif
  return 0;

#ifdef GPIO
err:
g_print("%s: err\n",__FUNCTION__);
  if(chip!=NULL) {
    gpiod_chip_close(chip);
    chip=NULL;
  }
  return ret;
#endif
}

void gpio_close() {
#ifdef GPIO
  if(chip!=NULL) gpiod_chip_close(chip);
#endif
}
