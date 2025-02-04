/* Copyright (C)
* 2015 - John Melton, G0ORX/N6LYT
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

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "appearance.h"
#include "band.h"
#include "receiver.h"
#include "meter.h"
#include "radio.h"
#include "wdsp.h"
#include "radio.h"
#include "version.h"
#include "mode.h"
#include "vox.h"
#include "new_menu.h"
#include "vfo.h"

static GtkWidget *meter;
static cairo_surface_t *meter_surface = NULL;

static int meter_width;
static int meter_height;
static int last_meter_type=SMETER;

#define min_rxlvl -200.0
#define min_alc   -100.0
#define min_pwr      0.0

static double max_rxlvl = min_rxlvl;
static double max_alc   = min_alc;
static double max_pwr   = min_pwr;

static int max_count=0;

static gboolean
meter_configure_event_cb (GtkWidget         *widget,
            GdkEventConfigure *event,
            gpointer           data)
{
  if (meter_surface)
    cairo_surface_destroy (meter_surface);

  meter_surface = gdk_window_create_similar_surface (gtk_widget_get_window (widget),
                                       CAIRO_CONTENT_COLOR,
                                       gtk_widget_get_allocated_width (widget),
                                       gtk_widget_get_allocated_height (widget));

  /* Initialize the surface to black */
  cairo_t *cr;
  cr = cairo_create (meter_surface);
  cairo_set_source_rgba(cr, COLOUR_VFO_BACKGND);
  cairo_paint (cr);
  cairo_destroy (cr);

  return TRUE;
}

/* Redraw the screen from the surface. Note that the ::draw
 * signal receives a ready-to-be-used cairo_t that is already
 * clipped to only draw the exposed areas of the widget
 */
static gboolean
meter_draw_cb (GtkWidget *widget, cairo_t   *cr, gpointer   data) {
  cairo_set_source_surface (cr, meter_surface, 0.0, 0.0);
  cairo_paint (cr);
  return FALSE;
}

static gboolean
meter_press_event_cb (GtkWidget *widget,
               GdkEventButton *event,
               gpointer        data)
{
  start_meter();
  return TRUE;
}


GtkWidget* meter_init(int width,int height) {

g_print("meter_init: width=%d height=%d\n",width,height);
  meter_width=width;
  meter_height=height;

  meter = gtk_drawing_area_new ();
  gtk_widget_set_size_request (meter, width, height);

  /* Signals used to handle the backing surface */
  g_signal_connect (meter, "draw",
            G_CALLBACK (meter_draw_cb), NULL);
  g_signal_connect (meter,"configure-event",
            G_CALLBACK (meter_configure_event_cb), NULL);

  /* Event signals */
  g_signal_connect (meter, "button-press-event",
            G_CALLBACK (meter_press_event_cb), NULL);
  gtk_widget_set_events (meter, gtk_widget_get_events (meter)
                     | GDK_BUTTON_PRESS_MASK);

  return meter;
}


void meter_update(RECEIVER *rx,int meter_type,double value,double reverse,double exciter,double alc,double swr) {

  double rxlvl;   // only used for RX input level, clones "value"
  double pwr;     // only used for TX power, clones "value"
  char sf[32];
  double offset;
  char *units="W";
  double interval=10.0;
  cairo_t *cr = cairo_create (meter_surface);
  BAND *band=band_get_current_band();

  //
  // First, do all the work that  does not depend on whether the
  // meter is analog or digital.
  //

  if(last_meter_type!=meter_type) {
    last_meter_type=meter_type;
    //
    // reset max values
    //
    max_rxlvl = min_rxlvl;
    max_pwr   = min_pwr;
    max_alc   = min_alc;
    max_count =    0;
  }

  //
  // Only the values max_rxlvl/max_pwr/max_alc are "on display"
  // The algorithm to calculate these "sedated" values from the
  // (possibly fluctuating)  input ones is as follows:
  //
  // - if counter > CNTMAX then move max_value towards current_value by exponential averaging
  //                            with parameter EXPAV1, EXPAV2 (but never go below the minimum value)
  // - if current_value >  max_value then set max_value to current_value and reset counter
  //
  // A new max value will therefore be immediately visible, the display stays (if not surpassed) for
  // CNTMAX cycles and then the displayed value will gradually approach the new one(s).
  #define CNTMAX 5
  #define EXPAV1 0.75
  #define EXPAV2 0.25

  switch (meter_type) {
    case POWER:
      pwr=value;
      if(pwr==0.0 || band->disablePA || !pa_enabled) {
        pwr=exciter;
      }
      if(band->disablePA || !pa_enabled) {
        units="mW";
        interval=100.0;
        pwr=pwr*1000.0;
      } else {
        switch(pa_power) {
          case PA_1W:
            units="mW";
            interval=100.0;
            pwr=pwr*1000.0;
            break;
          case PA_10W:
            interval=1.0;
            break;
          case PA_30W:
            interval=3.0;
            break;
          case PA_50W:
            interval=5.0;
            break;
          case PA_100W:
            interval=10.0;
            break;
          case PA_200W:
            interval=20.0;
            break;
          case PA_500W:
            interval=50.0;
            break;
        }
      }
      if (max_count > CNTMAX) {
        max_pwr = EXPAV1*max_pwr + EXPAV2*pwr;
        max_alc = EXPAV1*max_alc + EXPAV2*alc;
        // This is perhaps not necessary ...
        if (max_pwr < min_pwr) max_pwr=min_pwr;
        // ... but alc goes to -Infinity during CW
        if (max_alc < min_alc) max_alc=min_alc;
      }
      if (pwr > max_pwr) {
        max_pwr=pwr;
        max_count=0;
      }
      if (alc > max_alc) {
        max_alc=alc;
        max_count=0;
      }
      break;
    case SMETER:
      rxlvl=value;  // all corrections now in receiver.c
      if (max_count > CNTMAX) {
        max_rxlvl=EXPAV1*max_rxlvl + EXPAV2*rxlvl;
        if (max_rxlvl < min_rxlvl) max_rxlvl=min_rxlvl;
      }
      if (rxlvl > max_rxlvl) {
        max_rxlvl=rxlvl;
        max_count=0;
      }
      break;
  }
  max_count++;

  //
  // From now on, DO NOT USE rxlvl,pwr,alc but use max_rxlvl etc.
  //

if(analog_meter) {
  cairo_set_source_rgba(cr, COLOUR_VFO_BACKGND);
  cairo_paint (cr);

  cairo_set_font_size(cr, DISPLAY_FONT_SIZE2);

  switch(meter_type) {
    case SMETER:
      {
      offset=210.0;

      int i;
      double x;
      double y;
      double angle;
      double radians;
      double cx=(double)meter_width/2.0;
      double radius=cx-20.0;

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);
      cairo_arc(cr, cx, cx, radius, 216.0*M_PI/180.0, 324.0*M_PI/180.0);
      cairo_stroke(cr);

      cairo_set_line_width(cr, PAN_LINE_EXTRA);
      cairo_set_source_rgba(cr, COLOUR_ALARM);
      cairo_arc(cr, cx, cx, radius+2, 264.0*M_PI/180.0, 324.0*M_PI/180.0);
      cairo_stroke(cr);

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);

      for(i=1;i<10;i++) {
        angle=((double)i*6.0)+offset;
        radians=angle*M_PI/180.0;

        if((i%2)==1) {
          cairo_arc(cr, cx, cx, radius+4, radians, radians);
          cairo_get_current_point(cr, &x, &y);
          cairo_arc(cr, cx, cx, radius, radians, radians);
          cairo_line_to(cr, x, y);
          cairo_stroke(cr);

          sprintf(sf,"%d",i);
          cairo_arc(cr, cx, cx, radius+5, radians, radians);
          cairo_get_current_point(cr, &x, &y);
          cairo_new_path(cr);
          x-=4.0;
          cairo_move_to(cr, x, y);
          cairo_show_text(cr, sf);
        } else {
          cairo_arc(cr, cx, cx, radius+2, radians, radians);
          cairo_get_current_point(cr, &x, &y);
          cairo_arc(cr, cx, cx, radius, radians, radians);
          cairo_line_to(cr, x, y);
          cairo_stroke(cr);
        }
        cairo_new_path(cr);
      }

      for(i=20;i<=60;i+=20) {
        angle=((double)i+54.0)+offset;
        radians=angle*M_PI/180.0;
        cairo_arc(cr, cx, cx, radius+4, radians, radians);
        cairo_get_current_point(cr, &x, &y);
        cairo_arc(cr, cx, cx, radius, radians, radians);
        cairo_line_to(cr, x, y);
        cairo_stroke(cr);

        sprintf(sf,"+%d",i);
        cairo_arc(cr, cx, cx, radius+5, radians, radians);
        cairo_get_current_point(cr, &x, &y);
        cairo_new_path(cr);
        x-=4.0;
        cairo_move_to(cr, x, y);
        cairo_show_text(cr, sf);
        cairo_new_path(cr);
      }

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);

      angle=fmax(-127.0,max_rxlvl)+127.0+offset;

      // if frequency > 30MHz then -93 is S9
      if(vfo[active_receiver->id].frequency>30000000LL) {
        angle=angle+20;
      }

      radians=angle*M_PI/180.0;
      cairo_arc(cr, cx, cx, radius+8, radians, radians);
      cairo_line_to(cr, cx, cx);
      cairo_stroke(cr);

      cairo_set_source_rgba(cr, COLOUR_METER);
      sprintf(sf,"%d dBm",(int)(max_rxlvl+0.5));
      cairo_move_to(cr, 80, meter_height-2);
      cairo_show_text(cr, sf);

      }
      break;
    case POWER:
      {
      offset=220.0;

      int i;
      double x;
      double y;
      double angle;
      double radians;
      double cx=(double)meter_width/2.0;
      double radius=cx-20.0;

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);
      cairo_arc(cr, cx, cx, radius, 220.0*M_PI/180.0, 320.0*M_PI/180.0);
      cairo_stroke(cr);

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);

      for(i=0;i<=100;i++) {
        angle=(double)i+offset;
        radians=angle*M_PI/180.0;
        if((i%10)==0) {
          cairo_arc(cr, cx, cx, radius+4, radians, radians);
          cairo_get_current_point(cr, &x, &y);
          cairo_arc(cr, cx, cx, radius, radians, radians);
          cairo_line_to(cr, x, y);
          cairo_stroke(cr);

          if((i%20)==0) {
            sprintf(sf,"%d",(i/10)*(int)interval);
            cairo_arc(cr, cx, cx, radius+5, radians, radians);
            cairo_get_current_point(cr, &x, &y);
            cairo_new_path(cr);
            x-=6.0;
            cairo_move_to(cr, x, y);
            cairo_show_text(cr, sf);
          }
        }
        cairo_new_path(cr);
      }

      cairo_set_line_width(cr, PAN_LINE_THICK);
      cairo_set_source_rgba(cr, COLOUR_METER);

      angle=(max_pwr*10.0/(double)interval)+offset;
      radians=angle*M_PI/180.0;
      cairo_arc(cr, cx, cx, radius+8, radians, radians);
      cairo_line_to(cr, cx, cx);
      cairo_stroke(cr);


      cairo_set_source_rgba(cr, COLOUR_METER);
      sprintf(sf,"%d%s",(int)(max_pwr+0.5),units);
      cairo_move_to(cr, 80, meter_height-22);
      cairo_show_text(cr, sf);

      if (swr > transmitter->swr_alarm) {
        cairo_set_source_rgba(cr, COLOUR_ALARM);  // display SWR in red color
      } else {
        cairo_set_source_rgba(cr, COLOUR_METER); // display SWR in white color
      }
      sprintf(sf,"SWR: %1.1f:1",swr);
      cairo_move_to(cr, 60, meter_height-12);
      cairo_show_text(cr, sf);

      cairo_set_source_rgba(cr, COLOUR_METER);
      sprintf(sf,"ALC: %2.1f dB",max_alc);
      cairo_move_to(cr, 60, meter_height-2);
      cairo_show_text(cr, sf);


      }
      break;
  }

  if((meter_type==POWER) || (vox_enabled)) {
    offset=((double)meter_width-100.0)/2.0;
    double peak=vox_get_peak();
    peak=peak*100.0;
    cairo_set_source_rgba(cr, COLOUR_OK);
    cairo_rectangle(cr, offset, 0.0, peak, 5.0);
    cairo_fill(cr);

    cairo_select_font_face(cr, DISPLAY_FONT,
                CAIRO_FONT_SLANT_NORMAL,
                CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, DISPLAY_FONT_SIZE1);
    cairo_set_source_rgba(cr, COLOUR_METER);
    cairo_move_to(cr, 0.0, 8.0);
    cairo_show_text(cr, "Mic Lvl");


    cairo_move_to(cr, offset, 0.0);
    cairo_line_to(cr, offset, 5.0);
    cairo_stroke(cr);
    cairo_move_to(cr, offset+50.0, 0.0);
    cairo_line_to(cr, offset+50.0, 5.0);
    cairo_stroke(cr);
    cairo_move_to(cr, offset+100.0, 0.0);
    cairo_line_to(cr, offset+100.0, 5.0);
    cairo_stroke(cr);
    cairo_move_to(cr, offset, 5.0);
    cairo_line_to(cr, offset+100.0, 5.0);
    cairo_stroke(cr);

    if(vox_enabled) {
      cairo_set_source_rgba(cr, COLOUR_ALARM);
      cairo_move_to(cr,offset+(vox_threshold*100.0),0.0);
      cairo_line_to(cr,offset+(vox_threshold*100.0),5.0);
      cairo_stroke(cr);
    }
  }

} else {
  int text_location;
  // Section for the digital meter
  // clear the meter
  cairo_set_source_rgba(cr, COLOUR_VFO_BACKGND);
  cairo_paint (cr);

  cairo_select_font_face(cr, DISPLAY_FONT,
                CAIRO_FONT_SLANT_NORMAL,
                CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, DISPLAY_FONT_SIZE2);

  cairo_set_line_width(cr, PAN_LINE_THICK);

  if(can_transmit) {
    cairo_set_source_rgba(cr, COLOUR_METER);
    cairo_move_to(cr, 5.0, 15.0);
    cairo_line_to(cr, 5.0, 5.0);
    cairo_move_to(cr, 5.0+25.0, 15.0);
    cairo_line_to(cr, 5.0+25.0, 10.0);
    cairo_move_to(cr, 5.0+50.0, 15.0);
    cairo_line_to(cr, 5.0+50.0, 5.0);
    cairo_move_to(cr, 5.0+75.0, 15.0);
    cairo_line_to(cr, 5.0+75.0, 10.0);
    cairo_move_to(cr, 5.0+100.0, 15.0);
    cairo_line_to(cr, 5.0+100.0, 5.0);
    cairo_stroke(cr);

    double peak=vox_get_peak();
    peak=peak*100.0;
    cairo_set_source_rgba(cr, COLOUR_OK);
    cairo_rectangle(cr, 5.0, 5.0, peak, 5.0);
    cairo_fill(cr);

    if(vox_enabled) {
      cairo_set_source_rgba(cr, COLOUR_ALARM);
      cairo_move_to(cr,5.0+(vox_threshold*100.0),5.0);
      cairo_line_to(cr,5.0+(vox_threshold*100.0),15.0);
      cairo_stroke(cr);
    }

    cairo_set_source_rgba(cr, COLOUR_ATTN);
    cairo_move_to(cr, 120.0, 15.0);
    cairo_show_text(cr, "Mic Level");
  }

  cairo_set_source_rgba(cr, COLOUR_METER);
  switch(meter_type) {
    case SMETER:
      // value is dBm
      text_location=10;
      offset=5.0;

      if(meter_width>=114) {
        int db=1;
        int i;
        cairo_set_line_width(cr, PAN_LINE_THICK);
        cairo_set_source_rgba(cr, COLOUR_METER);
        for(i=0;i<54;i++) {
          cairo_move_to(cr,offset+(double)(i*db),(double)meter_height-10);
          if(i%18==0) {
            cairo_line_to(cr,offset+(double)(i*db),(double)(meter_height-20));
          } else if(i%6==0) {
            cairo_line_to(cr,offset+(double)(i*db),(double)(meter_height-15));
          }
        }
        cairo_stroke(cr);

        cairo_set_font_size(cr, DISPLAY_FONT_SIZE1);
        cairo_move_to(cr, offset+(double)(18*db)-3.0, (double)meter_height-1);
        cairo_show_text(cr, "3");
        cairo_move_to(cr, offset+(double)(36*db)-3.0, (double)meter_height-1);
        cairo_show_text(cr, "6");

        cairo_set_source_rgba(cr, COLOUR_ALARM);
        cairo_move_to(cr,offset+(double)(54*db),(double)meter_height-10);
        cairo_line_to(cr,offset+(double)(54*db),(double)(meter_height-20));
        cairo_move_to(cr,offset+(double)(74*db),(double)meter_height-10);
        cairo_line_to(cr,offset+(double)(74*db),(double)(meter_height-20));
        cairo_move_to(cr,offset+(double)(94*db),(double)meter_height-10);
        cairo_line_to(cr,offset+(double)(94*db),(double)(meter_height-20));
        cairo_move_to(cr,offset+(double)(114*db),(double)meter_height-10);
        cairo_line_to(cr,offset+(double)(114*db),(double)(meter_height-20));
        cairo_stroke(cr);

        cairo_move_to(cr, offset+(double)(54*db)-3.0, (double)meter_height-1);
        cairo_show_text(cr, "9");
        cairo_move_to(cr, offset+(double)(74*db)-12.0, (double)meter_height-1);
        cairo_show_text(cr, "+20");
        cairo_move_to(cr, offset+(double)(94*db)-9.0, (double)meter_height-1);
        cairo_show_text(cr, "+40");
        cairo_move_to(cr, offset+(double)(114*db)-6.0, (double)meter_height-1);
        cairo_show_text(cr, "+60");

        // if frequency > 30MHz then -93 is S9
        double l=fmax(-127.0,max_rxlvl);
        if(vfo[active_receiver->id].frequency>30000000LL) {
          l=max_rxlvl+20.0;
        }

        // use colours from the "gradient" panadapter display,
        // but use no gradient: S0-S9 first colour, <S9 last colour
        cairo_pattern_t *pat=cairo_pattern_create_linear(0.0,0.0,114.0,0.0);
        cairo_pattern_add_color_stop_rgba(pat,0.00,COLOUR_GRAD1);
        cairo_pattern_add_color_stop_rgba(pat,0.50,COLOUR_GRAD1);
        cairo_pattern_add_color_stop_rgba(pat,0.50,COLOUR_GRAD4);
        cairo_pattern_add_color_stop_rgba(pat,1.00,COLOUR_GRAD4);
        cairo_set_source(cr, pat);

        cairo_rectangle(cr, offset+0.0, (double)(meter_height-40), (double)((l+127.0)*db), 20.0);
        cairo_fill(cr);
        cairo_pattern_destroy(pat);

        if(l!=0) {
          //
          // Mark right edge of S-meter bar with a line in ATTN colour
          //
          cairo_set_source_rgba(cr, COLOUR_ATTN);
          cairo_move_to(cr,offset+(double)((max_rxlvl+127.0)*db),(double)meter_height-20);
          cairo_line_to(cr,offset+(double)((max_rxlvl+127.0)*db),(double)(meter_height-40));
        }


        cairo_stroke(cr);

        text_location=offset+(db*114)+10;
      }

      cairo_set_font_size(cr, DISPLAY_FONT_SIZE2);
      sprintf(sf,"%d dBm",(int)(max_rxlvl+0.5));
      cairo_move_to(cr, text_location, meter_height-12);
      cairo_show_text(cr, sf);
      break;
    case POWER:
      cairo_select_font_face(cr, DISPLAY_FONT,
            CAIRO_FONT_SLANT_NORMAL,
            CAIRO_FONT_WEIGHT_BOLD);
      cairo_set_font_size(cr, DISPLAY_FONT_SIZE2);

      if (protocol == ORIGINAL_PROTOCOL || protocol == NEW_PROTOCOL) {
        //
        // Power/Alc/SWR not available for SOAPY.
        //
        sprintf(sf,"FWD: %d%s",(int)(max_pwr+0.5),units);
        cairo_move_to(cr, 10, 35);
        cairo_show_text(cr, sf);

        if (swr > transmitter->swr_alarm) {
          cairo_set_source_rgba(cr, COLOUR_ALARM);  // display SWR in red color
        } else {
          cairo_set_source_rgba(cr, COLOUR_METER); // display SWR in white color
        }


        cairo_select_font_face(cr, DISPLAY_FONT,
            CAIRO_FONT_SLANT_NORMAL,
            CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, DISPLAY_FONT_SIZE2);
        sprintf(sf,"SWR: %1.1f:1",swr);
        cairo_move_to(cr, 10, 55);
        cairo_show_text(cr, sf);
      }

      cairo_set_source_rgba(cr, COLOUR_METER);  // revert to white color

      sprintf(sf,"ALC: %2.1f dB",max_alc);
      cairo_move_to(cr, meter_width/2, 35);
      cairo_show_text(cr, sf);
      break;
  }
}

  //
  // This is the same for analog and digital metering
  //
  cairo_destroy(cr);
  gtk_widget_queue_draw (meter);
}
