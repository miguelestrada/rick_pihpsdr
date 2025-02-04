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
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "new_menu.h"
#include "band.h"
#include "filter.h"
#include "mode.h"
#include "xvtr_menu.h"
#include "radio.h"
#include "vfo.h"

static GtkWidget *dialog=NULL;
static GtkWidget *title[BANDS+XVTRS];
static GtkWidget *min_frequency[BANDS+XVTRS];
static GtkWidget *max_frequency[BANDS+XVTRS];
static GtkWidget *lo_frequency[BANDS+XVTRS];
static GtkWidget *lo_error[BANDS+XVTRS];
static GtkWidget *disable_pa[BANDS+XVTRS];

static void save_xvtr () {
  if(dialog!=NULL) {
    const char *minf;
    const char *maxf;
    const char *lof;
    const char *loerr;
    for(int i=BANDS;i<BANDS+XVTRS;i++) {
      BAND *xvtr=band_get_band(i);
      BANDSTACK *bandstack=xvtr->bandstack;
      const char *t=gtk_entry_get_text(GTK_ENTRY(title[i]));
      strcpy(xvtr->title,t);
      if(strlen(t)!=0) {
        minf=gtk_entry_get_text(GTK_ENTRY(min_frequency[i]));
        xvtr->frequencyMin=(long long)(atof(minf)*1000000.0);
        maxf=gtk_entry_get_text(GTK_ENTRY(max_frequency[i]));
        xvtr->frequencyMax=(long long)(atof(maxf)*1000000.0);
        lof=gtk_entry_get_text(GTK_ENTRY(lo_frequency[i]));
        xvtr->frequencyLO=(long long)(atof(lof)*1000000.0);
        loerr=gtk_entry_get_text(GTK_ENTRY(lo_error[i]));
        xvtr->errorLO=atoll(loerr);
        xvtr->disablePA=gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(disable_pa[i]));
        //
        // Patch up the frequencies:
        // frequencyMax must be at most frequencyLO + max radio frequency
        // frequencyMin must be at least frequencyLO + min radio frequency
        //
        if ((xvtr->frequencyMin < xvtr->frequencyLO + radio->frequency_min) ||
            (xvtr->frequencyMin > xvtr->frequencyLO + radio->frequency_max)) {
          xvtr->frequencyMin = xvtr->frequencyLO + radio->frequency_min;
          g_print("XVTR band %s MinFrequency changed to %lld\n", t, xvtr->frequencyMin);
        }
        if (xvtr->frequencyMax < xvtr->frequencyMin) {
          xvtr->frequencyMax = xvtr->frequencyMin + 1000000LL;
          g_print("XVTR band %s MaxFrequency changed to %lld\n", t, xvtr->frequencyMax);
        }
        if (xvtr->frequencyMax > xvtr->frequencyLO + radio->frequency_max) {
          xvtr->frequencyMax = xvtr->frequencyLO + radio->frequency_max;
          g_print("XVTR band %s MaxFrequency changed to %lld\n", t, xvtr->frequencyMax);
        }
        for(int b=0;b<bandstack->entries;b++) {
          BANDSTACK_ENTRY *entry=&bandstack->entry[b];
          entry->frequency=xvtr->frequencyMin+((xvtr->frequencyMax-xvtr->frequencyMin)/2);
          entry->mode=modeUSB;
          entry->filter=filterF6;
        }
      } else {
        xvtr->frequencyMin=0;
        xvtr->frequencyMax=0;
        xvtr->frequencyLO=0;
        xvtr->errorLO=0;
        xvtr->disablePA=0;
      }
    }
    vfo_xvtr_changed();
  }
}

void pa_disable_cb(GtkWidget *widget, gpointer data) {
  int i = GPOINTER_TO_INT(data);
  BAND *xvtr=band_get_band(i);
  xvtr->disablePA=gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget));
}

static void cleanup() {
  save_xvtr();
  if(dialog!=NULL) {
    gtk_widget_destroy(dialog);
    dialog=NULL;
    sub_menu=NULL;
  }
}

static gboolean close_cb (GtkWidget *widget, GdkEventButton *event, gpointer data) {
  cleanup();
  return TRUE;
}

static gboolean delete_event(GtkWidget *widget, GdkEvent *event, gpointer user_data) {
  cleanup();
  return FALSE;
}

void xvtr_menu(GtkWidget *parent) {
  int i;
  char f[16];

  dialog=gtk_dialog_new();
  gtk_window_set_transient_for(GTK_WINDOW(dialog),GTK_WINDOW(parent));
  gtk_window_set_title(GTK_WINDOW(dialog),"piHPSDR - XVTR");
  g_signal_connect (dialog, "delete_event", G_CALLBACK (delete_event), NULL);
  set_backgnd(dialog);

  GtkWidget *content=gtk_dialog_get_content_area(GTK_DIALOG(dialog));

  GtkWidget *grid=gtk_grid_new();
  gtk_grid_set_column_spacing (GTK_GRID(grid),10);
  gtk_grid_set_row_homogeneous(GTK_GRID(grid),FALSE);
  gtk_grid_set_column_homogeneous(GTK_GRID(grid),FALSE);

  GtkWidget *close_b=gtk_button_new_with_label("Close");
  g_signal_connect (close_b, "pressed", G_CALLBACK(close_cb), NULL);
  gtk_grid_attach(GTK_GRID(grid),close_b,0,0,1,1);

  GtkWidget *label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>Title</b>");
  gtk_grid_attach(GTK_GRID(grid),label,0,1,1,1);
  label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>Min Freq(MHz)</b>");
  gtk_grid_attach(GTK_GRID(grid),label,1,1,1,1);
  label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>Max Freq(MHz)</b>");
  gtk_grid_attach(GTK_GRID(grid),label,2,1,1,1);
  label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>LO Freq(MHz)</b>");
  gtk_grid_attach(GTK_GRID(grid),label,3,1,1,1);
  label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>LO Err(MHz)</b>");
  gtk_grid_attach(GTK_GRID(grid),label,4,1,1,1);
  label=gtk_label_new(NULL);
  gtk_label_set_markup(GTK_LABEL(label), "<b>Disable PA</b>");
  gtk_grid_attach(GTK_GRID(grid),label,7,1,1,1);


  //
  // Note  no signal connect for the text fields:
  // this will lead to intermediate frequency values that are unreasonable.
  // Set all parameters only when leaving the menu, except for disablePA
  // which can be applied immediately
  //
  for(i=BANDS;i<BANDS+XVTRS;i++) {
    BAND *xvtr=band_get_band(i);

    title[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(title[i]),7);
    gtk_entry_set_text(GTK_ENTRY(title[i]),xvtr->title);
    gtk_grid_attach(GTK_GRID(grid),title[i],0,i+2,1,1);

    min_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(min_frequency[i]),7);
    sprintf(f,"%5.3f",(double)xvtr->frequencyMin/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(min_frequency[i]),f);
    gtk_grid_attach(GTK_GRID(grid),min_frequency[i],1,i+2,1,1);

    max_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(max_frequency[i]),7);
    sprintf(f,"%5.3f",(double)xvtr->frequencyMax/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(max_frequency[i]),f);
    gtk_grid_attach(GTK_GRID(grid),max_frequency[i],2,i+2,1,1);

    lo_frequency[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(lo_frequency[i]),7);
    sprintf(f,"%5.3f",(double)xvtr->frequencyLO/1000000.0);
    gtk_entry_set_text(GTK_ENTRY(lo_frequency[i]),f);
    gtk_grid_attach(GTK_GRID(grid),lo_frequency[i],3,i+2,1,1);

    lo_error[i]=gtk_entry_new();
    gtk_entry_set_width_chars(GTK_ENTRY(lo_error[i]),9);
    sprintf(f,"%lld",xvtr->errorLO);
    gtk_entry_set_text(GTK_ENTRY(lo_error[i]),f);
    gtk_grid_attach(GTK_GRID(grid),lo_error[i],4,i+2,1,1);

    disable_pa[i]=gtk_check_button_new();
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(disable_pa[i]),xvtr->disablePA);
    gtk_grid_attach(GTK_GRID(grid),disable_pa[i],7,i+2,1,1);
    g_signal_connect(disable_pa[i],"toggled",G_CALLBACK(pa_disable_cb),GINT_TO_POINTER(i));

  }

  gtk_container_add(GTK_CONTAINER(content),grid);

  sub_menu=dialog;

  gtk_widget_show_all(dialog);

}

