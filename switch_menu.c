/* Copyright (C)
* 2016 - John Melton, G0ORX/N6LYT
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
#include <glib/gprintf.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "new_menu.h"
#include "agc_menu.h"
#include "agc.h"
#include "band.h"
#include "channel.h"
#include "radio.h"
#include "receiver.h"
#include "vfo.h"
#include "button_text.h"
#include "toolbar.h"
#include "actions.h"
#include "action_dialog.h"
#include "gpio.h"
#include "i2c.h"

static GtkWidget *parent_window=NULL;

static GtkWidget *dialog=NULL;

static SWITCH *temp_switches;


static void cleanup() {
  if(dialog!=NULL) {
    gtk_widget_destroy(dialog);
    dialog=NULL;
    active_menu=NO_MENU;
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

void switch_page_cb(GtkNotebook *notebook,GtkWidget *page,guint page_num,gpointer user_data) {
  temp_switches=switches_controller1[page_num];
}

static gboolean switch_cb(GtkWidget *widget, GdkEvent *event, gpointer data) {
  int sw=GPOINTER_TO_INT(data);
  int action=action_dialog(dialog,CONTROLLER_SWITCH,temp_switches[sw].switch_function);
  gtk_button_set_label(GTK_BUTTON(widget),ActionTable[action].str);
  temp_switches[sw].switch_function=action;
  update_toolbar_labels();
  return TRUE;
}

static void response_event(GtkWidget *mydialog,gint id,gpointer user_data) {
  // mydialog is a copy of dialog
  gtk_widget_destroy(mydialog);
  dialog=NULL;
  active_menu=NO_MENU;
  sub_menu=NULL;
}

void switch_menu(GtkWidget *parent) {
  gint row;
  gint col;
  gchar label[64];
  GtkWidget *notebook;
  GtkWidget *grid;
  GtkWidget *widget;
  gint lfunction=0;


  dialog=gtk_dialog_new_with_buttons("piHPSDR - Switch Actions",GTK_WINDOW(parent),GTK_DIALOG_DESTROY_WITH_PARENT,"_OK",GTK_RESPONSE_ACCEPT,NULL);
  g_signal_connect (dialog, "response", G_CALLBACK (response_event), NULL);

  GtkWidget *content=gtk_dialog_get_content_area(GTK_DIALOG(dialog));

  lfunction=0;

  if(controller==NO_CONTROLLER || controller==CONTROLLER1) {
    notebook=gtk_notebook_new();
    set_backgnd(notebook);
  }

next_function_set:

  grid=gtk_grid_new();
  gtk_grid_set_column_homogeneous(GTK_GRID(grid),TRUE);
  gtk_grid_set_row_homogeneous(GTK_GRID(grid),FALSE);
  gtk_grid_set_column_spacing (GTK_GRID(grid),0);
  gtk_grid_set_row_spacing (GTK_GRID(grid),0);


  row=0;
  col=0;

  gint max_switches=MAX_SWITCHES;
  switch(controller) {
    case NO_CONTROLLER:
      max_switches=8;
      temp_switches=switches_controller1[lfunction];
      break;
    case CONTROLLER1:
      max_switches=8;
      temp_switches=switches_controller1[lfunction];
      break;
    case CONTROLLER2_V1:
      max_switches=16;
      temp_switches=switches_controller2_v1;
      break;
    case CONTROLLER2_V2:
      max_switches=16;
      temp_switches=switches_controller2_v2;
      break;
    case G2_FRONTPANEL:
      max_switches=16;
      temp_switches=switches_g2_frontpanel;
      break;
  }


  int original_row=row;
  int i;

  if(controller==CONTROLLER2_V1 || controller==CONTROLLER2_V2) {
    row=row+5;
    col=0;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[0].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(0));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[1].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(1));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[2].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(2));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[3].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(3));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[4].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(4));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[5].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(5));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[6].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(6));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    row=original_row;
    col=8;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[7].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(7));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row++;
    col=7;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[8].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(8));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[9].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(9));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row++;
    col=7;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[10].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(10));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[11].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(11));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row++;
    col=7;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[12].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(12));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[13].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(13));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row++;
    col=7;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[14].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(14));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[15].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(15));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    gtk_container_add(GTK_CONTAINER(content),grid);
  } else if(controller==G2_FRONTPANEL) {
    row=2;
    col=0;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[11].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(0));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[13].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(1));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    // padding
    row=1;
    col=1;
    widget=gtk_label_new(NULL);
    gtk_widget_set_name(widget,"small_button");
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row=1;
    col=2;
    widget=gtk_label_new(NULL);
    gtk_widget_set_name(widget,"small_button");
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    row=0;
    col=6;
    for(i=10; i>7; i--) {
    widget=gtk_button_new_with_label(ActionTable[temp_switches[i].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(i));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    }

    row=1;
    col=6;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[7].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(7));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[15].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(15));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[14].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(14));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    row=2;
    col=6;
    for(i=6; i>2; i--) {
    widget=gtk_button_new_with_label(ActionTable[temp_switches[i].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(i));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    if (i==5) i=4;
    col++;
    }

    row=3;
    col=6;
    for(i=2; i>-1; i--) {
    widget=gtk_button_new_with_label(ActionTable[temp_switches[i].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(i));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col++;
    }

    // padding
    row=4;
    col=6;
    widget=gtk_label_new(NULL);
    gtk_widget_set_name(widget,"small_button");
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    row=5;
    col=6;
    widget=gtk_label_new(NULL);
    gtk_widget_set_name(widget,"small_button");
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    row=6;
    col=6;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[12].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(2));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
    col+=2;
    widget=gtk_button_new_with_label(ActionTable[temp_switches[4].switch_function].str);
    gtk_widget_set_name(widget,"small_button");
    g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(3));
    gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);

    gtk_container_add(GTK_CONTAINER(content),grid);
  } else {
    for(int i=0;i<max_switches;i++) {
      if((controller==NO_CONTROLLER || controller==CONTROLLER1) && (temp_switches[i].switch_function==FUNCTION)) {
        widget=gtk_button_new_with_label(ActionTable[temp_switches[i].switch_function].str);
        // no signal for Function button
      } else {
        widget=gtk_button_new_with_label(ActionTable[temp_switches[i].switch_function].str);
        g_signal_connect(widget,"button-press-event",G_CALLBACK(switch_cb),GINT_TO_POINTER(i));
g_print("%s: %d\n",__FUNCTION__,i);
      }
      gtk_grid_attach(GTK_GRID(grid),widget,col,row,1,1);
      col++;
    }

    g_sprintf(label,"Function %d",lfunction);
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook),grid,gtk_label_new(label));
    lfunction++;
    if(lfunction<MAX_FUNCTIONS) {
      goto next_function_set;
    }
    gtk_container_add(GTK_CONTAINER(content),notebook);
    g_signal_connect (notebook, "switch-page",G_CALLBACK(switch_page_cb),NULL);
  }

  sub_menu=dialog;

  gtk_widget_show_all(dialog);
}
