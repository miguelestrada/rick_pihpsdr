/* Copyright (C)
* 2017 - John Melton, G0ORX/N6LYT
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

static gboolean draw_led_cb (GtkWidget *widget, cairo_t *cr, gpointer data) {
  GdkRGBA *color=(GdkRGBA *)data;
//g_print("%s: %p color=%p r=%f g=%f b=%f\n",__FUNCTION__,widget,color,color->red,color->green,color->blue);
  cairo_set_source_rgb(cr, color->red, color->green, color->blue);
  cairo_paint(cr);
  return FALSE;
}

void led_set_color(GtkWidget *led) {
//g_print("%s: %p\n",__FUNCTION__,led);
  gtk_widget_queue_draw (led);
}

GtkWidget *create_led(int width,int height,GdkRGBA *color) {
  GtkWidget *led=gtk_drawing_area_new();
  gtk_widget_set_size_request(led,width,height);
  g_signal_connect (led, "draw", G_CALLBACK (draw_led_cb), (gpointer)color);

g_print("%s: %p: color=%p\n",__FUNCTION__,led,color);
  return led;
}
