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
#include <stdio.h>
#include <string.h>

extern void rigctl_menu(GtkWidget *parent);
extern char ser_port[];
extern char andromeda_fp_serial_port[];
extern char andromeda_fp_version[];
extern void disable_rigctl(void);
extern void disable_serial(void);

extern int serial_baud_rate;
extern int andromeda_fp_baud_rate;
extern int serial_parity;
extern int andromeda_fp_serial_parity;
extern int serial_enable;
extern int andromeda_fp_serial_enable;

extern gboolean rigctl_debug;
