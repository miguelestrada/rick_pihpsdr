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

void saturn_discovery();
void start_saturn_receive_thread();
void start_saturn_micaudio_thread();
void start_saturn_high_priority_thread();
void saturn_handle_speaker_audio(uint8_t *UDPInBuffer);
void saturn_handle_high_priority(unsigned char *high_priority_buffer_to_radio);
void saturn_handle_general_packet(uint8_t *PacketBuffer);
void saturn_handle_ddc_specific(unsigned char *receive_specific_buffer);
void saturn_handle_duc_specific(unsigned char *transmit_specific_buffer);
void saturn_handle_duc_iq(uint8_t *UDPInBuffer);
void saturn_handle_high_priority(uint8_t *UDPInBuffer);
