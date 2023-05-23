/////////////////////////////////////////////////////////////
//
// Saturn project: Artix7 FPGA + Raspberry Pi4 Compute Module
// PCI Express interface from linux on Raspberry pi
// this application uses C code to emulate HPSDR protocol 1
//
// copyright Laurence Barker November 2021
//
// Contribution of interfacing to PiHPSDR from N1GP (Rick Koch)
//
// licenced under GNU GPL3
//
// saturnmain.h:
// Saturn interface to PiHPSDR
//
//////////////////////////////////////////////////////////////

#ifndef __saturnmain_h
#define __saturnmain_h

#include "saturnregisters.h"

void saturn_discovery();
void saturn_init();
void saturn_handle_speaker_audio(uint8_t *UDPInBuffer);
void saturn_handle_high_priority(bool FromNetwork, unsigned char *high_priority_buffer_to_radio);
void saturn_handle_general_packet(bool FromNetwork, uint8_t *PacketBuffer);
void saturn_handle_ddc_specific(bool FromNetwork, unsigned char *receive_specific_buffer);
void saturn_handle_duc_specific(bool FromNetwork, unsigned char *transmit_specific_buffer);
void saturn_handle_duc_iq(bool FromNetwork, uint8_t *UDPInBuffer);
void saturn_exit();

#endif
