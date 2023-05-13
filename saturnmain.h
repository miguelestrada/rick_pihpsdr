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
void saturn_exit();
