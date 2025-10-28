#ifndef __CELLULAR_HEADER_H__
#define __CELLULAR_HEADER_H__

#include "opusenc.h"

#ifdef CORE_CM4

void cell_power_on(void);
void cell_init(void);
uint8_t cell_update_state(void);
void cell_update_device_details(void);
void cell_transmit_alert(alert_message_t *alert);
void cell_transmit_audio(const opus_frame_t *restrict audio_frame, uint8_t is_final_frame);
uint8_t cell_is_busy(void); // TODO: USE THIS IN USB ISR TO DETERMINE WHETHER DATA SHOULD BE BUFFERED

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __CELLULAR_HEADER_H__
