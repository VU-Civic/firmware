#ifndef __CELLULAR_HEADER_H__
#define __CELLULAR_HEADER_H__

#include "opus_config.h"

#ifdef CORE_CM4

void cell_power_on(void);
void cell_init(void);
void cell_update_state(void);
uint8_t cell_pending_events(void);
void cell_update_device_details(void);
void cell_transmit_alert(alert_message_t *alert);
void cell_transmit_audio(const opus_frame_t *restrict audio_frame, uint8_t is_final_frame);
uint8_t cell_is_busy(void);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __CELLULAR_HEADER_H__
