#ifndef __CELLULAR_HEADER_H__
#define __CELLULAR_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void cell_power_on(void);
void cell_init(void);
void cell_update_state(void);
void cell_update_device_details(void);
void cell_transmit_alert(event_message_t *event, uint8_t *audio, uint32_t audio_len);
uint8_t cell_is_busy(void); // TODO: USE THIS IN USB ISR TO DETERMINE WHETHER DATA SHOULD BE BUFFERED

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __CELLULAR_HEADER_H__
