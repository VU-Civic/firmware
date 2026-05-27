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
uint8_t cell_transmit_audio(const opus_frame_t *restrict audio_frame, uint8_t is_final_frame);
uint8_t cell_is_busy(void);

uint8_t cell_at_cert_write_begin(const char *cmd, uint32_t cmd_len, uint32_t timeout_ms);
void cell_at_cert_write_data(const char *data, uint32_t len);
uint8_t cell_at_cert_write_end(uint32_t timeout_ms);
uint8_t cell_at_cmd(const char *cmd, uint32_t cmd_len, uint32_t timeout_ms);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __CELLULAR_HEADER_H__
