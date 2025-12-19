#ifndef __GPS_HEADER_H__
#define __GPS_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void gps_init(void);
uint8_t gps_get_timepulse_fired(void);
void gps_update_packet_timestamp(uint8_t interpolate);
void gps_update_packet_llh(void);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __GPS_HEADER_H__
