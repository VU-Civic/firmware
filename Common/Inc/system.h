#ifndef __SYSTEM_HEADER_H__
#define __SYSTEM_HEADER_H__

#include "common.h"

void chip_reset(void);
void chip_read_config(void);
void chip_update_config(void) __attribute__((section(".RamFunc")));
void chip_save_config(uint8_t needs_reboot);
void chip_initialize_unused_pins(void);
void cpu_init(void);
void cpu_sleep(void);
void cpu_feed_watchdog(void);

#endif  // #ifndef __SYSTEM_HEADER_H__
