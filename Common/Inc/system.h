#ifndef __SYSTEM_HEADER_H__
#define __SYSTEM_HEADER_H__

#include "common.h"

void chip_reset(void);
non_volatile_data_t chip_read_non_volatile(void);
void chip_save_non_volatile(const non_volatile_data_t *nvm_data);
void chip_initialize_unused_pins(void);
void cpu_init(void);
void cpu_sleep(void);

#endif  // #ifndef __SYSTEM_HEADER_H__
