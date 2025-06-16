#ifndef __USB_HEADER_H__
#define __USB_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void usb_init(void);
void usb_send(uint8_t* data, uint16_t data_length);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __USB_HEADER_H__
