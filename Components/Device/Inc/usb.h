#ifndef __USB_H__
#define __USB_H__

#include "minipc.h"

typedef enum
{
    USB_SEND_FAIL = 0,
    USB_SEND_OK,
} Usb_send_state_e;

typedef struct
{
    uint8_t *buffer;
    uint32_t len;
} Usb_receive_data_t;

void usbReset(void);
Usb_send_state_e usbDebug_buff(uint8_t *Buf, uint32_t Len);
Usb_send_state_e usbDebug_float(float data);
Usb_send_state_e usbDebug_uint(uint16_t data);
void usbReceiveData(uint8_t *Buf, uint32_t *Len);

#endif
