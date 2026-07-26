#include "usb.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"

static Usb_receive_data_t usb_receive_data = {.buffer = NULL, .len = 0U};

void usbReset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = USB_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_DP_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(USB_DP_GPIO_Port, USB_DP_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(USB_DP_GPIO_Port, USB_DP_Pin, GPIO_PIN_SET);
}

Usb_send_state_e usbDebug_buff(uint8_t *Buf, uint32_t Len)
{
    return (CDC_Transmit_FS(Buf, Len) == USBD_OK) ? USB_SEND_OK : USB_SEND_FAIL;
}

Usb_send_state_e usbDebug_float(float data)
{
    char buffer[15];
    snprintf(buffer, sizeof(buffer), "%.2f\n", data);

    return (CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer)) == USBD_OK) ? USB_SEND_OK : USB_SEND_FAIL;
}

Usb_send_state_e usbDebug_uint(uint16_t data)
{
    char buffer[15];
    snprintf(buffer, sizeof(buffer), "%u\r\n", data);

    return (CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer)) == USBD_OK) ? USB_SEND_OK : USB_SEND_FAIL;
}

void usbReceiveData(uint8_t *Buf, uint32_t *Len)
{
    if (Buf == NULL || Len == NULL)
    {
        return;
    }

    usb_receive_data.buffer = Buf;
    usb_receive_data.len = *Len;

    (void)MiniPC_UpdateChassisCmdFromBuffer(usb_receive_data.buffer, usb_receive_data.len);
}
