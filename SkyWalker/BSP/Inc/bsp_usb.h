#ifndef BSP_USB_H
#define BSP_USB_H
#include "stm32h7xx_hal.h"
// USB缓冲区大小
#define USB_BUFFER_SIZE 512
// USB接收回调函数指针
typedef void (*usb_rx_callback_t)(uint8_t* rx_buffer, uint16_t rx_length);
// USB对象结构体
typedef struct {
    uint8_t tx_buffer[USB_BUFFER_SIZE];   // 发送缓冲区
    usb_rx_callback_t callback;            // 接收回调函数指针
    uint8_t is_connected;                 // 连接状态
} usb_object_t;
extern usb_object_t usb_object;
void usb_init(usb_rx_callback_t callback);
void usb_transmit(uint8_t* tx_data, uint16_t tx_length);
void usb_state_update(uint8_t state);
#endif // BSP_USB_H
