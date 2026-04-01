#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>
// USB对象实例
usb_object_t usb_object;
/**
 * @brief 初始化USB
 * @param callback 接收回调函数指针
 */
void usb_init(usb_rx_callback_t callback) {
    usb_object.callback = callback;
    usb_object.is_connected = 0;
    // 清空缓冲区
    memset(usb_object.tx_buffer, 0, USB_BUFFER_SIZE);
}
/**
 * @brief 发送USB数据
 * @param tx_data 发送数据指针
 * @param tx_length 发送长度
 */
void usb_transmit(uint8_t* tx_data, uint16_t tx_length) {
    if(!usb_object.is_connected || tx_length == 0 || tx_length > USB_BUFFER_SIZE) {
        return;
    }
    // 复制数据到发送缓冲区
    memcpy(usb_object.tx_buffer, tx_data, tx_length);
    // 调用USB发送函数
    CDC_Transmit_HS(usb_object.tx_buffer, tx_length);
}
/**
 * @brief 更新USB连接状态
 * @param state USB 连接状态
 */
void usb_state_update(uint8_t state) {
    usb_object.is_connected = (state) ? 1 : 0;
}