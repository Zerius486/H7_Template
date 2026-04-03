#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "stm32h7xx_hal.h"
// FDCAN外设数量
#define FDCAN_NBR 3
// FDCAN接收回调函数指针
typedef void (*fdcan_rx_callback_t)(uint32_t std_id, uint8_t *rx_buffer);
// FDCAN对象结构体
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;     // FDCAN句柄
    FDCAN_RxHeaderTypeDef rx_header; // 接收帧头
    FDCAN_TxHeaderTypeDef tx_header; // 发送帧头
    uint8_t rx_buffer[8];            // 接收缓冲区
    uint8_t tx_buffer[8];            // 发送缓冲区
    uint8_t fifo_active;             // 当前激活的FIFO
    fdcan_rx_callback_t callback;    // 接收回调函数指针
} fdcan_object_t;
extern fdcan_object_t fdcan_object[FDCAN_NBR];
void fdcan_init(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_callback_t callback);
void fdcan_transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_data, uint32_t tx_length,
                    uint32_t std_id);
#endif // BSP_FDCAN_H
