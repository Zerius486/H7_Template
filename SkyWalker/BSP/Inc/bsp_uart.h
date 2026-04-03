#ifndef BSP_UART_H
#define BSP_UART_H
#include "stm32h7xx_hal.h"
// UART缓冲区大小
#define UART_BUFFER_SIZE 512
// UART外设数量
#define UART_NBR 6
// UART接收回调函数指针
typedef void (*uart_rx_callback_t)(uint8_t *rx_buffer, uint16_t rx_length);
// UART对象结构体
typedef struct {
    UART_HandleTypeDef *huart;             // UART句柄
    uint8_t rx_buffer_0[UART_BUFFER_SIZE]; // 接收缓冲区0
    uint8_t rx_buffer_1[UART_BUFFER_SIZE]; // 接收缓冲区1
    uint8_t *rx_buffer_active;             // 当前接收缓冲区
    uint8_t *rx_buffer_ready;              // 就绪接收缓冲区
    uint8_t tx_buffer[UART_BUFFER_SIZE];   // 发送缓冲区
    uart_rx_callback_t callback;           // 接收回调函数指针
} uart_object_t;
extern uart_object_t uart_object[UART_NBR];
uint8_t uart_index(UART_HandleTypeDef *huart);
void uart_init(UART_HandleTypeDef *huart, uart_rx_callback_t callback);
void uart_transmit(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t tx_length);
#endif // BSP_UART_H
