#include "bsp_uart.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
// UART对象实例
uart_object_t uart_object[UART_NBR] = {0};
/**
 * @brief 获取UART对象实例索引
 * @param huart UART句柄
 * @return UART对象实例索引，0xFF表示无效索引
 */
uint8_t uart_index(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        return 0;
    } else if (huart == &huart2) {
        return 1;
    } else if (huart == &huart3) {
        return 2;
    } else if (huart == &huart5) {
        return 3;
    } else if (huart == &huart7) {
        return 4;
    } else if (huart == &huart10) {
        return 5;
    } else {
        return 0xFF; // 无效索引
    }
}
/**
 * @brief 初始化UART
 * @param huart UART句柄
 * @param callback 接收回调函数指针
 */
void uart_init(UART_HandleTypeDef* huart, uart_rx_callback_t callback) {
    // 获取UART对象实例索引
    uint32_t index = uart_index(huart);
    // 初始化对应的对象实例
    if (index != 0xFF) {
        uart_object[index].huart = huart;
        uart_object[index].callback = callback;
        uart_object[index].rx_buffer_active = uart_object[index].rx_buffer_0;
        // 使能DMA接收到空闲中断
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_object[index].rx_buffer_active, UART_BUFFER_SIZE);
        // 关闭DMA传输半满中断
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}
/**
 * @brief 发送UART数据帧
 * @param huart UART句柄
 * @param tx_data 发送数据指针
 * @param tx_length 发送长度
 */
void uart_transmit(UART_HandleTypeDef* huart, uint8_t* tx_data, uint16_t tx_length) { // NOLINT
    // 获取UART对象实例索引
    uint32_t index = uart_index(huart);
    if (index != 0xFF && tx_length <= UART_BUFFER_SIZE) {
        // 复制数据到发送缓冲区
        memcpy(uart_object[index].tx_buffer, tx_data, tx_length);
        // 发送数据
        HAL_UART_Transmit_DMA(huart, uart_object[index].tx_buffer, tx_length);
    }
}
/**
 * @brief HAL库 UART接收事件回调函数
 * @param huart UART句柄
 * @param rx_length 接收长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t rx_length) {
    // 获取UART对象实例索引
    uint32_t index = uart_index(huart);
    if (index != 0xFF) {
        // 双缓冲接收
        uart_object[index].rx_buffer_ready = uart_object[index].rx_buffer_active;
        if (uart_object[index].rx_buffer_active == uart_object[index].rx_buffer_0) {
            uart_object[index].rx_buffer_active = uart_object[index].rx_buffer_1;
        } else {
            uart_object[index].rx_buffer_active = uart_object[index].rx_buffer_0;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_object[index].rx_buffer_active, UART_BUFFER_SIZE);
        // 回调函数处理
        if (uart_object[index].callback != NULL) {
            uart_object[index].callback(uart_object[index].rx_buffer_ready, rx_length);
        }
    }
}
/**
 * @brief HAL库 UART错误回调函数
 * @param huart UART句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    uint32_t index = uart_index(huart);
    if (index != 0xFF) {
        // 重启DMA接收
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_object[index].rx_buffer_active, UART_BUFFER_SIZE);
    }
}