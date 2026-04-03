#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "stm32h7xx_hal.h"
// SPI缓冲区大小
#define SPI_BUFFER_SIZE 512
// SPI外设数量
#define SPI_NBR 1
// SPI接收回调函数指针
typedef void (*spi_rx_callback_t)(uint8_t *rx_buffer, uint16_t rx_length);
// SPI对象结构体
typedef struct {
    SPI_HandleTypeDef *hspi;            // SPI句柄
    uint8_t rx_buffer[SPI_BUFFER_SIZE]; // 接收缓冲区
    uint8_t tx_buffer[SPI_BUFFER_SIZE]; // 发送缓冲区
    spi_rx_callback_t callback;         // 接收回调函数指针
} spi_object_t;
extern spi_object_t spi_object[SPI_NBR];
void spi_init(SPI_HandleTypeDef *hspi, spi_rx_callback_t callback);
void spi_transmit_receive(SPI_HandleTypeDef *hspi, uint8_t *tx_data, uint8_t *rx_data,
                          uint16_t length);
#endif // BSP_SPI_H
