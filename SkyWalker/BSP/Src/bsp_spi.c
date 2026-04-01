#include "bsp_spi.h"
#include "stm32h7xx_hal.h"
#include "spi.h"
#include <stdint.h>
#include <string.h>
// SPI对象实例
spi_object_t spi_object[SPI_NBR] = {0};
/**
 * @brief 获取SPI对象实例索引
 * @param hspi SPI句柄
 * @return SPI对象实例索引，0xFF表示无效索引
 */
static uint8_t spi_index(SPI_HandleTypeDef* hspi) {
    if (hspi == &hspi2) {
        return 0;
    } else {
        return 0xFF; // 无效索引
    }
}
/**
 * @brief 初始化SPI
 * @param hspi SPI句柄
 * @param callback 接收回调函数指针
 */
void spi_init(SPI_HandleTypeDef* hspi, spi_rx_callback_t callback) {
    // 获取SPI对象实例索引
    uint32_t index = spi_index(hspi);
    // 初始化对应的对象实例
    if (index != 0xFF) {
        spi_object[index].hspi = hspi;
        spi_object[index].callback = callback;
    }
}
/**
 * @brief SPI同时发送和接收数据
 * @param hspi SPI句柄
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针 (可以为NULL，如果不需要接收)
 * @param length 数据长度
 */
void spi_transmit_receive(SPI_HandleTypeDef* hspi, uint8_t* tx_data, uint8_t* rx_data, uint16_t length) {
    // 获取SPI对象实例索引
    uint32_t index = spi_index(hspi);
    if (index != 0xFF && length <= SPI_BUFFER_SIZE) {
        // 复制数据到发送缓冲区
        memcpy(spi_object[index].tx_buffer, tx_data, length);
        // 开启传输
        HAL_SPI_TransmitReceive_DMA(hspi, spi_object[index].tx_buffer, spi_object[index].rx_buffer, length);
    }
}
/**
 * @brief HAL库 SPI传输完成回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    // 获取SPI对象实例索引
    uint32_t index = spi_index(hspi);
    if (index != 0xFF) {
        // 回调函数处理
        if (spi_object[index].callback != NULL) {
            spi_object[index].callback(spi_object[index].rx_buffer, spi_object[index].hspi->RxXferCount);
        }
    }
}
/**
 * @brief HAL库 SPI错误回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    uint32_t index = spi_index(hspi);
    if (index != 0xFF) {
        // 错误处理
        HAL_SPI_Abort(hspi);
    }
}

