#include "bsp_fdcan.h"
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include <stdint.h>
#include <string.h>
// FDCAN对象实例
fdcan_object_t fdcan_object[FDCAN_NBR] = {0};
/**
 * @brief 获取FDCAN对象实例索引
 * @param hfdcan FDCAN句柄
 * @return FDCAN对象实例索引，0xFF表示无效索引
 */
static uint8_t fdcan_index(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan == &hfdcan1) {
        return 0;
    } else if (hfdcan == &hfdcan2) {
        return 1;
    } else if (hfdcan == &hfdcan3) {
        return 2;
    } else {
        return 0xFF; // 无效索引
    }
}
/**
 * @brief 配置FDCAN过滤器，接收所有标准帧
 * @param hfdcan FDCAN句柄
 */
static void fdcan_filter_config_all(FDCAN_HandleTypeDef *hfdcan) {
    // 获取FDCAN对象实例索引
    uint32_t index = fdcan_index(hfdcan);
    if (index != 0xFF) {
        FDCAN_FilterTypeDef filter_config;
        // 配置过滤器以接收所有标准帧
        filter_config.IdType = FDCAN_STANDARD_ID; // 标准ID
        filter_config.FilterIndex = 0; // 每个FDCAN实例使用各自的0号标准过滤器
        filter_config.FilterType = FDCAN_FILTER_MASK; // 使用掩码过滤器
        if (fdcan_object[index].fifo_active == 0) {
            filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 存储在RXFIFO0
        } else if (fdcan_object[index].fifo_active == 1) {
            filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // 存储在RXFIFO1
        }
        filter_config.FilterID1 = 0x0000;
        filter_config.FilterID2 = 0x0000; // 掩码为0表示接收所有ID
        HAL_FDCAN_ConfigFilter(hfdcan, &filter_config);
    }
}
/**
 * @brief 配置全局过滤器
 * @param hfdcan FDCAN句柄
 */
static void fdcan_global_filter_config(FDCAN_HandleTypeDef *hfdcan) {
    // 全局过滤器配置为拒绝所有帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);
}
/**
 * @brief 初始化FDCAN对象实例
 * @param hfdcan FDCAN句柄
 * @param callback 接收回调函数指针
 */
void fdcan_init(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_callback_t callback) {
    // 获取FDCAN对象实例索引
    uint32_t index = fdcan_index(hfdcan);
    if (index != 0xFF) {
        // 初始化FDCAN对象实例
        fdcan_object[index].hfdcan = hfdcan;
        fdcan_object[index].tx_header.IdType = FDCAN_STANDARD_ID;     // 使用标准ID
        fdcan_object[index].tx_header.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
        fdcan_object[index].tx_header.DataLength = 8; // 数据长度为8字节，对于某些设备需要单独更改
        fdcan_object[index].tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示器
        fdcan_object[index].tx_header.BitRateSwitch = FDCAN_BRS_OFF; // 不使用位速率切换
        fdcan_object[index].tx_header.FDFormat = FDCAN_CLASSIC_CAN;  // 使用经典CAN格式
        fdcan_object[index].tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不使用传输事件FIFO
        fdcan_object[index].tx_header.MessageMarker = 0; // 消息标记初始化为0
        fdcan_object[index].callback = callback;         // 设置接收回调函数指针
        fdcan_object[index].fifo_active = 0;             // 默认使用FIFO0
        // 配置过滤器
        fdcan_filter_config_all(hfdcan);
        fdcan_global_filter_config(hfdcan);
        if (fdcan_object[index].fifo_active == 0) {
            // 激活FIFO0接收中断
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        } else if (fdcan_object[index].fifo_active == 1) {
            // 激活FIFO1接收中断
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        }
        // 启动FDCAN
        HAL_FDCAN_Start(hfdcan);
    }
}
/**
 * @brief 发送FDCAN数据帧
 * @param hfdcan FDCAN句柄
 * @param tx_Data 发送数据指针
 * @param tx_length 发送数据长度
 * @param std_id 标准ID
 */
void fdcan_transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_data, uint32_t tx_length,
                    uint32_t std_id) {
    // 获取FDCAN对象实例索引
    uint32_t index = fdcan_index(hfdcan);
    if (index != 0xFF) {
        // 设置传输帧头的标准ID
        fdcan_object[index].tx_header.Identifier = std_id;
        // 限制发送数据长度不超过8字节
        if (tx_length > 8) {
            tx_length = 8;
        }
        // 复制数据到发送缓冲区
        memcpy(fdcan_object[index].tx_buffer, tx_data, tx_length);
        // 将数据添加到传输队列
        HAL_FDCAN_AddMessageToTxFifoQ(fdcan_object[index].hfdcan, &fdcan_object[index].tx_header,
                                      fdcan_object[index].tx_buffer);
    }
}
/**
 * @brief HAL库 FDCAN FIFO0接收回调函数
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs FIFO0中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    // 获取FDCAN对象实例索引
    uint32_t index = fdcan_index(hfdcan);
    if (index != 0xFF) {
        // 接收数据
        while (HAL_FDCAN_GetRxMessage(fdcan_object[index].hfdcan, FDCAN_RX_FIFO0,
                                      &fdcan_object[index].rx_header,
                                      fdcan_object[index].rx_buffer) == HAL_OK) {
            if (fdcan_object[index].callback != NULL) {
                // 调用回调函数
                fdcan_object[index].callback(fdcan_object[index].rx_header.Identifier,
                                             fdcan_object[index].rx_buffer);
            }
        }
    }
}
/**
 * @brief HAL库 FDCAN FIFO1接收回调函数
 * @param hfdcan FDCAN句柄
 * @param RxFifo1ITs FIFO1中断标志
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    // 获取FDCAN对象实例索引
    uint32_t index = fdcan_index(hfdcan);
    if (index != 0xFF) {
        // 接收数据
        while (HAL_FDCAN_GetRxMessage(fdcan_object[index].hfdcan, FDCAN_RX_FIFO1,
                                      &fdcan_object[index].rx_header,
                                      fdcan_object[index].rx_buffer) == HAL_OK) {
            if (fdcan_object[index].callback != NULL) {
                // 调用回调函数
                fdcan_object[index].callback(fdcan_object[index].rx_header.Identifier,
                                             fdcan_object[index].rx_buffer);
            }
        }
    }
}