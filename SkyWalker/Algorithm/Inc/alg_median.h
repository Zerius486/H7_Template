#ifndef ALG_MEDIAN_H
#define ALG_MEDIAN_H
#include "stdint.h"
#define MEDIAN_WINDOW_SIZE 5
// 中值滤波器结构体
typedef struct {
    float buffer[MEDIAN_WINDOW_SIZE]; // 环形缓冲区
    uint32_t index;                   // 当前写入位置
} median_filter_object_t;
void median_filter_init(median_filter_object_t *filter, float initial_value);
float median_filter_update(median_filter_object_t *filter, float value);
#endif // ALG_MEDIAN_H