#ifndef ALG_EULER_H
#define ALG_EULER_H
// 欧拉角结构体
typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_t;
// 欧拉角速度结构体
typedef struct {
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} euler_rate_t;
#include "alg_quaternion.h"
euler_t quaternion_to_euler(quaternion_t q);
#endif // ALG_EULER_H
