#ifndef ALG_QUATERNION_H
#define ALG_QUATERNION_H
// 四元数结构体
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;
#include "alg_euler.h"
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_normalize(quaternion_t q);
quaternion_t euler_to_quaternion(euler_t euler);
#endif // ALG_QUATERNION_H
