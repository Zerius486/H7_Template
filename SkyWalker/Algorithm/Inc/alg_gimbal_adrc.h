#ifndef ALG_GIMBAL_ADRC_H
#define ALG_GIMBAL_ADRC_H
#include "alg_td_nl.h"
// 云台ESO结构体
typedef struct {
    float theta_est;   // 角度估计
    float disturb_est; // 扰动估计
    float b0;          // 控制增益
    float omega0;      // 观测器带宽
    float u_last;      // 上次控制量
} gimbal_eso_t;
// 云台ADRC结构体
typedef struct {
    td_nl_t td;
    gimbal_eso_t eso;
    float omega_c; // 控制器带宽
} gimbal_adrc_t;
#endif