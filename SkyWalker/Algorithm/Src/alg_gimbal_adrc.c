#include "alg_gimbal_adrc.h"
/**
 * @brief 限幅函数
 * @param value: 输入值
 * @param min: 最小值
 * @param max: 最大值
 * @return 限幅后的值
 */
static float clamp(float value, float min, float max) {
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}
/**
 * @brief 云台ADRC初始化
 * @param adrc: 云台ADRC结构体指针
 * @param b0: 控制增益
 * @param omega0: 观测器带宽
 * @param omega_c: 控制器带宽
 * @param r: 速度因子
 * @param h: 滤波因子
 */
void gimbal_adrc_init(gimbal_adrc_t *adrc, float b0, float omega0, float omega_c, float r,
                      float h) {
    adrc->eso.theta_est = 0.0f;
    adrc->eso.disturb_est = 0.0f;
    adrc->eso.b0 = b0;
    adrc->eso.omega0 = omega0;
    adrc->eso.u_last = 0.0f;
    td_nl_init(&adrc->td, r, h);
    adrc->omega_c = omega_c;
}
/**
 * @brief 云台ADRC更新
 * @param adrc: 云台ADRC结构体指针
 * @param theta_meas: 测量角度
 * @param theta_ref: 参考角度
 * @param dt: 时间步长
 */
void gimbal_adrc_update(gimbal_adrc_t *adrc, float theta_meas, float theta_ref, float dt) {
    // TD计算
    td_nl_update(&adrc->td, theta_ref, dt);
    // ESO计算
    float e = theta_meas - adrc->eso.theta_est;
    float beta1 = 2.0f * adrc->eso.omega0;
    float beta2 = adrc->eso.omega0 * adrc->eso.omega0;
    adrc->eso.theta_est +=
        dt * (adrc->eso.disturb_est - beta1 * e + adrc->eso.b0 * adrc->eso.u_last);
    adrc->eso.disturb_est += dt * (-beta2 * e);
    // NLSEF控制律
    float v1 = adrc->td.x1;
    float u0 = adrc->omega_c * (v1 - adrc->eso.theta_est);
    float u = u0 - adrc->eso.disturb_est / adrc->eso.b0;
    // 限幅处理
    u = clamp(u, -10.0f, 10.0f);
    // 更新控制量
    adrc->eso.u_last = u;
}