#include "alg_lpf.h"
/**
 * @brief 一阶低通滤波器初始化
 * @param lpf_1st 一阶低通滤波器结构体指针
 * @param alpha 滤波系数
 */
void lpf_1st_init(lpf_1st_object_t *lpf_1st, float alpha) {
    if (alpha <= 0.0f || alpha >= 1.0f) {
        return;
    }
    lpf_1st->alpha = alpha;
    lpf_1st->prev_input = 0.0f;
}
/**
 * @brief 一阶低通滤波器更新
 * @param lpf_1st 一阶低通滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 */
float lpf_1st_update(lpf_1st_object_t *lpf_1st, float input) {
    float output = lpf_1st->alpha * input + (1.0f - lpf_1st->alpha) * lpf_1st->prev_input;
    lpf_1st->prev_input = output;
    return output;
}
/**
 * @brief 二阶低通滤波器初始化
 * @param lpf_2nd 二阶低通滤波器结构体指针
 * @param alpha_1 滤波系数1
 * @param alpha_2 滤波系数2
 */
void lpf_2nd_init(lpf_2nd_object_t *lpf_2nd, float alpha_1, float alpha_2) {
    if (alpha_1 + alpha_2 >= 1.0f || alpha_1 <= 0.0f || alpha_2 <= 0.0f) {
        return;
    }
    lpf_2nd->alpha_1 = alpha_1;
    lpf_2nd->alpha_2 = alpha_2;
    lpf_2nd->prev_input_1 = 0.0f;
    lpf_2nd->prev_input_2 = 0.0f;
}
/**
 * @brief 二阶低通滤波器更新
 * @param lpf_2nd 二阶低通滤波器结构体指针
 * @param input 当前输入值
 * @return 滤波后的输出值
 */
float lpf_2nd_update(lpf_2nd_object_t *lpf_2nd, float input) {
    float output = lpf_2nd->alpha_1 * input + lpf_2nd->alpha_2 * lpf_2nd->prev_input_1 +
                   (1.0f - lpf_2nd->alpha_1 - lpf_2nd->alpha_2) * lpf_2nd->prev_input_2;
    lpf_2nd->prev_input_1 = lpf_2nd->prev_input_2;
    lpf_2nd->prev_input_2 = output;
    return output;
}