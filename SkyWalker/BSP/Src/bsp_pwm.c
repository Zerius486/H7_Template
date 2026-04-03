#include "bsp_pwm.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
// PWM对象实例
pwm_object_t pwm_object[PWM_CHANNEL_NBR] = {0};
/**
 * @brief 获取PWM对象实例索引
 * @param htim TIM句柄
 * @param channel 通道
 * @return PWM对象实例索引，0xFF表示无效索引
 */
uint8_t pwm_index(TIM_HandleTypeDef *htim, uint32_t channel) {
    if (htim == &htim3 && channel == TIM_CHANNEL_4) {
        return 0;
    } else {
        return 0xFF; // 无效索引
    }
}
/**
 * @brief PWM通道初始化
 * @param htim TIM句柄
 * @param channel 通道
 */
void pwm_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    // 获取PWM对象实例索引
    uint32_t index = pwm_index(htim, channel);
    // 初始化PWM结构体数据并启动输出
    if (index != 0xFF) {
        pwm_object[index].htim = htim;
        pwm_object[index].channel = channel;
        pwm_object[index].arr = __HAL_TIM_GET_AUTORELOAD(htim);
        // 防止TIM未初始化导致ARR寄存器值为0，设置默认ARR值
        if (pwm_object[index].arr == 0) {
            pwm_object[index].arr = 20000;
        }
        pwm_object[index].duty_ratio = 0.0f;
        HAL_TIM_PWM_Start(htim, channel);
    }
}
/**
 * @brief 设置PWM占空比
 * @param htim TIM句柄
 * @param channel 通道
 * @param duty_ratio 占空比
 */
void pwm_set_duty_ratio(TIM_HandleTypeDef *htim, uint32_t channel, float duty_ratio) {
    // 获取PWM对象实例索引
    uint32_t index = pwm_index(htim, channel);
    if (index != 0xFF) {
        // 参数检查
        if (duty_ratio < 0.0f) {
            duty_ratio = 0.0f;
        } else if (duty_ratio > 1.0f) {
            duty_ratio = 1.0f;
        }
        // 防止浮点越界，直接处理
        if (duty_ratio == 0.0f) {
            pwm_object[index].duty_ratio = 0.0f;
            pwm_object[index].ccr = 0;
            __HAL_TIM_SET_COMPARE(htim, channel, pwm_object[index].ccr);
            return;
        } else if (duty_ratio == 1.0f) {
            pwm_object[index].duty_ratio = 1.0f;
            pwm_object[index].ccr = pwm_object[index].arr;
            __HAL_TIM_SET_COMPARE(htim, channel, pwm_object[index].ccr);
            return;
        }
        // 设置占空比
        pwm_object[index].duty_ratio = duty_ratio;
        // 计算CCR寄存器值
        pwm_object[index].ccr = (uint32_t)(pwm_object[index].arr * duty_ratio);
        // 更新CCR寄存器值
        __HAL_TIM_SET_COMPARE(htim, channel, pwm_object[index].ccr);
    }
}
