#include "alg_conversion.h"
#include "stdint.h"
#include "math.h"
/**
 * @brief 将转每分钟转换为弧度每秒
 * @param rpm 转每分钟
 * @return 弧度每秒
 */
float rpm_to_rad_s(float rpm) {
    return rpm * M_PI / 30.0f;
}
/**
 * @brief 将弧度每秒转换为转每分钟
 * @param rad_s 弧度每秒
 * @return 转每分钟
 */
float rad_s_to_rpm(float rad_s) {
    return rad_s * 30.0f / M_PI;
}
/**
 * @brief 将编码器值转换为弧度
 * @param encoder_value 编码器原始值
 * @return 弧度制角度值
 */
float encoder_to_radian(uint16_t encoder_value) {
    return (float)(encoder_value) / 8192.0f * 2.0f * M_PI;
}
/**
 * @brief 将弧度转换为编码器值
 * @param radian 弧度制角度值
 * @return 编码器值
 */
uint16_t radian_to_encoder(float radian) {
    return (uint16_t)((radian / (2.0f * M_PI)) * 8192.0f);
}
/**
 * @brief 将弧度转换为角度
 * @param radian 弧度制角度值
 * @return 角度制角度值
 */
float radian_to_degree(float radian) {
    return radian * 180.0f / M_PI;
}
/**
 * @brief 将角度转换为弧度
 * @param degree 角度制角度值
 * @return 弧度制角度值
 */
float degree_to_radian(float degree) {
    return degree * M_PI / 180.0f;
}
/**
 * @brief 将编码器值转换为角度
 * @param encoder_value 编码器原始值
 * @return 角度制角度值
 */
float encoder_to_degree(uint16_t encoder_value) {
    return (float)(encoder_value) / 8192.0f * 360.0f;
}
/**
 * @brief 将角度转换为编码器值
 * @param degree 角度制角度值
 * @return 编码器值
 */
uint16_t degree_to_encoder(float degree) {
    return (uint16_t)(degree * 8192.0f / 360.0f);
}