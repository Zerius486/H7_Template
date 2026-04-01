#ifndef ALG_CONVERSION_H
#define ALG_CONVERSION_H
#include <stdint.h>
float rpm_to_rad_s(float rpm);
float rad_s_to_rpm(float rad_s);
float encoder_to_radian(uint16_t encoder_value);
uint16_t radian_to_encoder(float radian);
float radian_to_degree(float radian);
float degree_to_radian(float degree);
float encoder_to_degree(uint16_t encoder_value);
uint16_t degree_to_encoder(float degree);
#endif // ALG_CONVERSION_H