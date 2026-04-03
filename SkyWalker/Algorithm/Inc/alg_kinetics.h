#ifndef ALG_KINETICS_H
#define ALG_KINETICS_H
#define SQRT2 1.41421356237f
// 底盘类型枚举
typedef enum { STEER_WHEEL = 0, MECANUM_WHEEL = 1, OMNI_WHEEL = 2 } chassis_type_e;
// 底盘速度结构体
typedef struct {
    float vx;
    float vy;
    float omega;
} chassis_velocity_t;
// 底盘状态结构体
typedef struct {
    float wheel_speed[4]; // 四轮速度
    float wheel_angle[4]; // 四轮转角
} chassis_state_t;
// 底盘参数结构体
typedef struct {
    float lx; // x方向半轮距
    float ly; // y方向半轮距
} chassis_params_t;
// 底盘结构体
typedef struct {
    chassis_type_e type;         // 底盘类型
    chassis_velocity_t velocity; // 速度
    chassis_state_t state;       // 状态
    chassis_params_t params;     // 车体参数
} chassis_t;
void chassis_init(chassis_t *chassis, chassis_type_e type);
void chassis_set_velocity(chassis_t *chassis, float vx, float vy, float omega);
void chassis_set_params(chassis_t *chassis, float lx, float ly);
void chassis_update_state(chassis_t *chassis);
#endif // ALG_KINETICS_H