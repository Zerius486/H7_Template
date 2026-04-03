#ifndef ALG_IMU_EKF_H
#define ALG_IMU_EKF_H
#include "arm_math.h"
#include "alg_euler.h"
#define EKF_IMU_STATE_SIZE 6U
#define EKF_IMU_MEAS_SIZE 3U
#define EKF_IMU_INPUT_SIZE 3U
// imu_ekf结构体
typedef struct {
    float dt;
    float gravity;
    uint8_t is_inited;
    arm_matrix_instance_f32 x;       // 当前状态
    arm_matrix_instance_f32 x_minus; // 预测状态
    arm_matrix_instance_f32 p;       // 当前协方差
    arm_matrix_instance_f32 p_minus; // 预测协方差
    arm_matrix_instance_f32 f;       // 状态雅可比 F
    arm_matrix_instance_f32 f_t;     // F 转置
    arm_matrix_instance_f32 h;       // 观测雅可比 H
    arm_matrix_instance_f32 h_t;     // H 转置
    arm_matrix_instance_f32 q;       // 过程噪声
    arm_matrix_instance_f32 r;       // 观测噪声
    arm_matrix_instance_f32 k;       // 卡尔曼增益
    arm_matrix_instance_f32 s;       // 创新协方差
    arm_matrix_instance_f32 i;       // 单位阵
    arm_matrix_instance_f32 z;       // 观测向量
    arm_matrix_instance_f32 z_pred;  // 预测观测
    arm_matrix_instance_f32 y;       // 创新向量
    arm_matrix_instance_f32 temp_xx_1;
    arm_matrix_instance_f32 temp_xx_2;
    arm_matrix_instance_f32 temp_xz;
    arm_matrix_instance_f32 temp_zx;
    arm_matrix_instance_f32 temp_zz;
    arm_matrix_instance_f32 temp_x1;
    float x_data[EKF_IMU_STATE_SIZE];
    float x_minus_data[EKF_IMU_STATE_SIZE];
    float p_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float p_minus_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float f_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float f_t_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float h_data[EKF_IMU_MEAS_SIZE * EKF_IMU_STATE_SIZE];
    float h_t_data[EKF_IMU_STATE_SIZE * EKF_IMU_MEAS_SIZE];
    float q_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float r_data[EKF_IMU_MEAS_SIZE * EKF_IMU_MEAS_SIZE];
    float k_data[EKF_IMU_STATE_SIZE * EKF_IMU_MEAS_SIZE];
    float s_data[EKF_IMU_MEAS_SIZE * EKF_IMU_MEAS_SIZE];
    float i_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float z_data[EKF_IMU_MEAS_SIZE];
    float z_pred_data[EKF_IMU_MEAS_SIZE];
    float y_data[EKF_IMU_MEAS_SIZE];
    float temp_xx_1_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float temp_xx_2_data[EKF_IMU_STATE_SIZE * EKF_IMU_STATE_SIZE];
    float temp_xz_data[EKF_IMU_STATE_SIZE * EKF_IMU_MEAS_SIZE];
    float temp_zx_data[EKF_IMU_MEAS_SIZE * EKF_IMU_STATE_SIZE];
    float temp_zz_data[EKF_IMU_MEAS_SIZE * EKF_IMU_MEAS_SIZE];
    float temp_x1_data[EKF_IMU_STATE_SIZE];
    float corrected_gyro_data[EKF_IMU_INPUT_SIZE];
} imu_ekf_object_t;
int8_t imu_ekf_init(imu_ekf_object_t *ekf, float dt, float gravity);
void imu_ekf_reset(imu_ekf_object_t *ekf);
void imu_ekf_set_qr(imu_ekf_object_t *ekf, const float q_diag[EKF_IMU_STATE_SIZE],
                    const float r_diag[EKF_IMU_MEAS_SIZE]);
arm_status imu_ekf_step(imu_ekf_object_t *ekf, const float gyro[EKF_IMU_INPUT_SIZE],
                        const float accel[EKF_IMU_MEAS_SIZE]);
void imu_ekf_get_euler(const imu_ekf_object_t *ekf, float *roll, float *pitch, float *yaw);
void imu_ekf_get_gyro_bias(const imu_ekf_object_t *ekf, float bias[EKF_IMU_INPUT_SIZE]);
void imu_ekf_get_state(const imu_ekf_object_t *ekf, euler_t *euler, euler_rate_t *euler_rate);
#endif // ALG_IMU_EKF_H