#include "alg_imu_ekf.h"
#include "math.h"
#include "string.h"
/**
 * @brief 生成单位阵
 * @param data 目标矩阵数据指针
 * @param size 矩阵阶数
 */
static void ekf_set_identity(float *data, uint16_t size) {
	uint16_t i;
	memset(data, 0, (uint32_t)size * size * sizeof(float));
	for (i = 0; i < size; i++) {
		data[(uint32_t)i * size + i] = 1.0f;
	}
}
/**
 * @brief 将角度归一化到[-pi, pi]
 * @param x 输入角度
 * @return 归一化后的角度
 */
static float ekf_wrap_pi(float x) {
	while (x > M_PI) {
		x -= 2.0f * M_PI;
	}
	while (x < -M_PI) {
		x += 2.0f * M_PI;
	}
	return x;
}
/**
 * @brief 安全的cos函数，避免分母接近0
 * @param x 输入角度
 * @return 保护后的cos值
 */
static float ekf_safe_cos(float x) {
	float c = cosf(x);
	if (c > 0.0f && c < 1e-4f) {
		c = 1e-4f;
	} else if (c < 0.0f && c > -1e-4f) {
		c = -1e-4f;
	}
	return c;
}
/**
 * @brief 过程模型与状态雅可比
 * @param ekf ekf对象指针
 * @param gyro 陀螺仪输入
 */
static void ekf_predict_model_and_jacobian(imu_ekf_object_t *ekf, const float gyro[EKF_IMU_INPUT_SIZE]) {
	float *x = ekf->x_data;
	float *x_minus = ekf->x_minus_data;
	float *f = ekf->f_data;
	const float dt = ekf->dt;
	const float phi = x[EKF_IDX_ROLL];
	const float theta = x[EKF_IDX_PITCH];
	const float sin_phi = sinf(phi);
	const float cos_phi = cosf(phi);
	const float cos_theta = ekf_safe_cos(theta);
	const float tan_theta = tanf(theta);
	const float sec_theta = 1.0f / cos_theta;
	const float sec_theta2 = sec_theta * sec_theta;
	const float p = gyro[0] - x[EKF_IDX_BGX];
	const float q = gyro[1] - x[EKF_IDX_BGY];
	const float r = gyro[2] - x[EKF_IDX_BGZ];
	const float phi_dot = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
	const float theta_dot = q * cos_phi - r * sin_phi;
	const float psi_dot = q * sin_phi * sec_theta + r * cos_phi * sec_theta;
	x_minus[EKF_IDX_ROLL] = ekf_wrap_pi(phi + dt * phi_dot);
	x_minus[EKF_IDX_PITCH] = ekf_wrap_pi(theta + dt * theta_dot);
	x_minus[EKF_IDX_YAW] = ekf_wrap_pi(x[EKF_IDX_YAW] + dt * psi_dot);
	x_minus[EKF_IDX_BGX] = x[EKF_IDX_BGX];
	x_minus[EKF_IDX_BGY] = x[EKF_IDX_BGY];
	x_minus[EKF_IDX_BGZ] = x[EKF_IDX_BGZ];
	ekf_set_identity(f, EKF_IMU_STATE_SIZE);
	f[EKF_IDX_ROLL * EKF_IMU_STATE_SIZE + EKF_IDX_ROLL] = 1.0f + dt * (q * cos_phi * tan_theta - r * sin_phi * tan_theta);
	f[EKF_IDX_ROLL * EKF_IMU_STATE_SIZE + EKF_IDX_PITCH] = dt * (q * sin_phi + r * cos_phi) * sec_theta2;
	f[EKF_IDX_ROLL * EKF_IMU_STATE_SIZE + EKF_IDX_BGX] = -dt;
	f[EKF_IDX_ROLL * EKF_IMU_STATE_SIZE + EKF_IDX_BGY] = -dt * sin_phi * tan_theta;
	f[EKF_IDX_ROLL * EKF_IMU_STATE_SIZE + EKF_IDX_BGZ] = -dt * cos_phi * tan_theta;
	f[EKF_IDX_PITCH * EKF_IMU_STATE_SIZE + EKF_IDX_ROLL] = dt * (-q * sin_phi - r * cos_phi);
	f[EKF_IDX_PITCH * EKF_IMU_STATE_SIZE + EKF_IDX_BGY] = -dt * cos_phi;
	f[EKF_IDX_PITCH * EKF_IMU_STATE_SIZE + EKF_IDX_BGZ] = dt * sin_phi;
	f[EKF_IDX_YAW * EKF_IMU_STATE_SIZE + EKF_IDX_ROLL] = dt * (q * cos_phi - r * sin_phi) * sec_theta;
	f[EKF_IDX_YAW * EKF_IMU_STATE_SIZE + EKF_IDX_PITCH] = dt * (q * sin_phi + r * cos_phi) * sec_theta * tan_theta;
	f[EKF_IDX_YAW * EKF_IMU_STATE_SIZE + EKF_IDX_BGY] = -dt * sin_phi * sec_theta;
	f[EKF_IDX_YAW * EKF_IMU_STATE_SIZE + EKF_IDX_BGZ] = -dt * cos_phi * sec_theta;
}
/**
 * @brief 观测模型与观测雅可比
 * @param ekf ekf对象指针
 */
static void ekf_measurement_model_and_jacobian(imu_ekf_object_t *ekf) {
	float *x_minus = ekf->x_minus_data;
	float *z_pred = ekf->z_pred_data;
	float *h = ekf->h_data;
	const float phi = x_minus[EKF_IDX_ROLL];
	const float theta = x_minus[EKF_IDX_PITCH];
	const float g = ekf->gravity;
	const float sin_phi = sinf(phi);
	const float cos_phi = cosf(phi);
	const float sin_theta = sinf(theta);
	const float cos_theta = cosf(theta);
	z_pred[0] = -g * sin_theta;
	z_pred[1] = g * sin_phi * cos_theta;
	z_pred[2] = g * cos_phi * cos_theta;
	memset(h, 0, EKF_IMU_MEAS_SIZE * EKF_IMU_STATE_SIZE * sizeof(float));
	h[0 * EKF_IMU_STATE_SIZE + EKF_IDX_PITCH] = -g * cos_theta;
	h[1 * EKF_IMU_STATE_SIZE + EKF_IDX_ROLL] = g * cos_phi * cos_theta;
	h[1 * EKF_IMU_STATE_SIZE + EKF_IDX_PITCH] = -g * sin_phi * sin_theta;
	h[2 * EKF_IMU_STATE_SIZE + EKF_IDX_ROLL] = -g * sin_phi * cos_theta;
	h[2 * EKF_IMU_STATE_SIZE + EKF_IDX_PITCH] = -g * cos_phi * sin_theta;
}
/**
 * @brief 初始化imu_ekf
 * @param ekf ekf对象指针
 * @param dt 采样周期
 * @param gravity 重力常数
 * @return 0表示成功，-1表示失败
 */
int8_t imu_ekf_init(imu_ekf_object_t *ekf, float dt, float gravity) {
	uint8_t i;
	if ((ekf == NULL) || (dt <= 0.0f)) {
		return -1;
	}
	memset(ekf, 0, sizeof(*ekf));
	ekf->dt = dt;
	ekf->gravity = (gravity > 0.0f) ? gravity : 9.81f;
	arm_mat_init_f32(&ekf->x, EKF_IMU_STATE_SIZE, 1U, ekf->x_data);
	arm_mat_init_f32(&ekf->x_minus, EKF_IMU_STATE_SIZE, 1U, ekf->x_minus_data);
	arm_mat_init_f32(&ekf->p, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->p_data);
	arm_mat_init_f32(&ekf->p_minus, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->p_minus_data);
	arm_mat_init_f32(&ekf->f, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->f_data);
	arm_mat_init_f32(&ekf->f_t, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->f_t_data);
	arm_mat_init_f32(&ekf->h, EKF_IMU_MEAS_SIZE, EKF_IMU_STATE_SIZE, ekf->h_data);
	arm_mat_init_f32(&ekf->h_t, EKF_IMU_STATE_SIZE, EKF_IMU_MEAS_SIZE, ekf->h_t_data);
	arm_mat_init_f32(&ekf->q, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->q_data);
	arm_mat_init_f32(&ekf->r, EKF_IMU_MEAS_SIZE, EKF_IMU_MEAS_SIZE, ekf->r_data);
	arm_mat_init_f32(&ekf->k, EKF_IMU_STATE_SIZE, EKF_IMU_MEAS_SIZE, ekf->k_data);
	arm_mat_init_f32(&ekf->s, EKF_IMU_MEAS_SIZE, EKF_IMU_MEAS_SIZE, ekf->s_data);
	arm_mat_init_f32(&ekf->i, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->i_data);
	arm_mat_init_f32(&ekf->z, EKF_IMU_MEAS_SIZE, 1U, ekf->z_data);
	arm_mat_init_f32(&ekf->z_pred, EKF_IMU_MEAS_SIZE, 1U, ekf->z_pred_data);
	arm_mat_init_f32(&ekf->y, EKF_IMU_MEAS_SIZE, 1U, ekf->y_data);
	arm_mat_init_f32(&ekf->temp_xx_1, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->temp_xx_1_data);
	arm_mat_init_f32(&ekf->temp_xx_2, EKF_IMU_STATE_SIZE, EKF_IMU_STATE_SIZE, ekf->temp_xx_2_data);
	arm_mat_init_f32(&ekf->temp_xz, EKF_IMU_STATE_SIZE, EKF_IMU_MEAS_SIZE, ekf->temp_xz_data);
	arm_mat_init_f32(&ekf->temp_zx, EKF_IMU_MEAS_SIZE, EKF_IMU_STATE_SIZE, ekf->temp_zx_data);
	arm_mat_init_f32(&ekf->temp_zz, EKF_IMU_MEAS_SIZE, EKF_IMU_MEAS_SIZE, ekf->temp_zz_data);
	arm_mat_init_f32(&ekf->temp_x1, EKF_IMU_STATE_SIZE, 1U, ekf->temp_x1_data);
	ekf_set_identity(ekf->i_data, EKF_IMU_STATE_SIZE);
	ekf_set_identity(ekf->p_data, EKF_IMU_STATE_SIZE);
	memset(ekf->q_data, 0, sizeof(ekf->q_data));
	memset(ekf->r_data, 0, sizeof(ekf->r_data));
	for (i = 0; i < EKF_IMU_STATE_SIZE; i++) {
		ekf->q_data[(uint32_t)i * EKF_IMU_STATE_SIZE + i] = (i < 3U) ? 1e-4f : 1e-6f;
	}
	for (i = 0; i < EKF_IMU_MEAS_SIZE; i++) {
		ekf->r_data[(uint32_t)i * EKF_IMU_MEAS_SIZE + i] = 0.5f;
	}
	memset(ekf->corrected_gyro_data, 0, sizeof(ekf->corrected_gyro_data));
	ekf->is_inited = 1U;
	return 0;
}
/**
 * @brief 重置EKF状态
 * @param ekf EKF对象指针
 */
void imu_ekf_reset(imu_ekf_object_t *ekf) {
	if ((ekf == NULL) || (ekf->is_inited == 0U)) {
		return;
	}
	memset(ekf->x_data, 0, sizeof(ekf->x_data));
	memset(ekf->x_minus_data, 0, sizeof(ekf->x_minus_data));
	ekf_set_identity(ekf->p_data, EKF_IMU_STATE_SIZE);
	memset(ekf->p_minus_data, 0, sizeof(ekf->p_minus_data));
	memset(ekf->corrected_gyro_data, 0, sizeof(ekf->corrected_gyro_data));
}
/**
 * @brief 设置Q/R对角线
 * @param ekf EKF对象指针
 * @param q_diag Q对角线，传NULL则不更新
 * @param r_diag R对角线，传NULL则不更新
 */
void imu_ekf_set_qr(imu_ekf_object_t *ekf, const float q_diag[EKF_IMU_STATE_SIZE], const float r_diag[EKF_IMU_MEAS_SIZE]) {
	uint8_t i;
	if ((ekf == NULL) || (ekf->is_inited == 0U)) {
		return;
	}
	if (q_diag != NULL) {
		memset(ekf->q_data, 0, sizeof(ekf->q_data));
		for (i = 0; i < EKF_IMU_STATE_SIZE; i++) {
			ekf->q_data[(uint32_t)i * EKF_IMU_STATE_SIZE + i] = q_diag[i];
		}
	}
	if (r_diag != NULL) {
		memset(ekf->r_data, 0, sizeof(ekf->r_data));
		for (i = 0; i < EKF_IMU_MEAS_SIZE; i++) {
			ekf->r_data[(uint32_t)i * EKF_IMU_MEAS_SIZE + i] = r_diag[i];
		}
	}
}
/**
 * @brief 执行EKF
 * @param ekf EKF对象指针
 * @param gyro 陀螺仪输入
 * @param accel 加速度输入
 * @return ARM_MATH_SUCCESS表示成功
 */
arm_status imu_ekf_step(imu_ekf_object_t *ekf, const float gyro[EKF_IMU_INPUT_SIZE], const float accel[EKF_IMU_MEAS_SIZE]) {
	arm_status status;
	uint8_t i;
	if ((ekf == NULL) || (gyro == NULL) || (accel == NULL) || (ekf->is_inited == 0U)) {
		return ARM_MATH_ARGUMENT_ERROR;
	}
	ekf_predict_model_and_jacobian(ekf, gyro);
	status = arm_mat_trans_f32(&ekf->f, &ekf->f_t);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->f, &ekf->p, &ekf->temp_xx_1);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->temp_xx_1, &ekf->f_t, &ekf->p_minus);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_add_f32(&ekf->p_minus, &ekf->q, &ekf->p_minus);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	ekf_measurement_model_and_jacobian(ekf);
	for (i = 0; i < EKF_IMU_MEAS_SIZE; i++) {
		ekf->z_data[i] = accel[i];
	}
	status = arm_mat_trans_f32(&ekf->h, &ekf->h_t);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->h, &ekf->p_minus, &ekf->temp_zx);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->temp_zx, &ekf->h_t, &ekf->s);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_add_f32(&ekf->s, &ekf->r, &ekf->s);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_inverse_f32(&ekf->s, &ekf->temp_zz);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->p_minus, &ekf->h_t, &ekf->temp_xz);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->temp_xz, &ekf->temp_zz, &ekf->k);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_sub_f32(&ekf->z, &ekf->z_pred, &ekf->y);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->k, &ekf->y, &ekf->temp_x1);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_add_f32(&ekf->x_minus, &ekf->temp_x1, &ekf->x);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	ekf->x_data[EKF_IDX_ROLL] = ekf_wrap_pi(ekf->x_data[EKF_IDX_ROLL]);
	ekf->x_data[EKF_IDX_PITCH] = ekf_wrap_pi(ekf->x_data[EKF_IDX_PITCH]);
	ekf->x_data[EKF_IDX_YAW] = ekf_wrap_pi(ekf->x_data[EKF_IDX_YAW]);
	status = arm_mat_mult_f32(&ekf->k, &ekf->h, &ekf->temp_xx_1);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_sub_f32(&ekf->i, &ekf->temp_xx_1, &ekf->temp_xx_2);
	if (status != ARM_MATH_SUCCESS) {
		return status;
	}
	status = arm_mat_mult_f32(&ekf->temp_xx_2, &ekf->p_minus, &ekf->p);
	if (status == ARM_MATH_SUCCESS) {
		ekf->corrected_gyro_data[0] = gyro[0] - ekf->x_data[EKF_IDX_BGX];
		ekf->corrected_gyro_data[1] = gyro[1] - ekf->x_data[EKF_IDX_BGY];
		ekf->corrected_gyro_data[2] = gyro[2] - ekf->x_data[EKF_IDX_BGZ];
	}
	return status;
}
/**
 * @brief 获取欧拉角估计
 * @param ekf EKF对象指针
 * @param roll 横滚角输出
 * @param pitch 俯仰角输出
 * @param yaw 偏航角输出
 */
void imu_ekf_get_euler(const imu_ekf_object_t *ekf, float *roll, float *pitch, float *yaw) {
	if ((ekf == NULL) || (ekf->is_inited == 0U)) {
		return;
	}
	if (roll != NULL) {
		*roll = ekf->x_data[EKF_IDX_ROLL];
	}
	if (pitch != NULL) {
		*pitch = ekf->x_data[EKF_IDX_PITCH];
	}
	if (yaw != NULL) {
		*yaw = ekf->x_data[EKF_IDX_YAW];
	}
}
/**
 * @brief 获取陀螺零偏估计
 * @param ekf EKF对象指针
 * @param bias 零偏输出
 */
void imu_ekf_get_gyro_bias(const imu_ekf_object_t *ekf, float bias[EKF_IMU_INPUT_SIZE]) {
	if ((ekf == NULL) || (bias == NULL) || (ekf->is_inited == 0U)) {
		return;
	}
	bias[0] = ekf->x_data[EKF_IDX_BGX];
	bias[1] = ekf->x_data[EKF_IDX_BGY];
	bias[2] = ekf->x_data[EKF_IDX_BGZ];
}
/**
 * @brief 获取状态估计
 * @param ekf EKF对象指针
 * @param euler 欧拉角输出
 * @param euler_rate 欧拉角速度输出
 */
void imu_ekf_get_state(const imu_ekf_object_t *ekf, euler_t *euler, euler_rate_t *euler_rate) {
	if ((ekf == NULL) || (ekf->is_inited == 0U)) {
		return;
	}
	if (euler != NULL) {
		euler->roll = ekf->x_data[EKF_IDX_ROLL];
		euler->pitch = ekf->x_data[EKF_IDX_PITCH];
		euler->yaw = ekf->x_data[EKF_IDX_YAW];
	}
	if (euler_rate != NULL) {
		const float phi = ekf->x_data[EKF_IDX_ROLL];
		const float theta = ekf->x_data[EKF_IDX_PITCH];
		const float p = ekf->corrected_gyro_data[0];
		const float q = ekf->corrected_gyro_data[1];
		const float r = ekf->corrected_gyro_data[2];
		const float sin_phi = sinf(phi);
		const float cos_phi = cosf(phi);
		const float tan_theta = tanf(theta);
		const float sec_theta = 1.0f / ekf_safe_cos(theta);
		euler_rate->roll_rate = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
		euler_rate->pitch_rate = q * cos_phi - r * sin_phi;
		euler_rate->yaw_rate = q * sin_phi * sec_theta + r * cos_phi * sec_theta;
	}
}