#ifndef ALG_LPF_H
#define ALG_LPF_H
// 一阶低通滤波器结构体
typedef struct {
    float alpha;      // 滤波系数
    float prev_input; // 上一次的输入值
} lpf_1st_object_t;
// 二阶低通滤波器结构体
typedef struct {
    float alpha_1;      // 滤波系数1
    float alpha_2;      // 滤波系数2
    float prev_input_1; // 上一次的输入值
    float prev_input_2; // 上两次的输入值
} lpf_2nd_object_t;
void lpf_1st_init(lpf_1st_object_t *lpf_1st, float alpha);
float lpf_1st_update(lpf_1st_object_t *lpf_1st, float input);
void lpf_2nd_init(lpf_2nd_object_t *lpf_2nd, float alpha_1, float alpha_2);
float lpf_2nd_update(lpf_2nd_object_t *lpf_2nd, float input);
#endif // ALG_LPF_H