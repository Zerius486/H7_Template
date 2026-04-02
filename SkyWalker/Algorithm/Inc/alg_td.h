#ifndef ALG_TD_H
#define ALG_TD_H
// 微分追踪器结构体
typedef struct {
    float x1; // 跟踪信号
    float x2; // 微分信号
    float r;  // 跟踪速度因子
} td_object_t;
void td_init(td_object_t *td, float r);
void td_update(td_object_t *td, float input, float dt);
#endif // ALG_TD_H