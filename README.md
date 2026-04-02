# 项目说明
本工程是一套基于STM32H723VGT6的通用电控开发模板，利用单独的CMakeLists.txt编译了静态模板库。在编写具体任务的时候，自行开启FreeRTOS并利用模板库编写任务即可。

# 待开发功能列表
- [x] LQR控制器 alg_lqr.c/h
- [x] 陷波滤波器 alg_notch.c/h
- [x] 滑模控制器 alg_smc.c/h