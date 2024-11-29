#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "main.h"

#define switch_loop 1 // 0：双闭环  1：单闭环

#define U_Kp_parameter 2
#define U_Ki_parameter 300

#define I_Kp_parameter 0.05
#define I_Ki_parameter 5

/******************离网直流侧350V时*******************/
// f = 10e3;
// Ts = 1 / f;
// Vdc = 350;
// L = 2e-3;
// C = 40e-6;
// R = 1;
// 电流环参数：Kp = 0.05 Ki = 5
// 电压环参数如下：
// 单闭环时：Kp = 0.05 Ki = 5
// 双闭环时：100: 2 300 ; 150:0.1 50

#endif