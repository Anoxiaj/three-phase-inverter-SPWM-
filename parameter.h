#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "main.h"

#define switch_loop 0 // 0：双闭环  1：单闭环

// 单闭环时Kp = 0.05 Ki = 5
// 双闭环时：100: 2 300 ; 150:0.1 50
#define U_Kp_parameter 2
#define U_Ki_parameter 300

#define I_Kp_parameter 0.05
#define I_Ki_parameter 5

#endif