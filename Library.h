#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "main.h"

// Definition of 16-bit and 32-bit signed/unsigned integers and decimals:
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned int Uint16;
typedef unsigned long Uint32;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;

/*角度结构体*/
typedef struct
{
    float32 theta;
    float32 cos_theta;
    float32 sin_theta;
    float32 cos_2theta;
    float32 sin_2theta;

    float32 cos_theta_p_120;
    float32 cos_theta_m_120;
    float32 sin_theta_p_120;
    float32 sin_theta_m_120;
    float32 cos_120;
    float32 sin_120;
} THETA_REGS;
#define THETA_REGS_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

extern THETA_REGS U_theta;
extern THETA_REGS I_theta;
extern THETA_REGS G_theta;

extern float32 Sample_vol_A, Sample_vol_B, Sample_vol_C;
extern float32 Sample_curr_A, Sample_curr_B, Sample_curr_C;
extern float32 Sample_Grid_A, Sample_Grid_B, Sample_Grid_C;

extern float32 Vref, Iref, Vdc;

extern void sin_cos_cal(THETA_REGS *p);
extern void THETA_REGS_VAR_INIT(THETA_REGS *p);
extern void PID_VAR_INIT(PID *p);
extern void RAMP_VAR_INIT(RAMP_REFERENCE *p);
extern void dq2abc(DQ2ABC *p, THETA_REGS *q);
extern void Clark(CLARK_REGS *p);
extern void iClark(ICLARK_REGS *p);
extern void Clark_d90A(CLARK_REGS *p);
extern void Park(PARK_REGS *p, THETA_REGS *q);
extern void iPark(IPARK_REGS *p, THETA_REGS *q);
extern void Park_d90A(PARK_REGS *p, THETA_REGS *q);
extern void Ramp_Given(RAMP_REFERENCE *v);

extern void Pid_calculation(PID *p);

#endif /* LIBRARY_H_ */