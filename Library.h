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

extern float32 Sample_vol_A, Sample_vol_B, Sample_vol_C;
extern float32 Sample_curr_A, Sample_curr_B, Sample_curr_C;

extern float32 Vref, Iref, Vdc;

extern float32 ramp_Ud_delta, ramp_Ud_length, ramp_Ud_count, ramp_Ud_output;
extern float32 ramp_Uq_delta, ramp_Uq_length, ramp_Uq_count, ramp_Uq_output;
extern float32 ramp_Id_delta, ramp_Id_length, ramp_Id_count, ramp_Id_output;
extern float32 ramp_Iq_delta, ramp_Iq_length, ramp_Iq_count, ramp_Iq_output;

extern void Var_allocation(double out[6], double in[12]);
extern void sin_cos_cal(THETA_REGS *p);
extern void THETA_REGS_VAR_INIT(THETA_REGS *p);
extern void dq2abc(DQ2ABC *p, THETA_REGS *q);
extern void Clark(CLARK_REGS *p);
extern void iClark(ICLARK_REGS *p);
extern void Park(PARK_REGS *p, THETA_REGS *q);
extern void iPark(IPARK_REGS *p, THETA_REGS *q);
extern void Ramp_Ud_Given(float32 ramp_Ud_given);
extern void Ramp_Uq_Given(float32 ramp_Uq_given);

extern void Pid_calculation(PID *p);

#endif /* LIBRARY_H_ */