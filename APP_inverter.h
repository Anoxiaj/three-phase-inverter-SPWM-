#ifndef APP_INVERTER_H_
#define APP_INVERTER_H_

#include "main.h"

// DQ->ABC
typedef struct
{
    float d;
    float q;
    float a;
    float b;
    float c;
} DQ2ABC;
#define DQ2ABC_DEFAULTS {0, 0, 0, 0, 0}

// park变换
typedef struct
{
    float alpha;
    float beta;
    float d;
    float q;
} PARK_REGS;
#define PARK_REGS_DEFAULTS {0, 0, 0, 0}

// park反变换
typedef struct
{
    float alpha;
    float beta;
    float d;
    float q;
} IPARK_REGS;
#define IPARK_REGS_DEFAULTS {0, 0, 0, 0}

// clark变换
typedef struct
{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
} CLARK_REGS;
#define CLARK_REGS_DEFAULTS {0, 0, 0, 0, 0}

// clark反变换
typedef struct
{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
} ICLARK_REGS;
#define ICLARK_REGS_DEFAULTS {0, 0, 0, 0, 0}

/*斜坡给定*/
typedef struct
{
    float Given;
    float output;
    float delta;
    unsigned int count;
    unsigned int length;
} RAMP_REFERENCE;

/*调节器结构体*/
typedef struct
{
    float Kp;
    float Ki;
    float Kc;
    float ref;
    float fdb;
    float err;
    float ui;
    float up;
    float upresat;
    float uo;
    float upper_limit;
    float lower_limit;
} PID;
#define PID_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

extern float waveA, waveB, waveC;
extern float theta_50Hz, PLL_theta;

extern float back_d, back_q;

extern float test1, test2, test3;

extern PID Ud_pid;
extern PID Uq_pid;
extern PID Id_pid;
extern PID Iq_pid;
extern PID PLL_pid;
extern RAMP_REFERENCE Ud_ramp;
extern RAMP_REFERENCE Uq_ramp;
extern RAMP_REFERENCE Id_ramp;
extern RAMP_REFERENCE Iq_ramp;

extern float U_feedback_d, U_feedback_q;
extern float I_feedback_d, I_feedback_q;

extern void THETA_GENERATE(void);
extern void INV_XY_CAL(void);
extern void OPEN_LOOP(float Modulation);
extern void VOLTAGE_CLOSED_LOOP(float V_ref);
extern void CURRENT_CLOSED_LOOP(float I_ref, float I_q);
extern void PHASE_LOCKED_LOOP(void);

#endif /* APP_INVERTER_H_ */