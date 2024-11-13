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
extern float theta_50Hz;

extern PID Ud_pid;
extern PID Uq_pid;

extern float U_Clark_a, U_Clark_b, U_Clark_c, U_Clark_alpha, U_Clark_beta;
extern float I_Clark_a, I_Clark_b, I_Clark_c, I_Clark_alpha, I_Clark_beta;
extern float U_Park_alpha, U_Park_beta, U_Park_d, U_Park_q;
extern float I_Park_alpha, I_Park_beta, I_Park_d, I_Park_q;

extern float U_feedback_d, U_feedback_q;
extern float I_feedback_d, I_feedback_q;

extern void THETA_GENERATE(void);
extern void INV_XY_CAL(void);
extern void OPEN_LOOP(float Modulation);
extern void VOLTAGE_CLOSED_LOOP(float V_ref);

#endif /* APP_INVERTER_H_ */