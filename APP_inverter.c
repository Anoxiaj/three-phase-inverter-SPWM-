#include "main.h"

float theta_50Hz;
#define PIE 3.1415926535897932384626433832795
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;

float32 waveA, waveB, waveC;

PID Ud_pid;
PID Uq_pid;
RAMP_REFERENCE Ud_ramp;
RAMP_REFERENCE Uq_ramp;
RAMP_REFERENCE Id_ramp;
RAMP_REFERENCE Iq_ramp;

// for pid control
float32 U_feedback_d, U_feedback_q;
float32 I_feedback_d, I_feedback_q;

/// @brief Generate the angle of system control
void THETA_GENERATE(void)
{
    /* theta=wt , w=2pie f */
    theta_50Hz = theta_50Hz + 0.0001 * PIEx100; // 0.0001:interrupt time 10K
    theta_50Hz = theta_50Hz > PIEx2 ? (theta_50Hz - PIEx2) : theta_50Hz;
    theta_50Hz = theta_50Hz < 0 ? (theta_50Hz + PIEx2) : theta_50Hz;

    U_theta.theta = theta_50Hz;
    I_theta.theta = theta_50Hz;
}

/// @brief coordinate transformation
void INV_XY_CAL(void)
{
    CLARK_REGS UClark;
    PARK_REGS UPark;

    CLARK_REGS IClark;
    PARK_REGS IPark;

    /*电压*/
    UClark.a = Sample_vol_A;
    UClark.b = Sample_vol_B;
    UClark.c = Sample_vol_C;
    Clark(&UClark); // ABC->alpha,beta
    UPark.alpha = UClark.alpha;
    UPark.beta = UClark.beta;
    Park(&UPark, &U_theta); // alpha,beta->d,q
    U_feedback_d = UPark.d;
    U_feedback_q = UPark.q;

    /*电流*/
    IClark.a = Sample_curr_A;
    IClark.b = Sample_curr_B;
    IClark.c = Sample_curr_C;
    Clark(&IClark); // ABC->alpha,beta
    IPark.alpha = IClark.alpha;
    IPark.alpha = IClark.beta;
    Park(&IPark, &I_theta); // alpha,beta->d,q
    I_feedback_d = IPark.d;
    I_feedback_q = IPark.q;
}

void OPEN_LOOP(float Modulation)
{
    DQ2ABC v;
    v.d = Modulation;
    v.q = 0;
    dq2abc(&v, &U_theta);

    waveA = (v.a + 1) / 2;
    waveB = (v.b + 1) / 2;
    waveC = (v.c + 1) / 2;
}

/// @brief PID Voltage Closed Loop
/// @param V_ref
void VOLTAGE_CLOSED_LOOP(float V_ref)
{
    IPARK_REGS UiPark;
    ICLARK_REGS UiClark;

    /*d轴斜坡给定*/
    Ud_ramp.Given = V_ref;
    Ud_ramp.delta = 1;
    Ud_ramp.length = 10;

    Ramp_Given(&Ud_ramp);
    Ud_pid.ref = Ud_ramp.output;
    Ud_pid.fdb = U_feedback_d;

    /*d轴PID计算*/
    Ud_pid.Kp = 0.5;
    Ud_pid.Ki = 0.1;

    Ud_pid.upper_limit = V_ref * 1.44;  // d轴PID限幅
    Ud_pid.lower_limit = -V_ref * 1.44; // d轴PID限幅

    Pid_calculation(&Ud_pid); // 计算出d轴pid输出

    /*q轴斜坡给定*/
    Uq_ramp.Given = 0;
    Uq_ramp.delta = 1;
    Uq_ramp.length = 10;

    Ramp_Given(&Uq_ramp);
    Uq_pid.ref = Uq_ramp.output;
    Uq_pid.fdb = U_feedback_q;

    /*q轴PID计算*/
    Uq_pid.Kp = 0.5;
    Uq_pid.Ki = 0.1;

    Uq_pid.upper_limit = V_ref * 1.44;  // d轴PID限幅
    Uq_pid.lower_limit = -V_ref * 1.44; // d轴PID限幅

    Pid_calculation(&Uq_pid); // 计算出q轴pid输出

    /*把pid计算出的dq输出值赋给park反变换的dq*/
    UiPark.d = Ud_pid.uo;
    UiPark.q = Uq_pid.uo;

    iPark(&UiPark, &U_theta); // Park反变换

    /*把Park反变换的α和β赋给Clark反变换的α和β*/
    UiClark.alpha = UiPark.alpha;
    UiClark.beta = UiPark.beta;

    iClark(&UiClark); // Clark反变换

    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = UiClark.a / (Vdc / 2);
    waveB = UiClark.b / (Vdc / 2);
    waveC = UiClark.c / (Vdc / 2);
}
