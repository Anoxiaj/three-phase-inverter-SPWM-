#include "main.h"

float theta_50Hz;
#define PIE 3.1415926535897932384626433832795
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;

float waveA, waveB, waveC;

float test1, test2, test3;

PID Ud_pid;
PID Uq_pid;
PID Id_pid;
PID Iq_pid;
RAMP_REFERENCE Ud_ramp;
RAMP_REFERENCE Uq_ramp;
RAMP_REFERENCE Id_ramp;
RAMP_REFERENCE Iq_ramp;

// for pid control
float U_feedback_d, U_feedback_q;
float I_feedback_d, I_feedback_q;

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

    // test1 = UClark.alpha;
    // test2 = UClark.beta;
    // test3 = U_theta.theta;

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
    Ud_ramp.length = 5;

    Ramp_Given(&Ud_ramp);
    Ud_pid.ref = Ud_ramp.output * 2 / Vdc;
    Ud_pid.fdb = U_feedback_d * 2 / Vdc;

    /*d轴PID计算*/
    Ud_pid.Kp = 0.01;
    Ud_pid.Ki = 50;

    Ud_pid.upper_limit = 1;  // d轴PID限幅
    Ud_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Ud_pid); // 计算出d轴pid输出

    /*q轴斜坡给定*/
    Uq_ramp.Given = 0;
    Uq_ramp.delta = 1;
    Uq_ramp.length = 5;

    Ramp_Given(&Uq_ramp);
    Uq_pid.ref = Uq_ramp.output * 2 / Vdc;
    Uq_pid.fdb = U_feedback_q * 2 / Vdc;

    /*q轴PID计算*/
    Uq_pid.Kp = 0.01;
    Uq_pid.Ki = 50;

    Uq_pid.upper_limit = 1;  // d轴PID限幅
    Uq_pid.lower_limit = -1; // d轴PID限幅

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
    waveA = (UiClark.a + 1) / 2;
    waveB = (UiClark.b + 1) / 2;
    waveC = (UiClark.c + 1) / 2;

    test1 = Ud_pid.err;
    test2 = Uq_pid.err;
    // test3 = waveC;
}

void CURRENT_CLOSED_LOOP(float I_ref)
{
    IPARK_REGS IiPark;
    ICLARK_REGS IiClark;

    /*d轴斜坡给定*/
    Ud_ramp.Given = I_ref;
    Ud_ramp.delta = 1;
    Ud_ramp.length = 5;

    Ramp_Given(&Id_ramp);
    Id_pid.ref = Id_ramp.output;
    Id_pid.fdb = I_feedback_d;

    /*d轴PID计算*/
    Id_pid.Kp = 0.01;
    Id_pid.Ki = 50;

    Id_pid.upper_limit = 1000;  // d轴PID限幅
    Id_pid.lower_limit = -1000; // d轴PID限幅

    Pid_calculation(&Id_pid); // 计算出d轴pid输出

    /*q轴斜坡给定*/
    Iq_ramp.Given = 0;
    Iq_ramp.delta = 1;
    Iq_ramp.length = 5;

    Ramp_Given(&Iq_ramp);
    Iq_pid.ref = Iq_ramp.output;
    Iq_pid.fdb = I_feedback_q;

    /*q轴PID计算*/
    Iq_pid.Kp = 0.01;
    Iq_pid.Ki = 50;

    Iq_pid.upper_limit = 1;  // d轴PID限幅
    Iq_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Iq_pid); // 计算出q轴pid输出

    /*把pid计算出的dq输出值赋给park反变换的dq*/
    IiPark.d = Id_pid.uo;
    IiPark.q = Iq_pid.uo;

    iPark(&IiPark, &I_theta); // Park反变换

    /*把Park反变换的α和β赋给Clark反变换的α和β*/
    IiClark.alpha = IiPark.alpha;
    IiClark.beta = IiPark.beta;

    iClark(&IiClark); // Clark反变换

    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = IiClark.a;
    waveB = IiClark.b;
    waveC = IiClark.c;

    test1 = Id_pid.err;
    test2 = Iq_pid.err;
    // test3 = waveC;
}