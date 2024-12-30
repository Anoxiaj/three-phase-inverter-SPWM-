#include "main.h"

float theta_50Hz, PLL_theta;
#define PIE 3.1415926535897932384626433832795
const float PIEx2 = 6.283185307179586476925286766559;
const float PIEx100 = 314.15926535897932384626433832795;

float waveA, waveB, waveC;

float test1, test2, test3;

float back_d, back_q; // 双闭环传递参数

PID Ud_pid;
PID Uq_pid;
PID Id_pid;
PID Iq_pid;
PID PLL_pid;
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

    /*离网电流*/
    IClark.a = Sample_curr_A;
    IClark.b = Sample_curr_B;
    IClark.c = Sample_curr_C;
    Clark(&IClark); // ABC->alpha,beta
    IPark.alpha = IClark.alpha;
    IPark.beta = IClark.beta;
    Park(&IPark, &I_theta); // alpha,beta->d,q
    I_feedback_d = IPark.d;
    I_feedback_q = IPark.q;

    /*并网电流*/
    // IClark.a = Sample_curr_A;
    // IClark.b = Sample_curr_B;
    // IClark.c = Sample_curr_C;
    // Clark(&IClark); // ABC->alpha,beta
    // IPark.alpha = IClark.alpha;
    // IPark.beta = IClark.beta;
    // Park(&IPark, &G_theta); // alpha,beta->d,q
    // I_feedback_d = IPark.d;
    // I_feedback_q = IPark.q;

    // test1 = IPark.d;
    // test2 = IPark.q;
    // test3 = I_theta.theta;
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
    Ud_ramp.length = 1;

    Ramp_Given(&Ud_ramp);
    Ud_pid.ref = Ud_ramp.output;
    Ud_pid.fdb = U_feedback_d;

    /*d轴PID计算：单闭环时Kp=0.05 Ki=5*/ // 100: 2 300 ; 150:0.1 50
    Ud_pid.Kp = U_Kp_parameter;
    Ud_pid.Ki = U_Ki_parameter;
#if switch_loop
    /*单闭环时限幅为+-1*/
    Ud_pid.upper_limit = 1;  // d轴PID限幅
    Ud_pid.lower_limit = -1; // d轴PID限幅
#else
    Ud_pid.upper_limit = 200;  // d轴PID限幅
    Ud_pid.lower_limit = -200; // d轴PID限幅
#endif

    Pid_calculation(&Ud_pid); // 计算出d轴pid输出

    /*q轴斜坡给定*/
    Uq_ramp.Given = 0;
    Uq_ramp.delta = 1;
    Uq_ramp.length = 1;

    Ramp_Given(&Uq_ramp);
    Uq_pid.ref = Uq_ramp.output;
    Uq_pid.fdb = U_feedback_q;

    /*q轴PID计算*/
    Uq_pid.Kp = U_Kp_parameter;
    Uq_pid.Ki = U_Ki_parameter;

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
#if switch_loop
    /*电压单闭环模块*/
    /*对调制波进行归一化（SPWM的相电压最大值为(Vdc/2)*/
    waveA = (UiClark.a + 1) / 2;
    waveB = (UiClark.b + 1) / 2;
    waveC = (UiClark.c + 1) / 2;
#else
    /*双闭环模块*/
    back_d = Ud_pid.uo;
    back_q = Uq_pid.uo;
#endif
    // test1 = Ud_pid.err;
    // test2 = Uq_pid.err;
    // test3 = Ud_pid.uo;
}

/// @brief PID Current Closed Loop
/// @param I_ref
/// @param I_q
void CURRENT_CLOSED_LOOP(float I_ref, float I_q)
{
    IPARK_REGS IiPark;
    ICLARK_REGS IiClark;

    /*d轴斜坡给定*/
    Id_ramp.Given = I_ref;
    Id_ramp.delta = 1;
    Id_ramp.length = 1;

    Ramp_Given(&Id_ramp);
    Id_pid.ref = Id_ramp.output;
    Id_pid.fdb = I_feedback_d;

    /*d轴PID计算*/
    Id_pid.Kp = I_Kp_parameter;
    Id_pid.Ki = I_Ki_parameter;

    Id_pid.upper_limit = 1;  // d轴PID限幅
    Id_pid.lower_limit = -1; // d轴PID限幅

    Pid_calculation(&Id_pid); // 计算出d轴pid输出

    /*q轴斜坡给定*/
    Iq_ramp.Given = I_q;
    Iq_ramp.delta = 1;
    Iq_ramp.length = 1;

    Ramp_Given(&Iq_ramp);
    Iq_pid.ref = Iq_ramp.output;
    Iq_pid.fdb = I_feedback_q;

    /*q轴PID计算*/
    Iq_pid.Kp = I_Kp_parameter;
    Iq_pid.Ki = I_Ki_parameter;

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
    waveA = (IiClark.a + 1) / 2;
    waveB = (IiClark.b + 1) / 2;
    waveC = (IiClark.c + 1) / 2;

    // test1 = Id_pid.err;
    // test2 = Iq_pid.err;
    // test3 = Ud_pid.uo;
}

void PHASE_LOCKED_LOOP(void)
{
    CLARK_REGS GClark;
    PARK_REGS GPark;

    GClark.a = Sample_Grid_A;
    GClark.b = Sample_Grid_B;
    GClark.c = Sample_Grid_C;

    Clark(&GClark);

    GPark.alpha = GClark.alpha;
    GPark.beta = GClark.beta;

    sin_cos_cal(&G_theta);

    /*
    锁相环锁角度，与给定无关，只与q轴与A轴的关系有关
    q轴与A轴重合时，锁的是Cos型：Park(&GPark, &G_theta);
    q轴滞后A轴90°时，锁的是Sin型：Park_d90A(&GPark, &G_theta);
    */
    Park(&GPark, &G_theta);

    PLL_pid.ref = 0;
    PLL_pid.fdb = GPark.q;

    PLL_pid.Kp = 1;
    PLL_pid.Ki = 5;

    PLL_pid.upper_limit = +PIEx100;
    PLL_pid.lower_limit = -PIEx100;

    Pid_calculation(&PLL_pid);

    PLL_theta = PIEx100 - PLL_pid.uo;
    G_theta.theta = G_theta.theta + PLL_theta * 0.0001;

    // G_theta.theta = fmod(G_theta.theta, PIEx2);

    G_theta.theta = G_theta.theta > PIEx2 ? (G_theta.theta - PIEx2) : G_theta.theta;
    G_theta.theta = G_theta.theta < 0 ? (G_theta.theta + PIEx2) : G_theta.theta;

    test1 = 60 * G_theta.theta;
    test2 = Sample_Grid_A;
    test3 = GPark.q;
}