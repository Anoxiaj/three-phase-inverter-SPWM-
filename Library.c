#include "main.h"

/*Sample varible*/
#define SAMPLE_PERIOD 0.0001

THETA_REGS U_theta;
THETA_REGS I_theta;

/*matlab_varible---->C_varible*/
float32 Sample_vol_A, Sample_vol_B, Sample_vol_C;
float32 Sample_curr_A, Sample_curr_B, Sample_curr_C;

float32 Vref, Iref, Vdc;

/*voltage & current ramp given varible*/
float32 ramp_Ud_delta, ramp_Ud_length, ramp_Ud_count, ramp_Ud_output; // 电压d变化率，变化步长，变化计数，变化输出
float32 ramp_Uq_delta, ramp_Uq_length, ramp_Uq_count, ramp_Uq_output; // 电压q变化率，变化步长，变化计数，变化输出
float32 ramp_Id_delta, ramp_Id_length, ramp_Id_count, ramp_Id_output; // 电流d变化率，变化步长，变化计数，变化输出
float32 ramp_Iq_delta, ramp_Iq_length, ramp_Iq_count, ramp_Iq_output; // 电流q变化率，变化步长，变化计数，变化输出

/// @brief matlab_varible---->C_varible
/// @param out_var
/// @param in_var
void Var_allocation(double out[6], double in[12])
{
}

/// @brief sin&cos calculate
/// @param U_theta
void sin_cos_cal(THETA_REGS *p)
{
    p->cos_theta = cosf(p->theta);
    p->sin_theta = sinf(p->theta);
    p->cos_120 = -0.5;
    p->sin_120 = 0.8660254;

    p->cos_2theta = (p->cos_theta * p->cos_theta) - (p->sin_theta * p->sin_theta); // cos(2θ)
    p->sin_2theta = (p->sin_theta * p->cos_theta) * 0.5;                           // sin(2θ)

    p->cos_theta_p_120 = (p->cos_theta * p->cos_120) - (p->sin_theta * p->sin_120); // cos(theta+120)
    p->cos_theta_m_120 = (p->cos_theta * p->cos_120) + (p->sin_theta * p->sin_120); // cos(theta-129)

    p->sin_theta_p_120 = (p->sin_theta * p->cos_120) + (p->cos_theta * p->sin_120); // sin(theta+120)
    p->sin_theta_m_120 = (p->sin_theta * p->cos_120) - (p->cos_theta * p->sin_120); // sin(theta-120)
}

/// @brief angle variable initialization
void THETA_REGS_VAR_INIT(THETA_REGS *p)
{
    p->theta = 0;
    p->cos_theta = 0;
    p->sin_theta = 0;
    p->cos_2theta = 0;
    p->sin_2theta = 0;

    p->cos_theta_p_120 = 0;
    p->cos_theta_m_120 = 0;
    p->sin_theta_p_120 = 0;
    p->sin_theta_m_120 = 0;
    p->cos_120 = 0;
    p->sin_120 = 0;
}

// 正序相反变换
void dq2abc(DQ2ABC *p, THETA_REGS *q)
{
#define c1 0.666666667 // 2/3
    p->a = q->cos_theta * p->d - q->sin_theta * p->q;
    p->b = q->cos_theta_m_120 * p->d - q->sin_theta_m_120 * p->q;
    p->c = q->cos_theta_p_120 * p->d - q->sin_theta_p_120 * p->q;

    //	 p->b= 2;//q->cos_theta_m_120*p->d- q->sin_theta_m_120*p->q;
    //	 p->c= 3;//q->cos_theta_p_120*p->d- q->sin_theta_p_120*p->q;
}

/// @brief abc -> alpha,beta (constant amplitude transform)
void Clark(CLARK_REGS *p)
{
    p->alpha = p->a; // 2/3(ua - ub/2  -uc/2)
    p->beta = (p->b - p->c) / sqrt(3);
}

void iClark(ICLARK_REGS *p)
{
    p->a = p->alpha;
    p->b = (p->alpha * (-0.5)) + (p->beta * sqrt(3) / 2);
    p->c = (p->alpha * (-0.5)) - (p->beta * sqrt(3) / 2);
}

/// @brief alpha,beta -> d,q (constant amplitude transform)
void Park(PARK_REGS *p, THETA_REGS *q)
{
    p->d = (p->alpha * q->cos_theta) + (p->beta * q->sin_theta);
    p->q = (-p->alpha * q->sin_theta) + (p->beta * q->cos_theta);
}

void iPark(IPARK_REGS *p, THETA_REGS *q)
{
    p->alpha = (p->d * q->cos_theta) + (-p->q * q->sin_theta);
    p->beta = (p->d * q->sin_theta) + (p->q * q->cos_theta);
}

/// @brief voltage d ramp given
/// @param ramp_Ud_given
void Ramp_Ud_Given(float32 ramp_Ud_given)
{
    ramp_Ud_count = ramp_Ud_count + 1;
    if (ramp_Ud_count >= ramp_Ud_length)
    {
        ramp_Ud_count = 0;
        if (ramp_Ud_output < ramp_Ud_given)
        {
            ramp_Ud_output = ramp_Ud_output + ramp_Ud_delta;
            if (ramp_Ud_output > ramp_Ud_given)
                ramp_Ud_output = ramp_Ud_given;
        }

        if (ramp_Ud_output > ramp_Ud_given)
        {
            ramp_Ud_output = ramp_Ud_output - ramp_Ud_delta;
            if (ramp_Ud_output < ramp_Ud_given)
                ramp_Ud_output = ramp_Ud_given;
        }
    }
}

void Ramp_Uq_Given(float32 ramp_Uq_given)
{
    ramp_Uq_count = ramp_Uq_count + 1;
    if (ramp_Uq_count >= ramp_Uq_length)
    {
        ramp_Uq_count = 0;
        if (ramp_Uq_output < ramp_Uq_given)
        {
            ramp_Uq_output = ramp_Uq_output + ramp_Uq_delta;
            if (ramp_Uq_output > ramp_Uq_given)
                ramp_Uq_output = ramp_Uq_given;
        }

        if (ramp_Uq_output > ramp_Uq_given)
        {
            ramp_Uq_output = ramp_Uq_output - ramp_Uq_delta;
            if (ramp_Uq_output < ramp_Uq_given)
                ramp_Uq_output = ramp_Uq_given;
        }
    }
}

void Ramp_Id_Given(float32 ramp_Id_given)
{
    ramp_Id_count = ramp_Id_count + 1;
    if (ramp_Id_count >= ramp_Id_length)
    {
        ramp_Id_count = 0;
        if (ramp_Id_output < ramp_Id_given)
        {
            ramp_Id_output = ramp_Id_output + ramp_Id_delta;
            if (ramp_Id_output > ramp_Id_given)
                ramp_Id_output = ramp_Id_given;
        }

        if (ramp_Id_output > ramp_Id_given)
        {
            ramp_Id_output = ramp_Id_output - ramp_Id_delta;
            if (ramp_Id_output < ramp_Id_given)
                ramp_Id_output = ramp_Id_given;
        }
    }
}

void Ramp_Iq_Given(float32 ramp_Iq_given)
{
    ramp_Iq_count = ramp_Iq_count + 1;
    if (ramp_Iq_count >= ramp_Iq_length)
    {
        ramp_Iq_count = 0;
        if (ramp_Iq_output < ramp_Iq_given)
        {
            ramp_Iq_output = ramp_Iq_output + ramp_Iq_delta;
            if (ramp_Iq_output > ramp_Iq_given)
                ramp_Iq_output = ramp_Iq_given;
        }

        if (ramp_Iq_output > ramp_Iq_given)
        {
            ramp_Iq_output = ramp_Iq_output - ramp_Iq_delta;
            if (ramp_Iq_output < ramp_Iq_given)
                ramp_Iq_output = ramp_Iq_given;
        }
    }
}

/// @brief PID calculation
/// @param p
void Pid_calculation(PID *p)
{
    // 误差
    p->err = p->ref - p->fdb;

    // 比例
    p->up = p->err * p->Kp;
    // 给比例限幅
    if (p->up >= p->upper_limit)
    {
        p->up = p->upper_limit;
    }
    else if (p->up <= p->lower_limit)
    {
        p->up = p->lower_limit;
    }

    // 积分
    p->ui = p->ui + ((p->err * p->Ki) * SAMPLE_PERIOD);
    // 给积分限幅
    if (p->ui >= p->upper_limit)
    {
        p->ui = p->upper_limit;
    }
    else if (p->ui <= p->lower_limit)
    {
        p->ui = p->lower_limit;
    }

    // 求和
    p->upresat = p->up + p->ui; /* + _IQmpy(p->Kc,(p->uo - p->upresat));*/
    // 给输出限幅
    if (p->upresat >= p->upper_limit)
    {
        p->uo = p->upper_limit;
    }
    else if (p->upresat <= p->lower_limit)
    {
        p->uo = p->lower_limit;
    }
    else
    {
        p->uo = p->upresat;
    }
}