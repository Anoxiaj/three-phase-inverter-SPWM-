#include "mex.h"
#include "main.h"

static int pulse_f = 0;
static int pulse_f_Old = 0; // 上面两个参数控制代码每周期只执行一次 模拟芯片里的操作
float32 m = 0;				// 调制度

/*变量定义*/

void SPWM_2Closed_loop(double out_var[9], double in_var[18]) // 相当于主函数名：example_func【可以按照想法更改，最后一行处也要改】// out_var[6]输出变量，个数为6  in_var[6]输出变量，个数为6
{
	pulse_f = in_var[9];

	/*given*/
	Vref = in_var[0];	// 参考相电压
	Iref = in_var[1];	// 参考电感电流
	Vdc = in_var[8];	// 直流母线电压
	m = 2 * Vref / Vdc; // SPWM调制度

	if (in_var[10] == 0)			   // 延时启动，一开始时钟信号都是0
	{								   // 初始化
		THETA_REGS_VAR_INIT(&U_theta); // 角度计算变量初始化
		THETA_REGS_VAR_INIT(&I_theta);
		THETA_REGS_VAR_INIT(&G_theta);
		PID_VAR_INIT(&Ud_pid); // PID参数变量初始化
		PID_VAR_INIT(&Uq_pid);
		PID_VAR_INIT(&PLL_pid);
		RAMP_VAR_INIT(&Ud_ramp); // 斜坡给定变量初始化
		RAMP_VAR_INIT(&Uq_ramp);
		RAMP_VAR_INIT(&Id_ramp);
		RAMP_VAR_INIT(&Iq_ramp);
		theta_50Hz = 0; // 角度生成变量初始化
		PLL_theta = 0;
		G_theta.theta = 0;
		waveA = 0; // 调制波
		waveB = 0;
		waveC = 0;
		m = 0;
	}

	if (pulse_f_Old == 0 && pulse_f == 1)
	{
		/*Sample*/
		Sample_curr_A = in_var[2]; // 电流采样变量
		Sample_curr_B = in_var[3];
		Sample_curr_C = in_var[4];

		Sample_vol_A = in_var[5]; // 电压采样变量
		Sample_vol_B = in_var[6];
		Sample_vol_C = in_var[7];

		Sample_Grid_A = in_var[12]; // 电网电压采样变量
		Sample_Grid_B = in_var[13];
		Sample_Grid_C = in_var[14];

		PHASE_LOCKED_LOOP(); // 角度生成-->G_theta

		THETA_GENERATE();	   // 角度生成-->U_theta, I_theta
		sin_cos_cal(&U_theta); // 正余弦计算
		sin_cos_cal(&I_theta);
		INV_XY_CAL(); // 坐标变换-->I_feedback_d, I_feedback_q, U_feedback_d, U_feedback_q

		// OPEN_LOOP(m);
		// VOLTAGE_CLOSED_LOOP(Vref);

#if switch_loop
		CURRENT_CLOSED_LOOP(Iref, 0); // 电流单闭环
#else
		CURRENT_CLOSED_LOOP(back_d, back_q); // 双闭环
#endif
	}

	// 4、载波调制 载波in_var[5]; 因为脉冲要一直比较，所以放到最外层，每个仿真时间执行一次
	// 无死区；
	out_var[0] = waveA > in_var[11] ? 1 : 0;
	out_var[1] = 1 - out_var[0];
	out_var[2] = waveB > in_var[11] ? 1 : 0;
	out_var[3] = 1 - out_var[2];
	out_var[4] = waveC > in_var[11] ? 1 : 0;
	out_var[5] = 1 - out_var[4];
	// 测试端口6-8
	out_var[6] = test1;
	out_var[7] = test2;
	out_var[8] = test3;
	// out_var[6] = Sample_curr_A;
	// out_var[7] = Sample_curr_B;
	// out_var[8] = Sample_curr_C;

	pulse_f_Old = pulse_f;
}

void mexFunction( // 此处可更改地方
	int nlhs,	  //     |
	mxArray *plhs[],
	int nrhs, //     |
	const mxArray *prhs[])
{ //     |
	float *out;
	float *in; //     |

	// allocate memory for output                           //     |
	plhs[0] = mxCreateDoubleMatrix(9, 1, mxREAL); // 输出矩阵 列向量 这里输出数据的个数要和模块里的个数一一对应
												  //(6,1,mxREAL)  【参数6表示输出变量维数（输出变量个数）】  【1和mxREAL默认不变】
	out = mxGetPr(plhs[0]);
	in = mxGetPr(prhs[0]);
	SPWM_2Closed_loop(out, in); // example_func为主函数名【可以按照想法更改】    out->plhs     in->prhs
}
