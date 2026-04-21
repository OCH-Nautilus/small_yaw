#ifndef __SMC_H
#define __SMC_H

#include "stdbool.h"

typedef struct
{
	float C;
	float K;
	float ref;         //初始目标值
	float error_eps;   //误差下限
	float u_max;       //输出最大值
	float J;           //估计惯量
	float angle;       //角度反馈   °
	float ang_vel;     //角速度反馈 °/s
	float epsilon;     //近端指数收敛项
	float u;           //输入
	float phi;         //饱和函数厚度
	
	float error_now;
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面
}SMC_struct_t;

typedef struct
{
	float A;
	float B;
	float Y1;
	float Y2;
	float K1;
	float K2;
	float phi;
	float dt;
	float i;
	float i_max;
	float i_out;
	
	float ref;         //初始目标值
	float error_eps;   //误差下限
	float u_max;       //输出最大值
	float J;           //估计惯量
	float angle;       //角度反馈   °
	float ang_vel;     //角速度反馈 °/s
	float u;           //输入
	
	float error_now;
	float error_last;
	float error_d;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面
}FTSMC_struct_t;

float Sat_SMC(float s , float phi);
void FTSMC_init(FTSMC_struct_t *FTSMC,  //结构体               Y1 > Y2 , 1 < Y2 < 2
	              float A,            //大误差收敛速度        
                float B,            //小误差收敛速度        
	              float Y1,           //大误差非线性强度          
                float Y2,           //小误差收敛速度/奇异性     越大越慢
                float K1,           //抗扰动能力    
                float K2,           //趋近速度/平滑性       
								float phi,          //饱和函数厚度
								float u_max,        //输入限幅
								float error_eps,    //误差下限
								float J,            //惯量
								float dt,           //运行时间间隔 
								float i,
								float i_max);             
float FTSMC_calc(FTSMC_struct_t *FTSMC, float get_ang_vel,float get_angle, float set_angle);							
									
								
typedef struct
{
	float A;           // 快速终端非线性项增益 (大误差收敛)
	float Y1;          // 快速终端指数 (必须满足 0 < Y1 < 1，通常取0.5~0.8)
	float K1;          // 鲁棒项增益 (抗扰动能力)
	float K2;          // 线性比例增益 (趋近速度)
	float phi;         // 饱和函数边界层厚度
	float dt;          // 运行时间间隔
	float i;           // 积分增益
	float i_max;       // 积分限幅
	float i_out;       // 积分器输出
	
	float ref;         // 目标速度
	float error_eps;   // 积分介入的误差阈值 (抗积分饱和)
	float u_max;       // 输出最大值
	float J;           // 估计系统惯量
	float vel;         // 速度反馈 (如: rad/s 或 RPM)
	float u;           // 控制输入输出
	
	float error_now;   // 当前速度误差
	float error_last;  // 上次速度误差
	float dref;        // 目标速度一阶导 (前馈角加速度)
	float refl;        // 上一次的目标速度

	float s;           // 滑模面
} Vel_SMC_struct_t;

void Vel_SMC_init(Vel_SMC_struct_t *SMC, 
					float A, 
					float Y1, 
					float K1, 
					float K2, 
					float phi, 
					float u_max,
					float error_eps, 
					float J, 
					float dt, 
					float i, 
					float i_max);

float Vel_SMC_calc(Vel_SMC_struct_t *SMC, float get_vel, float set_vel);					
#endif
