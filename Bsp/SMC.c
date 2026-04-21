#include "SMC.h" 
#include "pid.h"
#include "math.h"
#include "string.h"
#include "user_lib.h"


//经典线性滑模
void SMC_init(SMC_struct_t *SMC,  //结构体
	              float C,          //比例        20
                float K,          //增益        120
	              float error_eps,  //误差死区     0.001
                float u_max,      //输入幅值     7 
                float J,          //惯量         ？
                float epsilon,    //趋近律       0.5
								float phi)        //饱和函数厚度  ？
{ 
  memset(SMC, 0, sizeof(SMC_struct_t));
	SMC->C            = C;
	SMC->K            = K;
	SMC->error_eps    = error_eps;
	SMC->u_max        = u_max;
	SMC->J            = J;
	SMC->phi          = phi;
	SMC->epsilon      = epsilon;
	SMC->ref          = 0;
}


float SMC_calc(SMC_struct_t *SMC, float get_ang_vel,float get_angle, float set_angle)
{
	SMC->ang_vel    = get_ang_vel;
  SMC->angle      = get_angle;
	SMC->ref        = set_angle;
  SMC->ddref      = (SMC->ref - SMC->refl) - SMC->dref;
	SMC->dref       = SMC->ref - SMC->refl;
	SMC->error_now  = get_angle - set_angle;
   
	if((fabs(SMC->error_now) < SMC->error_eps))
	  SMC->u = 0;
  else
	{
	  SMC->s = SMC->C*SMC->error_now + SMC->ang_vel - SMC->dref;
		SMC->u = SMC->J*(SMC->ddref - SMC->C * (SMC->ang_vel - SMC->dref) - SMC->epsilon * Sat_SMC(SMC->s , SMC->phi) - SMC->K * SMC->s);
	}
	
	abs_limit(SMC->u , SMC->u_max);
	
	SMC->error_last = SMC->error_now;
	SMC->refl = SMC->ref;
	
	return SMC->u;
}

float Sat_SMC(float s , float phi)
{
	float sum=0; 
	if(s > phi)
		sum = 1;
	else if(fabs(s) < phi)
		sum = s/phi;
	else if(s < -phi)
		sum = -1;
	return sum;
}



//非奇异快速终端滑模
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
								float i_max)            
{ 
	memset(FTSMC, 0, sizeof(FTSMC_struct_t));
	FTSMC->A            = A;
	FTSMC->B            = B;
	FTSMC->Y1           = Y1;
	FTSMC->Y2           = Y2;
	FTSMC->K1           = K1;
	FTSMC->K2           = K2;
	FTSMC->phi          = phi;
 	FTSMC->dt           = dt;         
	FTSMC->i            = i;
	FTSMC->i_max        = i_max;
	
	FTSMC->u_max        = u_max;
	FTSMC->error_eps    = error_eps;
	FTSMC->J            = J;
}


float FTSMC_calc(FTSMC_struct_t *FTSMC, float get_ang_vel,float get_angle, float set_angle)
{
  FTSMC->angle      = get_angle;
	FTSMC->ref        = set_angle;
	FTSMC->ang_vel    = get_ang_vel;
	
  FTSMC->ddref      = ((FTSMC->ref - FTSMC->refl)/FTSMC->dt - FTSMC->dref)/FTSMC->dt;
	FTSMC->dref       = (FTSMC->ref - FTSMC->refl)/FTSMC->dt;
 
	FTSMC->error_now  = set_angle - get_angle;
  FTSMC->error_d    = FTSMC->dref - FTSMC->ang_vel;
	
	if(fabs(set_angle - get_angle)<1.5f*0.1745f)
	FTSMC->i_out += FTSMC->i*(set_angle - get_angle);
	else FTSMC->i_out = 0;
		
  if(FTSMC->i_out>=FTSMC->i_max) FTSMC->i_out=FTSMC->i_max;
	else if(FTSMC->i_out<=-FTSMC->i_max) FTSMC->i_out=FTSMC->i_max;
	
//	if((fabs(FTSMC->error_now) < FTSMC->error_eps))
//	  FTSMC->u = 0;
//  else
//	{
	  FTSMC->s = FTSMC->error_now + FTSMC->A*pow(fabs(FTSMC->error_now), FTSMC->Y1)*sign(FTSMC->error_now) + FTSMC->B*pow(fabs(FTSMC->error_d), FTSMC->Y2)*sign(FTSMC->error_d);
		FTSMC->u = FTSMC->J*( FTSMC->ddref + 1/(FTSMC->B*FTSMC->Y2)*pow(fabs(FTSMC->error_d),2.0f - FTSMC->Y2)*sign(FTSMC->error_d)*(1+FTSMC->A*FTSMC->Y1*pow(fabs(FTSMC->error_now),FTSMC->Y1 - 1.0f)) + FTSMC->K1*Sat_SMC(FTSMC->s , FTSMC->phi) + FTSMC->K2*FTSMC->s) + FTSMC->i_out;
//	}
	
	abs_limit(FTSMC->u , FTSMC->u_max);
	
	FTSMC->error_last = FTSMC->error_now;
	FTSMC->refl = FTSMC->ref;
	
	return FTSMC->u;
}


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
									float i_max)                  
{ 
	memset(SMC, 0, sizeof(Vel_SMC_struct_t));
	SMC->A         = A;
	SMC->Y1        = Y1;
	SMC->K1        = K1;
	SMC->K2        = K2;
	SMC->phi       = phi;
	SMC->dt        = dt;         
	SMC->i         = i;
	SMC->i_max     = i_max;
	SMC->u_max     = u_max;
	SMC->error_eps = error_eps;
	SMC->J         = J;
}


float Vel_SMC_calc(Vel_SMC_struct_t *SMC, float get_vel, float set_vel)
{
	SMC->vel = get_vel;
	SMC->ref = set_vel;
	
	// 1. 计算目标加速度 (前馈项，提升对变速指令的响应)
	SMC->dref = (SMC->ref - SMC->refl) / SMC->dt;
 
	// 2. 计算速度误差
	SMC->error_now = SMC->ref - SMC->vel;
	
	// 3. 积分项计算 (带有误差窗口的抗积分饱和)
	// 原代码是角度小于约15度时积分，这里改为速度误差小于设定阈值(error_eps)时积分
	if(fabs(SMC->error_now) < SMC->error_eps)
		SMC->i_out += SMC->i * SMC->error_now;
	else 
		SMC->i_out = 0; // 误差过大时清零积分，防止超调 (或根据需求改为保持不变)
		
	// 积分限幅
	if(SMC->i_out >= SMC->i_max) SMC->i_out = SMC->i_max;
	else if(SMC->i_out <= -SMC->i_max) SMC->i_out = -SMC->i_max;
	
	// 4. 定义一阶滑模面
	SMC->s = SMC->error_now;

	// 5. 快速终端滑模控制律 (Equivalent Control + Fast Terminal Reaching Law)
	SMC->u = SMC->J * (SMC->dref 
	         + SMC->A * pow(fabs(SMC->s), SMC->Y1) * sign(SMC->s) 
	         + SMC->K1 * Sat_SMC(SMC->s, SMC->phi) 
	         + SMC->K2 * SMC->s) 
	         + SMC->i_out;
	
	// 6. 输出限幅
	abs_limit(SMC->u, SMC->u_max);
	
	// 7. 更新历史值
	SMC->error_last = SMC->error_now;
	SMC->refl       = SMC->ref;
	
	return SMC->u;
}
