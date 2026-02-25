#include "struct_typedef.h"
#include "pid.h"
typedef struct
{

	float yaw_target;//小yaw目标角
	float pitch_target;//小pitch目标角
	float yaw_vision_target;//视觉模式下小yaw目标角
	float pitch_vision_target;//视觉模式下小pitch目标角
	float lowpass_yaw_target;//低通滤波后小yaw目标角（键鼠）
	float lowpass_pitch_target;//低通滤波后小pitch目标角（键鼠）
	uint16_t vision_block;//视觉响应超时
	uint16_t found_flag;//视觉发现目标

	float output_yaw;//小yaw目标输出电流
	float output_pitch;//小pitch目标输出电流
	uint16_t last_mode;//云台上一时刻模式
	uint16_t PT_flag;//平头触发标志位
	uint32_t DT_Time;//掉头时间
	uint16_t IF_DT_OVER;//掉头完成标志位
	uint16_t IF_DT;//掉头触发标志位
	uint16_t IF_FOLLOW_ALLOW;//运行底盘跟随标志位
	uint16_t IF_FOLD;//触发折叠标志位
	uint16_t IF_FOLD_OVER;//折叠完成标志位
	uint16_t rise_over;//大pitch升起完成标志位
	uint16_t down_over;//大pitch折叠完成标志位
	uint16_t angle_limit_flag;//限位标志位
	uint16_t target_renew_flag;
	uint16_t big_pitch_allow_flag;
	float big_pitch_target;//大pitch目标角
	float big_pitch_output;//大pitch目标输出电流
	float kalman_pitch_target;
	float fold_yaw_pos;//折叠目标角
	float fold_big_pitch_pos;//折叠目标角
	float ratio_yaw;//大yaw转动系数
	float last_yaw_vision_target;//上一时刻视觉传输的目标值
}GIMBAL_t;

typedef struct
{
  float Inertia;//云台转动惯量
  float Viscosity_coefficient; //粘滞系数
  float Gravity;//重力矩
  float F_Coulomb; //库伦力
  
  float target_acc;//经过pid算出来的目标加速度
  float target_vel;//目标角速度
  float target_tq;//通过建模算出来的最佳力矩
}Modeling_Parameters_t;
//力矩=转动惯量乘以角加速度+粘滞系数乘以角速度+重力矩乘以sin(角度)+库伦摩擦力乘以角速度的方向
//  力矩T = Inertia*target_yaw_acc + Viscosity_coefficient * target_yaw_vel +Gravity*sin(target_angle) + F_Coulomb*sign(target_yaw_vel)

void gimbal_init(void);
void gimbal_pid_calc(void);
void gimbal_mode_rc_ctrl(void);
void gimbal_mode_rc_idle(void);
void gimbal_mode_rc_normal(void);
void gimbal_mode_rc_vision(void);
void gimbal_mode_rc_fold(void);

float pitch_protect(float data);
void gimbal_u_turn(void);
int Get_Sign(float a);
float tq_limit(float tq);
float Modeling_Parameters_cacl(Modeling_Parameters_t Modeling_Parameters);
void gimbal_mode_key_ctrl(void);
void gimbal_mode_key_idle(void);
void gimbal_mode_key_normal(void);
void gimbal_mode_key_fold(void);

void gimbal_mode_key_vision(void);
float record_small_yaw_pos(void);
float record_big_pitch_pos(void);
float zero_180(float angle);
void fold_state_judge(void);
void u_turn(void);
void turn_round(void);	
float shortestAngleDiff(float current, float target) ;


void yaw_limit(void);
float zero_PI(float angle);


extern GIMBAL_t GIMBAL;



//写一个根据掉头时的标志位决定底盘状态

