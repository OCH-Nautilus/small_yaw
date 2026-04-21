#include "struct_typedef.h"
#include "stdbool.h"
#include "mode_task.h"

typedef struct
{
	bool vision_fire;
	bool flag_if_single;
	bool flag_if_single_over;
	bool flag_if_back;
	bool flag_if_back_over;
	bool flag_if_flug[3];//0正拨,1反拨,2校准堵转
	bool flag_shoot_heat_warning; 
	bool if_cal;     //是否校准
	bool cal_step[2]; //校准的两个阶段 0为反转,1为偏移量控制
	
	float initial_ecd;        //校准后初始值
	float ecd; 
	float ecd_total;
	float v_trigger;
	float limit_v;
  	float tire_retreat_current;      //正转堵转计数
	float tire_retreat_current_back; //反转堵住计数
	float tire_retreat_current_cal;  //校准堵转计数
	float pid_trigger_single_out;
	float pid_trigger_long_out;
	float pid_trigger_out;
	trigger_state_t  last_trigger_mode;
	float cal_protect_start_time;
	float cal_protect_now_time;
}trigger_t;



void trigger_init(void);
void trigger_heat(void);
void trigger_retreat(void);
void trigger_mode(void);
void trigger_pid_calc(void);

extern trigger_t TRIGGER;
