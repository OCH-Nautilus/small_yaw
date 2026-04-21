#include "trigger_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "config.h"
#include "CAN_receive.h"
#include "bsp_transmit.h"
#include "mode_task.h"
#include "math.h"
#include <stdlib.h>
trigger_t TRIGGER;
uint32_t trigger_time;

pid_type_def pid_speed_trigger_single,pid_angle_trigger_single;   //单发pid
pid_type_def pid_speed_trigger_long;

void trigger_task(void const * argument)
{
  trigger_init();
  vTaskDelay(10);
  for(;;)
  {
	trigger_heat();
		trigger_retreat();
		trigger_mode();
		trigger_pid_calc();	
     
    vTaskDelay(1);
  }
  
}

//拨弹盘初始化
void trigger_init(void)
{
	TRIGGER.flag_shoot_heat_warning = 0;
	TRIGGER.flag_if_back = 0;
	TRIGGER.flag_if_back_over = 1;
	TRIGGER.flag_if_single = 0;
	TRIGGER.flag_if_single_over = 1;

	PID_init(&pid_angle_trigger_single, 0,PID_TRIGGER_ANGLE_SINGLE_KP, PID_TRIGGER_ANGLE_SINGLE_KI, PID_TRIGGER_ANGLE_SINGLE_KD,  PID_TRIGGER_ANGLE_SINGLE_IMAX, PID_TRIGGER_ANGLE_SINGLE_MAX);
  PID_init(&pid_speed_trigger_single,  0,PID_TRIGGER_SPEED_SINGLE_KP, PID_TRIGGER_SPEED_SINGLE_KI, PID_TRIGGER_SPEED_SINGLE_KD, PID_TRIGGER_SPEED_SINGLE_IMAX, PID_TRIGGER_SPEED_SINGLE_MAX);

	PID_init(&pid_speed_trigger_long, 0, PID_TRIGGER_SPEED_LONG_KP, PID_TRIGGER_SPEED_LONG_KI, PID_TRIGGER_SPEED_LONG_KD, PID_TRIGGER_SPEED_LONG_IMAX, PID_TRIGGER_SPEED_LONG_MAX);
}




//拨弹盘pid计算
void trigger_pid_calc(void)
{
	                                  PID_calc(&pid_angle_trigger_single , trigger_motor.total_ecd, TRIGGER.ecd_total);
	 TRIGGER.pid_trigger_single_out = PID_calc(&pid_speed_trigger_single , trigger_motor.speed_rpm , pid_angle_trigger_single.out);
	 TRIGGER.pid_trigger_long_out =   PID_calc(&pid_speed_trigger_long  ,  trigger_motor.speed_rpm , TRIGGER.v_trigger );
	
	#ifdef TRIGGER_SENT  
	if(mode.trigger_state == TRIGGER_BACK || mode.trigger_state == TRIGGER_SINGLE || mode.trigger_state == TRIGGER_STATIC || (mode.trigger_state == TRIGGER_CAL && TRIGGER.cal_step[0] == 0 && TRIGGER.cal_step[1] == 1))
	{                            
			TRIGGER.pid_trigger_out = TRIGGER.pid_trigger_single_out;
	}
	else if(mode.trigger_state == TRIGGER_LONG || (mode.trigger_state == TRIGGER_CAL && TRIGGER.cal_step[0] == 1 && TRIGGER.cal_step[1] == 0))
	{
	  TRIGGER.pid_trigger_out = TRIGGER.pid_trigger_long_out;
	}
	else if(mode.trigger_state == TRIGGER_IDLE)
	{
	  TRIGGER.pid_trigger_out = 0;
	}	
	#else 
	TRIGGER.pid_trigger_out = 0;
	#endif

}


//模式选择
void trigger_mode(void)    
{
	
	switch(mode.trigger_state)
	{
		case TRIGGER_SINGLE:    // 单发
			   if(TRIGGER.flag_if_single == 1)
				 {
//					 if(TRIGGER.flag_shoot_heat_warning == 0)
//					 {
					   TRIGGER.ecd_total += TRIGGER_ECD;
//					 }
					 TRIGGER.flag_if_single = 0;
				 }
					
				 if(fabs(trigger_motor.total_ecd - TRIGGER.ecd_total) < TRIGGER_SINGLE_OVER_THRESHOLD && TRIGGER.flag_if_single == 0)
				 {
					 TRIGGER.flag_if_single_over = 1;
				 }
			   break;
		
		case TRIGGER_LONG:      // 连发
			   TRIGGER.v_trigger =  TRIGGER.limit_v;	
         TRIGGER.ecd_total = trigger_motor.total_ecd+TRIGGER_ECD-((int)(trigger_motor.total_ecd - TRIGGER.initial_ecd)%(int)TRIGGER_ECD);
		     break;		
		
		case TRIGGER_BACK:    // 回退
		     if(TRIGGER.flag_if_back == 1)
				 {
					 TRIGGER.ecd_total -= TRIGGER_ECD;
					 TRIGGER.flag_if_back = 0;
				 }
					
				 if(fabs(trigger_motor.total_ecd - TRIGGER.ecd_total) < TRIGGER_BACK_OVER_THRESHOLD && TRIGGER.flag_if_back == 0)
				 {
					 TRIGGER.flag_if_back_over = 1;
					 TRIGGER.tire_retreat_current = 0;
					 TRIGGER.tire_retreat_current_back = 0;
				 }
		     break;
		
		case TRIGGER_CAL:   //校准
			   if(TRIGGER.if_cal == 1)
				 {
				   if(TRIGGER.cal_step[0] == 1)
					 {
						 TRIGGER.ecd_total = trigger_motor.total_ecd;
					   TRIGGER.v_trigger = TRIGGER_CAL_SPEED;
					 }
					 else if(TRIGGER.cal_step[1] == 1)
					 {
					   TRIGGER.v_trigger = 0;
						 TRIGGER.ecd_total = trigger_motor.total_ecd + TRIGGER_CAL_OFFSET_ECD;
						 TRIGGER.initial_ecd = trigger_motor.total_ecd + TRIGGER_CAL_OFFSET_ECD;
						 TRIGGER.cal_step[1] = 0;
						 TRIGGER.if_cal = 0;
					 }			 
				 }
				 
				 if(TRIGGER.cal_step[0] == 1 && TRIGGER.cal_step[1] == 0 && TRIGGER.flag_if_flug[2] == 1)  //第一阶段结束判断
				 {
				   TRIGGER.cal_step[0] = 0;
					 TRIGGER.cal_step[1] = 1;
				 } 
			   break;
			
		case TRIGGER_STATIC:   //不动
			   TRIGGER.v_trigger = 0;
//         TRIGGER.ecd_total=motor[TRIGGER_motor].total_ecd;
		     if(TRIGGER.last_trigger_mode == TRIGGER_LONG && TRIGGER.initial_ecd != 0 && trigger_motor.total_ecd != 0)
		      TRIGGER.ecd_total = trigger_motor.total_ecd + TRIGGER_ECD - (int)(trigger_motor.total_ecd - TRIGGER.initial_ecd)%(int)TRIGGER_ECD;
				 TRIGGER.tire_retreat_current = 0;
		     TRIGGER.tire_retreat_current_back = 0;
		     break;
		
		case TRIGGER_IDLE:      // 无力
			   TRIGGER.v_trigger = 0;
         TRIGGER.ecd_total=trigger_motor.total_ecd;
	       TRIGGER.flag_if_back = 0;
	       TRIGGER.flag_if_back_over = 1;
	       TRIGGER.flag_if_single = 0;
	       TRIGGER.flag_if_single_over = 1;
		     break;

		default:
			break;
	}
	TRIGGER.last_trigger_mode = mode.trigger_state;
}

/*******************************************************************摩擦轮其他控制*******************************************************************/
//卡弹计时
void trigger_retreat(void)
{
		if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out >TRIGGER_FORWARD_TORQUE_THRESHOLD )//如果转速低而输出电流大的情况则判断为卡弹(正拨)
		{
				TRIGGER.tire_retreat_current++;
		}
		else if(TRIGGER.tire_retreat_current>0)
		{
			  TRIGGER.tire_retreat_current--;
		}
		
		 if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out < (-TRIGGER_BACK_TORQUE_THRESHOLD ))//如果转速低而输出电流大的情况则判断为卡弹(反拨)
		{
				TRIGGER.tire_retreat_current_back++;
		}
		else if(TRIGGER.tire_retreat_current_back>0)
		{
			  TRIGGER.tire_retreat_current_back--;
		}
		
		if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out < (-TRIGGER_CAL_TORQUE_THRESHOLD ))//如果转速低而输出电流大的情况则判断为卡弹(校准)
		{
				TRIGGER.tire_retreat_current_cal++;
		}
		else if(TRIGGER.tire_retreat_current_cal>0)
		{
			  TRIGGER.tire_retreat_current_cal--;
		}
		

   if(mode.trigger_state != TRIGGER_CAL)
	 {		 
		if(TRIGGER.tire_retreat_current > TRIGGER_FORWARD_TIME_THRESHOLD)   //正转标志位
			 TRIGGER.flag_if_flug[0] = 1;
		else
			 TRIGGER.flag_if_flug[0] = 0;
		
		if(TRIGGER.tire_retreat_current_back > TRIGGER_BACK_TIME_THRESHOLD) //反转标志位
			 TRIGGER.flag_if_flug[1] = 1;
    else
			 TRIGGER.flag_if_flug[1] = 0;
	}
	else
	{
		if(TRIGGER.tire_retreat_current_cal > TRIGGER_CAL_TIME_THRESHOLD)   //校准标志位
			 TRIGGER.flag_if_flug[2] = 1;
    else
			 TRIGGER.flag_if_flug[2] = 0;		
	}
		
}



//热量反馈检测器（反馈裁判系统下次反馈剩余热量变化-仅连发时使用)
int next_heat_change(void)
{
    int next_Shoot_quantity=0;//下次消耗弹数
    int heat_recover=0;
    int heat_consume=0;
    int heat_change=0;//剩余热量变化
    next_Shoot_quantity=(int)(TRIGGER.limit_v/(V_TRIGGER_15)*0.25f+1);//TRIGGER.limit_v为连发转速，0.25f为反馈时差
//    heat_recover=robot_status.shooter_barrel_cooling_value/10;
    heat_consume=next_Shoot_quantity*10;
    heat_change=heat_recover-heat_consume;
    return heat_change;
}


//枪管热量限制
void trigger_heat(void)
{
// int heat_limit=robot_status.shooter_barrel_heat_limit;
// int heat=power_heat_data.shooter_17mm_barrel_heat;
//	
//	if(heat_limit - heat + next_heat_change() < 20)
//		mode.tirgger_state = TIRGGER_STATIC;
//			 
//	 if(robot_status.shooter_barrel_cooling_value == 40 && heat_limit==50)
//	 {
//		TIRGGER.limit_v = V_TIRGGER_10;
//	 }
//	 else
//	 {
//		 if(mode.vision_switch_state == VISION_ARMOR)
//		 {
//			 TIRGGER.limit_v = V_TIRGGER_18;
//		 }
//		   
//		 if(USART_Rx_data.rc_ctrl_key_f == 0&&mode.vision_switch_state == VISION_CLOSE)
//		 {	
	     TRIGGER.limit_v = V_TRIGGER_15;
//		 }
//		 else if(USART_Rx_data.rc_ctrl_key_f == 1&&mode.vision_switch_state == VISION_CLOSE)
//		 {
//		   TRIGGER.limit_v = -V_TRIGGER_20;
//		 }
//	 }
}


