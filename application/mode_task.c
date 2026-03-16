#include "mode_task.h"
#include "config.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "detect.h"
#include "gimbal_task.h"
#include "vision.h"
#include "ins_task.h"
#include "bsp_transmit.h"
#include "trigger_task.h"
mode_t mode;
uint16_t chassis_last_mode=0;
uint16_t chassis_now_mode=0;
int16_t Rotate_direction=1;
/**
 * @brief 主控模式切换
 * @note  滚轮上切换为键鼠控制，滚轮下切换为遥控器控制
 * @param
 */
void mode_task(void const * argument)
{
  /* USER CODE BEGIN mode_task */
     mode_init();
	vTaskDelay(10);
  /* Infinite loop */
  for(;;)
  {
		system_conctrl();
		if(mode.controls_state==RC_ctrl)
		{
			chassis_rc_ctrl();
			gimbal_rc_ctrl();
			vision_rc_ctrl();
			shoot_rc_ctrl();
			tirgger_rc_ctrl();
		}
		else if(mode.controls_state==KEY_ctrl)
		{
			chassis_pc_ctrl();
			gimbal_pc_ctrl();
			vision_pc_ctrl();
			shoot_pc_ctrl();
			tirgger_pc_ctrl();
		}
		else
		{
			chassis_rc_ctrl();
			gimbal_rc_ctrl();
			vision_rc_ctrl();
			shoot_rc_ctrl();
			tirgger_rc_ctrl();
		}
    vTaskDelay(1);
  }
  /* USER CODE END mode_task */
}

/**
 * @brief 主控模式初始化
 * @note  
 * @param
 */

void mode_init()
{
    mode.controls_state=RC_ctrl;
    mode.chassis_speed_state=SPEED_NORMAL;
    mode.chassis_state=CHASSIS_IDLE;
    mode.gimbal_state=GIMBAL_IDLE;
    mode.shoot_state=SHOOT_IDLE;
    mode.trigger_state=TRIGGER_IDLE;
    mode.vision_switch_state=VISION_CLOSE;
		toe_offline[0].communication_state = COMMUNICATION_NONE;
}

/**
 * @brief 控制模式切换
 * @note
 * @param
 */
void system_conctrl()
{
    if (rc_ctrl.rc.wheel >= 600)
        mode.controls_state=RC_ctrl;
    else if (rc_ctrl.rc.wheel <= -600)
        mode.controls_state=KEY_ctrl;
    if(mode.gimbal_state==GIMBAL_IDLE)
        mode.chassis_state=CHASSIS_IDLE;
    switch (mode.controls_state)
    {
    case RC_ctrl:
        if (rc_ctrl.rc.wheel <= -600)
		{
			mode.controls_state = KEY_ctrl;
			mode.chassis_state = CHASSIS_IDLE;
			mode.shoot_state = SHOOT_IDLE;
			mode.gimbal_state = GIMBAL_IDLE;
			mode.trigger_state =TRIGGER_IDLE;
			mode.vision_switch_state = VISION_CLOSE;
			
		}
        break;
    case KEY_ctrl:
        if (rc_ctrl.rc.wheel >= 600)
		{
			mode.controls_state = RC_ctrl;
			mode.controls_state = KEY_ctrl;
			mode.chassis_state = CHASSIS_IDLE;
			mode.shoot_state = SHOOT_IDLE;
			mode.gimbal_state = GIMBAL_IDLE;
			mode.trigger_state =TRIGGER_IDLE;
			mode.vision_switch_state = VISION_CLOSE;
		}
        break;
    default:
        break;
    }
}

/**
 * @brief 底盘模式切换
 * @note  优化陀螺模式下掉头切换跟随
 * @param
 */
void chassis_mode_change()
{

	chassis_now_mode = mode.chassis_state;

	
	if (chassis_last_mode == CHASSIS_TOP && chassis_now_mode == CHASSIS_FOLLOW)
	{
		if (Rotate_direction == 1)
		{
			if (USART_Rx_data.chassis_diff_angle < -0.6f || USART_Rx_data.chassis_diff_angle > 0.0f)
				mode.chassis_state = CHASSIS_TOP;
			else
			{
				mode.chassis_state = CHASSIS_FOLLOW;
				Rotate_direction = Rotate_direction * (-1);
			}
		}
		else if (Rotate_direction == -1)
		{
			if (USART_Rx_data.chassis_diff_angle > 0.6f || USART_Rx_data.chassis_diff_angle < 0.0f)
				 mode.chassis_state = CHASSIS_TOP;
			else
			{
				 mode.chassis_state = CHASSIS_FOLLOW;
				Rotate_direction = Rotate_direction * (-1);
			}
		}
	}
	chassis_last_mode =  mode.chassis_state;
}

/**
 * @brief 遥控器控制底盘
 * @note
 * @param
 */
void chassis_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE || rc_ctrl.rc.s[0] == 2)
		mode.chassis_state = CHASSIS_IDLE; // 无力
	if (mode.gimbal_state != GIMBAL_IDLE)
	{
		switch (rc_ctrl.rc.s[0])
		{
		case 1: // 陀螺
			mode.chassis_state = CHASSIS_TOP;//mode.chassis_state = CHASSIS_TOP;
			break;
		case 3: // 跟随
			mode.chassis_state = CHASSIS_FOLLOW;
			break;
		case 2:
			mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		default:
			mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		}
	}
	else
		mode.chassis_state = CHASSIS_IDLE; // 无力

	chassis_mode_change();
}

/**
 * @brief 遥控器控制云台
 * @note
 * @param
 */

void gimbal_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.gimbal_state = GIMBAL_IDLE;
	}
	if (toe_offline[0].communication_state == COMMUNICATION_NORMAL)
	{//未进入视觉模式下，左拨杆最上是折叠模式，中下是正常模式
		if (rc_ctrl.rc.s[1] != 1)
		{
			if(rc_ctrl.rc.s[0]!=1)
				mode.gimbal_state = GIMBAL_NORMAL;
			else 
				mode.gimbal_state = GIMBAL_FOLD;//GIMBAL_FOLD
		}
		else
			mode.gimbal_state = GIMBAL_VISION;
		//直接从折叠进入视觉
		if(rc_ctrl.rc.s[1]==1&&GIMBAL.last_mode == GIMBAL_FOLD)
		{
			if(GIMBAL.rise_over==0)
				mode.gimbal_state = GIMBAL_NORMAL;
			else
				mode.gimbal_state = GIMBAL_VISION;
		}

		//平头
		if (GIMBAL.last_mode == GIMBAL_IDLE && mode.gimbal_state != GIMBAL_IDLE)
		{
			GIMBAL.PT_flag = 1;
		}
	}

	GIMBAL.last_mode = mode.gimbal_state;
}

/**
 * @brief 遥控器控制视觉
 * @note
 * @param
 */
void vision_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.vision_switch_state = VISION_CLOSE;
	}
	else
	{
		if (rc_ctrl.rc.s[0] == 3)
			mode.vision_switch_state = VISION_SMALL_BUFF;
		else if (rc_ctrl.rc.s[0] == 1)
			mode.vision_switch_state = VISION_BIG_BUFF;
		else if (rc_ctrl.rc.s[0] == 2)
			mode.vision_switch_state = VISION_ARMOR;
	}
}

/**
 * @brief 遥控器控制摩擦轮
 * @note
 * @param
 */
void shoot_rc_ctrl()
{
	if (rc_ctrl.rc.s[1] == 2 || toe_offline[0].communication_state == COMMUNICATION_NONE||mode.gimbal_state!=GIMBAL_FOLD)
		mode.shoot_state=SHOOT_IDLE;
	if (rc_ctrl.rc.s[1] != 2 && toe_offline[0].communication_state == COMMUNICATION_NORMAL)
		mode.shoot_state=SHOOT_OPEN;
}

/**
 * @brief 遥控器控制拨盘
 * @note
 * @param
 */
void tirgger_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NORMAL)
	{
		if (mode.gimbal_state == GIMBAL_NORMAL&&TRIGGER.back_over_flag) 
		{
			if (TRIGGER.weak_flag == 0 && rc_ctrl.rc.s[1] != 2&&mode.shoot_state==SHOOT_OPEN) // 未超热量
			{
				if (rc_ctrl.rc.WHEEL_State == DOWN_SHORT)
					mode.trigger_state = TRIGGER_SINGLE;
				else if (rc_ctrl.rc.WHEEL_State == DOWN_LONG)
					mode.trigger_state = TRIGGER_LONG;
				else
					mode.trigger_state =TRIGGER_IDLE;
			}
			else
				mode.trigger_state =TRIGGER_IDLE;
		} 
		else
			mode.trigger_state =TRIGGER_IDLE;
		
		//视觉下拨弹控制
//		if (mode.gimbal_state == GIMBAL_VISION&&mode.shoot_state==SHOOT_OPEN&&TRIGGER.back_over_flag)
//		{
//			switch(VisionToGimbal.mode)//同济
//			{
//				case 0:
//					mode.trigger_state =TRIGGER_IDLE;
//				break;
//				case 1:
//					mode.trigger_state =TRIGGER_IDLE;
//				break;
//				case 2:
//					if (TRIGGER.weak_flag == 0) 
//					{							
//						if (mode.vision_switch_state == VISION_ARMOR)
//							mode.trigger_state = TRIGGER_LONG;//TRIGGER_LONG
//						if (mode.vision_switch_state == VISION_SMALL_BUFF || mode.vision_switch_state == VISION_BIG_BUFF)
//							mode.trigger_state = TRIGGER_SINGLE;
//					} 
//					else						  
//						mode.trigger_state =TRIGGER_IDLE;
//				break;
//				default :
//				break;
//			}
//			
//		}
		
		if (mode.gimbal_state == GIMBAL_VISION&&mode.shoot_state==SHOOT_OPEN&&TRIGGER.back_over_flag)
		{
			if(TRIGGER.weak_flag == 0&&IF_DISCERN()&&IF_FIRE()&&mode.vision_switch_state==VISION_ARMOR&&rc_ctrl.rc.WHEEL_State == DOWN_LONG)//武科
			{
				mode.trigger_state = TRIGGER_SINGLE;
			}
			else if(TRIGGER.weak_flag == 0&&IF_DISCERN()&&(mode.vision_switch_state==VISION_SMALL_BUFF||mode.vision_switch_state==VISION_BIG_BUFF)&&rc_ctrl.rc.WHEEL_State == DOWN_SHORT)
			{
				mode.trigger_state = TRIGGER_SINGLE;
			}
			else 
				mode.trigger_state =TRIGGER_IDLE;
		}
		
	}
	else
		mode.trigger_state =TRIGGER_IDLE;
}



 /**
 * @brief 键盘控制底盘
 * @note
 * @param
 */int a1,b1,c1,d1;
void chassis_pc_ctrl()
{a1++;
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.chassis_state = CHASSIS_IDLE; // 无力
		b1++;
	}
	else
	{
		if (mode.gimbal_state!=GIMBAL_IDLE)
		{
			if(GIMBAL.IF_DT_OVER==0)//处在掉头
			{
				if(rc_ctrl.keyboard.key_CTRL)
					mode.chassis_state=CHASSIS_TOP;
				else
				{
					mode.chassis_state=CHASSIS_IDLE;
					c1++;
				}
			}
			else
			{
				if(rc_ctrl.keyboard.key_CTRL)
					mode.chassis_state=CHASSIS_TOP;
				else
					mode.chassis_state=CHASSIS_FOLLOW;
			}
		}
		else
		{
			mode.chassis_state=CHASSIS_IDLE;
			d1++;
		}
			
	}

	chassis_mode_change();
}

/**
 * @brief 键盘控制云台
 * @note
 * @param
 */
void gimbal_pc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
		mode.gimbal_state = GIMBAL_IDLE;
	else
	{
		if (rc_ctrl.mouse.press_r == 0)//不处于视觉模式
		{
			if (rc_ctrl.keyboard.flag_X == 0)
				mode.gimbal_state = GIMBAL_NORMAL;
			else
				mode.gimbal_state = GIMBAL_FOLD;
		}
		else if (GIMBAL.IF_DT_OVER == 1&&mode.gimbal_state != GIMBAL_FOLD&&GIMBAL.rise_over==1)
			mode.gimbal_state = GIMBAL_VISION;
	}

	

	if (rc_ctrl.keyboard.key_B == 1)
		GIMBAL.PT_flag = 1;
	else
		GIMBAL.PT_flag = 0;


	
	if (rc_ctrl.keyboard.key_R == 1 && GIMBAL.IF_DT_OVER == 1 && (mode.gimbal_state == GIMBAL_NORMAL||mode.gimbal_state == GIMBAL_FOLD)) // 调头——键鼠
	{
		GIMBAL.IF_DT = 1;
		GIMBAL.IF_DT_OVER = 0;
	}


	GIMBAL.last_mode = mode.gimbal_state;
}

/**
 * @brief 键鼠控制视觉
 * @note
 * @param
 */
void vision_pc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.vision_switch_state = VISION_CLOSE;
	}
	else
	{
		if (mode.gimbal_state == GIMBAL_VISION)
		{
			if (rc_ctrl.mouse.press_l == 1)
				mode.vision_switch_state = VISION_ARMOR;
			else if (rc_ctrl.keyboard.key_V)
				mode.vision_switch_state = VISION_SMALL_BUFF;
			else if (rc_ctrl.keyboard.key_Z)
				mode.vision_switch_state = VISION_BIG_BUFF;
			else
				mode.vision_switch_state = VISION_CLOSE;
		}
	}
}

/**
 * @brief 键鼠控制摩擦轮
 * @note
 * @param
 */
void shoot_pc_ctrl()
{
	if(rc_ctrl.keyboard.flag_C==1&&toe_offline[0].communication_state == COMMUNICATION_NORMAL&&mode.gimbal_state!=GIMBAL_FOLD)
		 mode.shoot_state=SHOOT_OPEN;
	else
		 mode.shoot_state=SHOOT_IDLE;
}

/**
 * @brief 键鼠控制拨盘
 * @note
 * @param
 */
void tirgger_pc_ctrl()
{

	if (toe_offline[0].communication_state == COMMUNICATION_NORMAL && mode.shoot_state==SHOOT_OPEN)
	{
		if (mode.gimbal_state == GIMBAL_NORMAL)//&&TRIGGER.back_over_flag
		{
			if (TRIGGER.weak_flag == 0)
			{
				if (rc_ctrl.mouse.KEY_L_State == PUSH_SHORT)
					mode.trigger_state = TRIGGER_SINGLE;
				else if (rc_ctrl.mouse.KEY_L_State == PUSH_LONG)
					mode.trigger_state = TRIGGER_LONG;
				else
					mode.trigger_state =TRIGGER_IDLE;
			}
			else
				mode.trigger_state =TRIGGER_IDLE;
		}
	}
	else
		mode.trigger_state =TRIGGER_IDLE;

//	if (mode.gimbal_state == GIMBAL_VISION && mode.shoot_state==SHOOT_OPEN)//&&TRIGGER.back_over_flag
//	{
//		switch(VisionToGimbal.mode)
//			{
//				case 0:
//					mode.trigger_state =TRIGGER_IDLE;
//				break;
//				case 1:
//					mode.trigger_state =TRIGGER_IDLE;
//				break;
//				case 2:
//				
//					if (TRIGGER.weak_flag == 0) 
//					{							
//						if (mode.vision_switch_state == VISION_ARMOR)
//							mode.trigger_state = TRIGGER_LONG;
//						if (mode.vision_switch_state == VISION_SMALL_BUFF || mode.vision_switch_state == VISION_BIG_BUFF)
//							mode.trigger_state = TRIGGER_SINGLE;
//					} 
//					else						  
//						mode.trigger_state =TRIGGER_IDLE;
//								
//				break;
//				default :
//				break;
//			}
//	}
		if (mode.gimbal_state == GIMBAL_VISION&&mode.shoot_state==SHOOT_OPEN&&TRIGGER.back_over_flag)
		{
			if(TRIGGER.weak_flag == 0&&IF_DISCERN()&&IF_FIRE()&&mode.vision_switch_state==VISION_ARMOR)//武科
			{
				mode.trigger_state = TRIGGER_LONG;
			}
			else if(TRIGGER.weak_flag == 0&&IF_DISCERN()&&(mode.vision_switch_state==VISION_SMALL_BUFF||mode.vision_switch_state==VISION_BIG_BUFF)&&rc_ctrl.mouse.KEY_L_State == PUSH_SHORT)
			{
				mode.trigger_state = TRIGGER_SINGLE;
			}
			else 
				mode.trigger_state =TRIGGER_IDLE;
		}

}


/**
 * @brief 云台模式切换数据重置
 * @note
 * @param
 */
void Vision_to_Normal_init()
{
	if (GIMBAL.last_mode == GIMBAL_VISION &&mode.gimbal_state != GIMBAL_VISION)
	{
		GIMBAL.yaw_target = INS.YawTotalAngle;
		GIMBAL.pitch_target = INS.Pitch;
		mode.trigger_state =TRIGGER_IDLE;
		
		GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
	}

	GIMBAL.last_mode = mode.gimbal_state;
}

void Normal_to_Vision_init()
{
	if (GIMBAL.last_mode == GIMBAL_NORMAL && mode.gimbal_state == GIMBAL_VISION)
	{
		GIMBAL.pitch_target = INS.Pitch;
		GIMBAL.yaw_target = INS.YawTotalAngle;
	}
	GIMBAL.last_mode = mode.gimbal_state;
}
// 底盘加速度
void chassis_speed(void)
{
	if (rc_ctrl.keyboard.key_Shift)
	{
		mode.chassis_speed_state = SPEED_SHIFT;
	}
	else if (rc_ctrl.keyboard.flag_E)
	{
		mode.chassis_speed_state = SPEED_FLY;
	}
	else
	{
		mode.chassis_speed_state = SPEED_NORMAL;
	}
}



