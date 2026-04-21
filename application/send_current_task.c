#include "send_current_task.h"
#include "config.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "mode_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "trigger_task.h"
#include "bsp_transmit.h"
#include "ins_task.h"
#include "math.h"
#include "detect.h"
int time=0;
void send_current_task(void const * argument)
{
  /* USER CODE BEGIN current_task */
  vTaskDelay(30);
  /* Infinite loop */
  for(;;)
  {
		enable_disable_DM4310();
		
		if(communication_state==UART_COMMUNICATION_NORMAL)		
		{
			#ifdef GIMBAL_YAW_SENT
				yaw_ctrl_current();
			#else
					Error_Yaw();
			#endif
			#ifdef GIMBAL_PITCH_SENT
					pitch_ctrl_current();
			#else
					Error_Pitch();
			#endif
					vTaskDelay(1);
					
			#ifdef SHOOT_SEND
					shoot_ctrl_current();
			#else
					Error_Shoot();
			#endif
			
		}
		else
		{
			Error_Yaw();
			Error_Pitch();
			Error_Shoot();
		}
			
		vTaskDelay(1);
  }
  /* USER CODE END current_task */
}


/**
 * @brief 云台yaw电流发送
 * @note
 * @param
 */
void yaw_ctrl_current()
{
	if (mode.gimbal_state != GIMBAL_IDLE) 
		ctrl_motor(&hcan2, 0x05, 0, 0, 0, 0, GIMBAL.output_yaw); 
	else
		ctrl_motor(&hcan2, 0x05, 0, 0, 0, 0, 0);
}




/**
 * @brief yaw云台错误电流发送
 * @note
 * @param
 */int uooo=0;
void Error_Yaw()
{
	ctrl_motor(&hcan2, 0x05, 0, 0, 0, 0, 0);
	
}

/**
 * @brief pitch云台正常电流发送
 * @note
 * @param
 */
void pitch_ctrl_current()
{

	if (mode.gimbal_state != GIMBAL_IDLE)
	{
		ctrl_motor(&hcan1, 0x04, 0, 0, 0, 0, GIMBAL.output_pitch );//
			DM_position_ctrl(&hcan1,0x103,GIMBAL.big_pitch_target,10);
	}		
	else
	{
		ctrl_motor(&hcan1, 0x04, 0, 0, 0, 0, 0);
		DM_position_ctrl(&hcan1,0x103,GIMBAL.big_pitch_target,0);
	}	
	
	if(rc_ctrl.keyboard.key_Q==1)
		damiao_clear(&hcan1,0x103);

}

/**
 * @brief pitch云台错误电流发送
 * @note
 * @param
 */
void Error_Pitch()
{
	ctrl_motor(&hcan1, 0x04, 0, 0, 0, 0, 0);
	DM_position_ctrl(&hcan1,0x103,GIMBAL.big_pitch_target,0);
}

/**
 * @brief 摩擦轮拨弹盘正常电流发送
 * @note
 * @param
 */
void shoot_ctrl_current()
{
	
		if(toe_offline[0].communication_state == COMMUNICATION_NORMAL)
			set_motor_current(&hcan2, 0x1ff,TRIGGER.pid_trigger_out, SHOOT.output[0], SHOOT.output[1], 0);//TRIGGER.output
		else
			set_motor_current(&hcan2, 0x1ff,0, SHOOT.output[0], SHOOT.output[1], 0);	
		
}
/**
 * @brief 摩擦轮拨弹盘错误电流发送
 * @note
 * @param
 */
void Error_Shoot()
{
	set_motor_current(&hcan2, 0x1ff, 0, 0, 0, 0);
}


int yuu1=0,yuu2=0,yuu3=0;


void enable_disable_DM4310(void)
{
//	yuu1++;
//	static int16_t enable_cnt=80;
//	if(mode.gimbal_state != GIMBAL_IDLE&&enable_cnt>0)
//	{
//		yuu2++;
//		damiao_init(&hcan2, 0x05);
//		vTaskDelay(1);
//		damiao_init(&hcan1, 0x03);
//		vTaskDelay(1);
//		damiao_init(&hcan1, 0x04);
//		vTaskDelay(1);	
//		enable_cnt--;
//	}
//	else if(mode.gimbal_state == GIMBAL_IDLE)
//	{
//		yuu3++;
//		enable_cnt=80;
//		damiao_exit(&hcan2, 0x05);
//		vTaskDelay(1);
//		damiao_exit(&hcan1, 0x03);
//		vTaskDelay(1);
//		damiao_exit(&hcan1, 0x04);
//		vTaskDelay(1);
//	}
	if (mode.controls_state==RC_ctrl&&mode.gimbal_state != GIMBAL_IDLE&&(small_pitch.ERR==0||big_pitch.ERR==0))
	{			
			
					damiao_init(&hcan2, 0x05);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x03);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x04);
					vTaskDelay(1);						
	}
	if(mode.controls_state==KEY_ctrl&&toe_offline[0].communication_state == COMMUNICATION_NORMAL&&(small_pitch.ERR==0||big_pitch.ERR==0))
	{
					damiao_init(&hcan2, 0x05);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x03);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x04);
					vTaskDelay(1);						
	}
	if(mode.gimbal_state == GIMBAL_IDLE||toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
			damiao_exit(&hcan2, 0x05);
			vTaskDelay(1);
			damiao_exit(&hcan1, 0x03);
			vTaskDelay(1);
			damiao_exit(&hcan1, 0x04);
			vTaskDelay(1);
	}
}
