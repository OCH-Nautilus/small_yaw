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
int time=0;
void send_current_task(void const * argument)
{
  /* USER CODE BEGIN current_task */
  vTaskDelay(30);
  /* Infinite loop */
  for(;;)
  {
		enable_disable_DM4310();
		
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
		ctrl_motor(&hcan1, 0x04, 0, 0, 0, 0, GIMBAL.output_pitch); 
			DM_position_ctrl(&hcan1,0x103,GIMBAL.big_pitch_target,10);
	}		
	else
	{
		ctrl_motor(&hcan1, 0x04, 0, 0, 0, 0, 0);
		DM_position_ctrl(&hcan1,0x103,GIMBAL.big_pitch_target,0);
	}	
	
	

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
	set_motor_current(&hcan2, 0x1ff,TRIGGER.output, SHOOT.output[0], SHOOT.output[1], 0);
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





void enable_disable_DM4310(void)
{
	static int16_t dm_cnt=30;
	if (mode.gimbal_state != GIMBAL_IDLE)
	{			
			if (dm_cnt > 0)
			{
					damiao_init(&hcan2, 0x05);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x03);
					vTaskDelay(1);
					damiao_init(&hcan1, 0x04);
					dm_cnt--;
					vTaskDelay(1);
			}
			
	}
	else
	{
			damiao_exit(&hcan2, 0x05);
			damiao_exit(&hcan1, 0x03);
			damiao_exit(&hcan1, 0x04);
			dm_cnt=30;
			vTaskDelay(1);
	}
}
