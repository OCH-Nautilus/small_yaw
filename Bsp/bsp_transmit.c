#include "bsp_transmit.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "detect.h"
#include "mode_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
USART_Rx_data_t USART_Rx_data;
USART_TX_data_t  USART_TX_data;

uint8_t USART_Rx_data_handle[DATA_COUNT];
uint8_t USART_Tx_buff[USART_DATA_COUNT] = {0};

/**
  * @Name    Transmit_Data_Task
  * @brief   串口数据发送任务
  * @param   None
  * @Data    2024-03-14
*/
int j=0;
void Transmit_Data_Task(void const *pvParameters)
{
	vTaskDelay(RECIVE_TASK_INIT_TIME);
	
	USART_Data_init(&USART_TX_data);
	for(;;)
	{ 
		j++;
		USART_Data_Send(&USART_TX_data,USART_Tx_buff);
		
		vTaskDelay(RECIVE_TASK_TIME_MS);
	}
	
}
/**
  * @Name    USART_Data_init
  * @brief   串口数据初始化
  * @param   data_init: [输入/出] 
  * @Data    2024-03-14
*/

void USART_Data_init(USART_TX_data_t *data_init)
{
		 if (data_init == NULL)
    {
        return;
    }
		 memset(data_init, 0, sizeof(USART_TX_data_t));
		 data_init->mode.bits.DT_OVER_FLAG=1;
}

/**
  * @Name    USART_Data_Handle
  * @brief   串口发送数据处理
  * @param   data: [输入/出] 
  * @Data    2024-03-14
*/
void USART_Data_Handle(USART_TX_data_t *data)
{
    if (data == NULL)
    {
        return;
    }
    
    
    // 设置帧头
    data->head = USART_TX_HEAD;
    
    // 设置模式位域 - 只取低2位确保不越界
    data->mode.bits.controls_mode = mode.controls_state & 0x03;
    data->mode.bits.gimbal_mode = mode.gimbal_state & 0x03;
    data->mode.bits.vision_mode = mode.vision_switch_state & 0x03;
    data->mode.bits.shoot_mode = mode.shoot_state & 0x03;
    data->mode.bits.trigger_mode = mode.trigger_state & 0x03;
    data->mode.bits.chassis_mode = mode.chassis_state & 0x03;
    data->mode.bits.chassis_speed_mode = mode.chassis_speed_state & 0x03;
    data->mode.bits.IF_DT_FLAG = GIMBAL.IF_DT&0x01;
		data->mode.bits.DT_OVER_FLAG=GIMBAL.IF_DT_OVER&0x01;
    // 设置遥控器数据
    data->rc_ctrl_r_x = rc_ctrl.rc.ch[0];
    data->rc_ctrl_r_y = rc_ctrl.rc.ch[1];
    data->rc_ctrl_l_x = rc_ctrl.rc.ch[2];
    data->rc_ctrl_l_y = rc_ctrl.rc.ch[3];
    
    // 设置云台和传感器数据
    data->small_yaw_pos = small_yaw._pos;
    data->yaw = INS.Yaw;
    
    // 设置鼠标数据
    data->mouse_vx = rc_ctrl.mouse.vx;
    data->mouse_vy = rc_ctrl.mouse.vy;
    
    // 设置按键位域 - 确保值为0或1
    data->key.bits.Key_W = rc_ctrl.keyboard.key_W ? 1 : 0;
    data->key.bits.Key_S = rc_ctrl.keyboard.key_S ? 1 : 0;
    data->key.bits.Key_A = rc_ctrl.keyboard.key_A ? 1 : 0;
    data->key.bits.Key_D = rc_ctrl.keyboard.key_D ? 1 : 0;
    data->key.bits.Key_Shift = rc_ctrl.keyboard.key_Shift ? 1 : 0;
		data->key.bits.Key_Flag_E = rc_ctrl.keyboard.flag_E ? 1 : 0;
		data->key.bits.Key_E = rc_ctrl.keyboard.key_E ? 1 : 0;
		
		data->rc_ctrl_s.bits.s_l=rc_ctrl.rc.s[0];
		data->rc_ctrl_s.bits.s_r=rc_ctrl.rc.s[1];
    // 设置帧尾
    data->tail = USART_TX_END;
}




/**
  * @Name    USART_Data_Send
  * @brief   串口数据发送
  * @param   data: [输入/出] 
**			 buff: [输入/出] 
  * @Data    2024-03-14
*/
void USART_Data_Send(USART_TX_data_t *data, uint8_t *buff)
{
    if (data == NULL)
    {
        return;
    }
    
    USART_Data_Handle(data);
		
    memcpy(buff + 0, &data->head, 1);
    memcpy(buff + 1, &data->mode.mode_pack, 2);
    memcpy(buff + 3, &data->rc_ctrl_r_x, 2);
    memcpy(buff + 5, &data->rc_ctrl_r_y, 2);
    memcpy(buff + 7, &data->rc_ctrl_l_x, 2);
    memcpy(buff + 9, &data->rc_ctrl_l_y, 2);
    memcpy(buff + 11, &data->small_yaw_pos, 4);
    memcpy(buff + 15, &data->yaw, 4);
    memcpy(buff + 19, &data->mouse_vx, 4);
    memcpy(buff + 23, &data->mouse_vy, 4);
    memcpy(buff + 27, &data->key.key_pack, 1);
		memcpy(buff + 28, &data->rc_ctrl_s.rc_s_pack, 1);
    memcpy(buff + 29, &data->tail, 1);
		
		
    HAL_UART_Transmit_DMA(&huart1, buff, USART_DATA_COUNT);
}



/**
  * @Name    Head1_data_Handle
  * @brief   
  * @param   buff: [输入/出] 
  * @Data    2024-01-31
*/
int asss=0;
void Head1_data_Handle(uint8_t *buff,USART_Rx_data_t *data)
{
	if(buff[0] == USART_RX_HAED && buff[DATA_COUNT-1] == USART_RX_END)
	{asss++;
		Algorithm_fp32_u diff_angle;
		Algorithm_fp32_u initial_speed;
		Algorithm_fp32_u ins_big_yaw;
		Algorithm_fp32_u big_yaw_target;
		Algorithm_int16_u shooter_barrel_heat_limit;
		Algorithm_int16_u shooter_barrel_cooling_value;
		Algorithm_int16_u shooter_17mm_1_barrel_heat;
		Algorithm_int16_u chassis_power_limit;
		Algorithm_fp32_u real_power;
		Algorithm_int16_u buffer_energy;
		for(int i=0;i<4;i++)
		{
			diff_angle.d[i]=buff[i+1];
			initial_speed.d[i]=buff[i+7];
			ins_big_yaw.d[i]=buff[i+11];
			big_yaw_target.d[i]=buff[i+15];
			shooter_barrel_heat_limit.d[i]=buff[i+19];
			shooter_barrel_cooling_value.d[i]=buff[i+23];
			shooter_17mm_1_barrel_heat.d[i]=buff[i+27];
			chassis_power_limit.d[i]=buff[i+31];
			real_power.d[i]=buff[i+35];
			buffer_energy.d[i]=buff[i+39];
		}
		data->chassis_diff_angle=diff_angle.data;
		data->trigger_back_over_flag=buff[5];
		data->trigger_weak_flag=buff[6];
		data->initial_speed=initial_speed.data;
		data->ins_big_yaw=ins_big_yaw.data;
		data->big_yaw_target=big_yaw_target.data;
		data->shooter_17mm_1_barrel_heat=shooter_17mm_1_barrel_heat.data;
		data->shooter_barrel_heat_limit=shooter_barrel_heat_limit.data;
		data->shooter_barrel_cooling_value=shooter_barrel_cooling_value.data;
		data->chassis_power_limit=chassis_power_limit.data;
		data->real_power=real_power.data;
		data->buffer_energy=buffer_energy.data;
	}

}


