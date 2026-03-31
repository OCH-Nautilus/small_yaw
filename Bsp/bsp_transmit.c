#include "bsp_transmit.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "detect.h"
#include "mode_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "vision.h"
#include "shoot_task.h"
#include "trigger_task.h"
USART_Rx_data_t USART_Rx_data;
USART_TX_data_t  USART_TX_data;

uint8_t USART_Rx_data_handle[DATA_COUNT];
uint8_t USART_Tx_buff[USART_DATA_COUNT] = {0};
communication_state_e communication_state;
extern uint8_t Rotate_direction;//小陀螺旋转方向
extern uint8_t top_mode;
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
	communication_state=UART_COMMUNICATION_NORMAL;
	
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
		data->key.bits.Key_G=	rc_ctrl.keyboard.key_G ? 1 : 0;
		data->rc_ctrl_s.bits.s_l=rc_ctrl.rc.s[0];
		data->rc_ctrl_s.bits.s_r=rc_ctrl.rc.s[1];
		// 设置通信时长
		data->Communication_count=HAL_GetTick();
		//设置标志位
		data->flag.bits.IF_DISCERN=IF_DISCERN();
		data->flag.bits.shoot_l=shoot_l_detect();
		data->flag.bits.shoot_r=shoot_r_detect();
		data->flag.bits.stuck_state=stuck_state;
		data->flag.bits.down_over_flag=GIMBAL.down_over;
		data->flag.bits.rotate_direction=Rotate_direction;
		data->flag.bits.top_mode=top_mode;
		data->flag.bits.detect_flag=toe_offline[0].communication_state;
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
		memcpy(buff + 29, &data->Communication_count, 4);
		memcpy(buff + 33, &data->flag.flag_pack, 2);
    memcpy(buff + 35, &data->tail, 1);
		
		
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
	
	static uint32_t last_Communication_count=0;
	static uint16_t err_cnt=0;
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
		Algorithm_fp32_u cap_v;
		Algorithm_int32_u Communication_count;
		Algorithm_int16 speed_out;
		Algorithm_int16 chassis_given_current;
		Algorithm_int16 chassis_speed_rpm;

		for(int i=0;i<4;i++)
		{
			diff_angle.d[i]=buff[i+1];
			initial_speed.d[i]=buff[i+7];
			ins_big_yaw.d[i]=buff[i+11];
			big_yaw_target.d[i]=buff[i+15];
			real_power.d[i]=buff[i+27];			
			cap_v.d[i]=buff[i+33];
			Communication_count.d[i]=buff[i+37];
		}
		for(int i=0;i<2;i++)
		{
			shooter_barrel_heat_limit.d[i]=buff[i+19];
			shooter_barrel_cooling_value.d[i]=buff[i+21];
			shooter_17mm_1_barrel_heat.d[i]=buff[i+23];
			chassis_power_limit.d[i]=buff[i+25];
			buffer_energy.d[i]=buff[i+31];
			speed_out.d[i]=buff[i+41];
			chassis_given_current.d[i]=buff[i+43];
			chassis_speed_rpm.d[i]=buff[i+45];
		}
		data->chassis_diff_angle=diff_angle.data;
		data->chassis_if_blackout=buff[5];
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
		data->cap_v=cap_v.data;
		data->Communication_count=Communication_count.data;
		data->speed_out=speed_out.data;
		data->chassis_given_current=chassis_given_current.data;
		data->chassis_speed_rpm=chassis_speed_rpm.data;
		data->vision_color=buff[47];
	}

	if(last_Communication_count==data->Communication_count)
			err_cnt++;
	else 
		err_cnt=0;
	
	if(err_cnt>=100)
		communication_state=UART_COMMUNICATION_ERR;
	else
		communication_state=UART_COMMUNICATION_NORMAL;

	last_Communication_count=data->Communication_count;
}


