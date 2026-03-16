#include "vofa.h"
#include "freertos.h" 
#include "bsp_dwt.h"
#include "task.h"
#include "usart.h"
#include "gimbal_task.h"
#include "mode_task.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "vision.h"
#include "trigger_task.h"
#include "bsp_transmit.h"
#include "remote_control.h"
Vofa_data_m_2 Vofa_data_2={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_4 Vofa_data_4={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_8 Vofa_data_8={.tail={0x00,0x00,0x80,0x7f}};

extern pid_type_def pid_yaw_vision_armor_angle;
extern pid_type_def pid_yaw_vision_armor_speed;

extern pid_type_def pid_yaw_angle_pos;
extern pid_type_def pid_yaw_speed_pos;
extern pid_type_def pid_pitch_angle;
extern pid_type_def pid_pitch_speed;
extern pid_type_def pid_yaw_angle;
extern pid_type_def pid_yaw_speed;
extern pid_type_def pid_pitch_vision_armor_angle;
extern pid_type_def pid_pitch_vision_armor_speed;
extern feedforward_control_t yaw_vision_forward;
extern float qqq11;
extern feedforward_control_t yaw_vision_speed_forward;
extern int a1,b1,c1,d1,temp_angle_pid;
extern float accel_kalman,angel_accle;//差分获得的角加速度
extern pid_type_def pid_yaw_angle_Recognition, pid_yaw_speed_Recognition;
extern pid_type_def pid_pitch_angle;
extern pid_type_def pid_pitch_speed;

void StartVOFATask(void const * argument)
{
  for(;;)
  {
		//Vofa_Send_Data8(VisionToGimbal.yaw.d*57.3f,INS.Yaw,yaw_vision_forward.output,INS.Pitch ,VisionToGimbal.pitch.d*57.3f,GIMBAL.output_yaw,VisionToGimbal.mode,VisionToGimbal.yaw_vel.d);
		//Vofa_Send_Data8(Vision_Rx.yaw,INS.Yaw,Vision_Rx.pitch,INS.Pitch,VisionToGimbal.yaw_vel.d,VisionToGimbal.mode,VisionToGimbal.yaw_acc.d, GIMBAL.output_yaw);
		//Vofa_Send_Data8(GIMBAL.IF_DT_OVER,GIMBAL.IF_DT,mode.gimbal_state,mode.chassis_state,a1,GIMBAL.IF_FOLD_OVER,GIMBAL.IF_DT,GIMBAL.yaw_target);
		//Vofa_Send_Data8(VisionToGimbal.yaw.d*57.3f,INS.Yaw,VisionToGimbal.mode,pid_yaw_speed_Recognition.set,pid_yaw_speed_Recognition.ref,0,0,0);
		//Vofa_Send_Data8(small_yaw._torq,accel_kalman,INS.Gyro[2],angel_accle,0,0,0,0);
		//Vofa_Send_Data8(small_yaw._torq,VisionToGimbal.yaw_acc.d,VisionToGimbal.yaw_vel.d,0,0,0,0,0);
		//Vofa_Send_Data8(TRIGGER.if_back_flag,TRIGGER.back_over_flag,TRIGGER.err_cnt,TRIGGER.once_target_ecd,0,0,0,0);
		//Vofa_Send_Data8(Vision_Rx.yaw,INS.Yaw,Vision_Rx.v_yaw,Vision_Rx.enable_yaw_diff,Vision_Rx.appear,IF_FIRE(),IF_DISCERN(),mode.trigger_state);
Vofa_Send_Data8(Vision_Rx.yaw,INS.Yaw,Vision_Rx.pitch,INS.Pitch,Vision_Rx.a_yaw,IF_FIRE(),Vision_Rx.enable_pitch_diff,Vision_Rx.enable_yaw_diff);
		//Vofa_Send_Data8(mode.gimbal_state,mode.vision_switch_state,mode.trigger_state,0,0,0,0,0);
		//Vofa_Send_Data8(USART_Rx_data.chassis_power_limit,USART_Rx_data.real_power,USART_Rx_data.buffer_energy,USART_Rx_data.cap_v,USART_Rx_data.data[5],USART_Rx_data.data[0],USART_Rx_data.data[1],USART_Rx_data.data[2]);
		//Vofa_Send_Data8(USART_Rx_data.shooter_barrel_heat_limit,USART_Rx_data.shooter_barrel_cooling_value,USART_Rx_data.shooter_17mm_1_barrel_heat,USART_Rx_data.chassis_speed_rpm,0,0,0,0);
		//Vofa_Send_Data8(USART_Rx_data.real_power,USART_Rx_data.chassis_given_current,USART_Rx_data.chassis_speed_rpm,USART_Rx_data.speed_out,USART_Rx_data.data[1],USART_Rx_data.data[2],USART_Rx_data.data[3],USART_Rx_data.data[4]);
		//Vofa_Send_Data8(big_pitch.ERR,0,0,0,0,0,0,0);
		//Vofa_Send_Data8(pid_pitch_angle.set,pid_pitch_angle.ref,pid_pitch_angle.out,pid_pitch_speed.ref,pid_pitch_speed.out,INS.Pitch,small_pitch._torq,0);
		vTaskDelay(10);
  }
}

void Vofa_Send_Data2(float data1, float data2)
{
    Vofa_data_2.ch_data[0] = data1;
    Vofa_data_2.ch_data[1] = data2;
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&Vofa_data_2, sizeof(Vofa_data_2));   
//    CDC_Transmit_FS((uint8_t *)&Vofa_data_2,sizeof(Vofa_data_2)); 
}

void Vofa_Send_Data4(float data1, float data2,float data3, float data4)
{
    Vofa_data_4.ch_data[0] = data1;
    Vofa_data_4.ch_data[1] = data2;
    Vofa_data_4.ch_data[2] = data3;
    Vofa_data_4.ch_data[3] = data4;
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&Vofa_data_4, sizeof(Vofa_data_4));   
}

void Vofa_Send_Data8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8)
{ 
    Vofa_data_8.ch_data[0] = data1;
    Vofa_data_8.ch_data[1] = data2;
    Vofa_data_8.ch_data[2] = data3;
    Vofa_data_8.ch_data[3] = data4;
    Vofa_data_8.ch_data[4] = data5;
    Vofa_data_8.ch_data[5] = data6;
    Vofa_data_8.ch_data[6] = data7;
    Vofa_data_8.ch_data[7] = data8;
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&Vofa_data_8, sizeof(Vofa_data_8));   
}
