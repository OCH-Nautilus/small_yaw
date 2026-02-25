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
		Vofa_Send_Data8(Vision_Rx.yaw,INS.Yaw,Vision_Rx.a_yaw,pid_yaw_vision_armor_angle.out+yaw_vision_forward.output,pid_yaw_vision_armor_angle.out,INS.Gyro[2],GIMBAL.output_yaw,Vision_Rx.enable_yaw_diff);
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
