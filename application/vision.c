#include "vision.h"
#include "cmsis_os.h"
#include "mode_task.h"
#include "ins_task.h"
#include "gimbal_task.h"
#include "usbd_cdc_if.h"
#include "bsp_dwt.h"
#include "string.h"
#include "pid.h"
#include "CRC8_CRC16.h"
#include "config.h"
uint8_t buf[43] = {0};
uint32_t vision_time = 0;
uint16_t TX_OK_FLAG=0;

uint8_t vision_buff[55]={0};
GimbalToVision_t GimbalToVision;
VisionToGimbal_t VisionToGimbal;

ReceiveAimINFO Vision_Rx;//视觉发送回来的
SendRobotCmdData Vision_Tx;//发送给视觉的

int32_t vision_cnt;

void vision_task(void const * argument)
{
  /* USER CODE BEGIN vision_start */
  /* Infinite loop */
  for(;;)
  {
		//vision_tx();//同济
		send_vision();//武科
    vTaskDelay(1);
  }
  /* USER CODE END vision_start */
}

////视觉接收
//void vision_rx(uint8_t *buff)
//{
//	VisionToGimbal.head[0]=buff[0];
//	VisionToGimbal.head[1]=buff[1];
//	
//	if(buff[0]=='S'&&buff[1]=='P')
//	{
//		VisionToGimbal.mode=buff[2];
//		if(buff[2]==0)
//			GIMBAL.found_flag=0;
//		else
//			GIMBAL.found_flag=1;
//		for(int i=0;i<4;i++)
//		{
//			VisionToGimbal.yaw.data[i]=buff[i+3];
//			VisionToGimbal.yaw_vel.data[i]=buff[i+7];
//			VisionToGimbal.yaw_acc.data[i]=buff[i+11];
//			VisionToGimbal.pitch.data[i]=buff[i+15];
//			VisionToGimbal.pitch_vel.data[i]=buff[i+19];
//			VisionToGimbal.pitch_acc.data[i]=buff[i+23];
//		}
//		
//		if(mode.vision_switch_state!=VISION_CLOSE)
//		{
//			VisionToGimbal.Yaw_add=VisionToGimbal.yaw.d/2/PI*360 - INS.Yaw;
//			if(VisionToGimbal.pitch.d>-37.0f&&VisionToGimbal.pitch.d<37.0f)
//				VisionToGimbal.Pitch_add=VisionToGimbal.pitch.d/2/PI*360 - INS.Pitch;
//			else
//				VisionToGimbal.Pitch_add=0;
//			if(VisionToGimbal.Yaw_add> 180.0f)//过零处理
//			VisionToGimbal.Yaw_add-=360.0f;
//		else if(VisionToGimbal.Yaw_add<- 180.0f)
//			VisionToGimbal.Yaw_add+=360.0f;
//			VisionToGimbal.yaw_vision_target=VisionToGimbal.yaw.d/2/PI*360.0f;
//			VisionToGimbal.pitch_vision_target=VisionToGimbal.pitch.d/2/PI*360.0f;
//			VisionToGimbal.yaw_vision_target=zero_180(VisionToGimbal.yaw_vision_target);
//			VisionToGimbal.pitch_vision_target=pitch_protect(VisionToGimbal.pitch_vision_target);
//		}
//		else
//		{
//			VisionToGimbal.Yaw_add=0;
//			VisionToGimbal.Pitch_add=0;
//			VisionToGimbal.yaw_vision_target=INS.Yaw;
//			VisionToGimbal.pitch_vision_target=INS.Pitch;
//			VisionToGimbal.yaw_vision_target=zero_180(VisionToGimbal.yaw_vision_target);
//			VisionToGimbal.pitch_vision_target=pitch_protect(VisionToGimbal.pitch_vision_target);
//		}
//		

//		GIMBAL.yaw_vision_target=VisionToGimbal.yaw_vision_target;
//		GIMBAL.pitch_vision_target=VisionToGimbal.pitch_vision_target;
//		
//		if(GIMBAL.last_yaw_vision_target == VisionToGimbal.yaw.d/2/PI*360.0f)
//		{		
//		  vision_cnt++;
//		}
//		else
//			vision_cnt=0;
//		
//		if(vision_cnt>500)
//			GIMBAL.vision_block = 1;
//		else
//			GIMBAL.vision_block = 0;
//	}
//}

//void Tx_Handle(GimbalToVision_t *data)
//{
//  data->head[0]='S';
//  data->head[1]='P';
//  data->mode=mode.vision_switch_state;
//  data->q[0].d=INS.q[0];
//  data->q[1].d=INS.q[1];
//  data->q[2].d=INS.q[2];
//  data->q[3].d=INS.q[3];
//  data->yaw.d=INS.Yaw*0.01745f;
//  data->yaw_vel.d=INS.Gyro[2];
//  data->pitch.d=INS.Pitch*0.01745f;
//  data->pitch_vel.d=INS.Gyro[1];
//  data->bullet_speed.d=23;
//  data->bullet_count.d=0;
//  
//}

//视觉发送
//void vision_tx(void)
//{
//	Tx_Handle(&GimbalToVision);
//	

//	
//	memcpy(buf    ,&GimbalToVision.head,2);
//  memcpy(buf + 2 , &GimbalToVision.mode,1);
//  memcpy(buf + 3 , &GimbalToVision.q ,16);
//  memcpy(buf + 19 ,&GimbalToVision.yaw  ,4 );
//  memcpy(buf + 23 ,&GimbalToVision.yaw_vel  ,4 );
//  memcpy(buf + 27 ,&GimbalToVision.pitch  , 4);
//  memcpy(buf + 31 ,&GimbalToVision.pitch_vel  ,4 );
//  memcpy(buf + 35 ,&GimbalToVision.bullet_speed  ,4 );
//  memcpy(buf + 39 ,&GimbalToVision.bullet_count  ,2 );
//	
//	
//		append_CRC16_check_sum(buf,43);
//	if(verify_CRC16_check_sum(buf,43))
//		TX_OK_FLAG=1;
//	else
//		TX_OK_FLAG=0;
//	CDC_Transmit_FS(buf,sizeof(buf));//usb发送
//}
//是否识别到目标
//bool IF_DISCERN(void)
//{
//	if(GIMBAL.found_flag ==1 && GIMBAL.vision_block == 0)
//	{
//	  return 1;
//	}
//	else return 0;
//}


////武科
void tx_handle(SendRobotCmdData *data)
{
	data->cmd_ID=0x02;
	data->time_stamp=HAL_GetTick();
	data->yaw=INS.Yaw*0.01745f;
	data->pitch=INS.Pitch*0.01745f;
	data->roll=INS.Roll*0.01745f;
	data->ins_sum=0;
	data->yaw_vel=INS.Gyro[2];
	data->pitch_vel=INS.Gyro[1];
	data->roll_vel=INS.Gyro[0];
	data->v_x=0;
	data->v_y=0;
	data->v_z=0;
	data->bullet_speed=25;
	data->controller_delay=0;
	data->manual_reset_count=0;
	data->detect_color=0;
	

}

void send_vision()
{
	tx_handle(&Vision_Tx);

	memcpy(vision_buff    ,(const void*)&Vision_Tx.cmd_ID,1);
	memcpy(vision_buff+1    ,(const void*)&Vision_Tx.time_stamp,4);
	memcpy(vision_buff+5    ,(const void*)&Vision_Tx.yaw,4);
	memcpy(vision_buff+9    ,(const void*)&Vision_Tx.pitch,4);
	memcpy(vision_buff+13    ,(const void*)&Vision_Tx.roll,4);
	memcpy(vision_buff+17    ,(const void*)&Vision_Tx.ins_sum,4);
	memcpy(vision_buff+21    ,(const void*)&Vision_Tx.yaw_vel,4);
	memcpy(vision_buff+25    ,(const void*)&Vision_Tx.pitch_vel,4);
	memcpy(vision_buff+29    ,(const void*)&Vision_Tx.roll_vel,4);
	memcpy(vision_buff+33    ,(const void*)&Vision_Tx.v_x,4);
	memcpy(vision_buff+37    ,(const void*)&Vision_Tx.v_y,4);
	memcpy(vision_buff+41    ,(const void*)&Vision_Tx.v_z,4);
	memcpy(vision_buff+45    ,(const void*)&Vision_Tx.bullet_speed,4);
	memcpy(vision_buff+49    ,(const void*)&Vision_Tx.controller_delay,4);
	memcpy(vision_buff+53    ,(const void*)&Vision_Tx.manual_reset_count,1);
	memcpy(vision_buff+54    ,(const void*)&Vision_Tx.detect_color,1);
	
	CDC_Transmit_FS(vision_buff,sizeof(vision_buff));//usb发送
}

void receive_vision(uint8_t *buff)
{
	vision_data_u32 time_stamp;
	vision_data_float yaw;
	vision_data_float pitch;
	vision_data_float target_yaw;
	vision_data_float target_pitch;
	vision_data_float enable_yaw_diff;
	vision_data_float enable_pitch_diff;
	vision_data_float yaw_vel;
	vision_data_float pitch_vel;
	vision_data_float yaw_acc;
	vision_data_float pitch_acc;
	if(buff[0]==0x01)
	{
		
		for(int i=0;i<4;i++)
		{
			time_stamp.data[i]=buff[i+1];
			yaw.data[i]=buff[i+11];
			pitch.data[i]=buff[i+7];
			target_yaw.data[i]=buff[i+15];
			target_pitch.data[i]=buff[i+19];
			enable_yaw_diff.data[i]=buff[i+23];
			enable_pitch_diff.data[i]=buff[i+27];
			yaw_vel.data[i]=buff[i+31];
			pitch_vel.data[i]=buff[i+35];
			yaw_acc.data[i]=buff[i+39];
			pitch_acc.data[i]=buff[i+43];
		}
		
		Vision_Rx.time_stamp=time_stamp.d;
		Vision_Rx.yaw=yaw.d;
		Vision_Rx.pitch=pitch.d;
		Vision_Rx.target_yaw=target_yaw.d;
		Vision_Rx.target_pitch=target_pitch.d;
		Vision_Rx.enable_yaw_diff=enable_yaw_diff.d;
		Vision_Rx.enable_pitch_diff=enable_pitch_diff.d;
		Vision_Rx.v_yaw=yaw_vel.d;
		Vision_Rx.v_pitch=pitch_vel.d;
		Vision_Rx.a_yaw=yaw_acc.d;
		Vision_Rx.a_pitch=pitch_acc.d;
		Vision_Rx.appear=buff[5];
		Vision_Rx.shoot_rate=buff[6];
		Vision_Rx.detect_color=buff[47];
		
		if(Vision_Rx.appear)
		{
			GIMBAL.yaw_vision_target=zero_180(Vision_Rx.yaw);
			GIMBAL.pitch_vision_target=pitch_protect(Vision_Rx.pitch);
		}
		else
		{
			GIMBAL.yaw_vision_target=INS.Yaw;
			GIMBAL.pitch_vision_target=INS.Pitch;
		}
		
		if(GIMBAL.last_yaw_vision_target == Vision_Rx.yaw)
		  vision_cnt++;
		else
			vision_cnt=0;
	}
}

//是否识别到目标
bool IF_DISCERN(void)
{
	if(Vision_Rx.appear ==1 && GIMBAL.vision_block == 0)
	{
	  return 1;
	}
	else return 0;
}

//是否开火
bool IF_FIRE(void)
{
	if(fabs(INS.Yaw-Vision_Rx.yaw)<Vision_Rx.enable_yaw_diff&&fabs(INS.Pitch-Vision_Rx.pitch)<Vision_Rx.enable_pitch_diff)
		return 1;
	else
		return 0;
}
