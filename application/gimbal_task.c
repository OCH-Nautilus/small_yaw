#include "gimbal_task.h"
#include "cmsis_os.h"
#include "mode_task.h"
#include "pid.h"
#include "config.h"
#include "remote_control.h"
#include "ins_task.h"
#include "lowpass.h"
#include "bsp_transmit.h"
#include "CAN_receive.h"
#include "vision.h"
#include "math.h"
#include "kalman.h"
GIMBAL_t GIMBAL;
Modeling_Parameters_t Modeling_Parameters_yaw;

first_order_filter_type_t pitch_lowpass_key, yaw_lowpass_key;

feedforward_control_t yaw_vision_forward;
feedforward_control_t yaw_vision_forward_mechanism;

feedforward_control_t yaw_vision_speed_forward;

//系统辨识
pid_type_def pid_yaw_angle_Recognition;
pid_type_def pid_yaw_speed_Recognition;
pid_type_def pid_pitch_angle_Recognition;
pid_type_def pid_pitch_speed_Recognition;

// 无视觉
pid_type_def pid_yaw_angle;
pid_type_def pid_yaw_speed;
pid_type_def pid_pitch_angle;
pid_type_def pid_pitch_speed;
// 折叠
pid_type_def pid_yaw_angle_pos;
pid_type_def pid_yaw_speed_pos;
pid_type_def pid_pitch_angle_fold;
pid_type_def pid_pitch_speed_fold;
//掉头
pid_type_def pid_yaw_angle_dt;
pid_type_def pid_yaw_speed_dt;
pid_type_def pid_pitch_angle_dt;
pid_type_def pid_pitch_speed_dt;
// 自瞄装甲板
pid_type_def pid_yaw_vision_armor_angle;
pid_type_def pid_yaw_vision_armor_speed;
pid_type_def pid_pitch_vision_armor_angle;
pid_type_def pid_pitch_vision_armor_speed;

// 打符
pid_type_def pid_yaw_vision_buff_angle;
pid_type_def pid_yaw_vision_buff_speed;
pid_type_def pid_pitch_vision_buff_angle;
pid_type_def pid_pitch_vision_buff_speed;

float low_pass_yaw_key_num[1] = {LowPass_YAW_KEY_NUM};
float low_pass_pitch_key_num[1] = {LowPass_PITCH_KEY_NUM};
float qqq22=0;
void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
	
  gimbal_init();
 vTaskDelay(10);
  /* Infinite loop */
  for(;;)
  { qqq22++;
	switch (mode.controls_state)
	{
	case RC_ctrl:
		gimbal_mode_rc_ctrl();
		break;
	case KEY_ctrl:
	gimbal_mode_key_ctrl();
		break;
	default:
		break;
	}
	fold_state_judge();
	gimbal_pid_calc();
	GIMBAL.last_mode =mode.gimbal_state;
    vTaskDelay(1);
  }
  /* USER CODE END gimbal_task */
}
void gimbal_init()
{
    	//系统辨识参数
	Modeling_Parameters_yaw.Inertia=0.0318f;
	Modeling_Parameters_yaw.Viscosity_coefficient=0.1318f;
	Modeling_Parameters_yaw.Gravity=0;
	Modeling_Parameters_yaw.F_Coulomb=0.0655f;
	
	mode.gimbal_state=GIMBAL_IDLE;
	GIMBAL.PT_flag = 0;
	GIMBAL.last_mode =GIMBAL_IDLE;
	GIMBAL.IF_DT_OVER=1;
	GIMBAL.IF_FOLD_OVER=1;
	GIMBAL.angle_limit_flag=0;
	first_order_filter_init(&yaw_lowpass_key, LowPass_YAW_KEY_TIME, low_pass_yaw_key_num);
	first_order_filter_init(&pitch_lowpass_key, LowPass_PITCH_KEY_TIME, low_pass_pitch_key_num);

	
	feedforward_control_init(&yaw_vision_forward, VISION_YAW_ALPHA, VISION_YAW_BELTA, VISION_YAW_OUTMAX);
	feedforward_control_init(&yaw_vision_forward_mechanism, VISION_YAW_ALPHA_MECHANIASM, VISION_YAW_BELTA_MECHANIASM, VISION_YAW_OUTMAX_MECHANIASM);
	feedforward_control_init(&yaw_vision_speed_forward, VISION_YAW_SPEED_ALPHA, VISION_YAW_SPEED_BELTA, VISION_YAW_SPEED_OUTMAX);

  	//系统辨识
	PID_init(&pid_yaw_angle_Recognition, PID_YAW_ANGLE_RECO_MODE, PID_YAW_ANGLE_RECO_KP, PID_YAW_ANGLE_RECO_KI, PID_YAW_ANGLE_RECO_KD, PID_YAW_ANGLE_RECO_IMAX_OUT, PID_YAW_ANGLE_RECO_MAX_OUT);
	PID_init(&pid_yaw_speed_Recognition, PID_YAW_SPEED_RECO_MODE, PID_YAW_SPEED_RECO_KP, PID_YAW_SPEED_RECO_KI, PID_YAW_SPEED_RECO_KD, PID_YAW_SPEED_RECO_IMAX_OUT, PID_YAW_SPEED_RECO_MAX_OUT);
	PID_init(&pid_pitch_angle_Recognition, PID_PITCH_ANGLE_RECO_MODE, PID_PITCH_ANGLE_RECO_KP, PID_PITCH_ANGLE_RECO_KI, PID_PITCH_ANGLE_RECO_KD, PID_PITCH_ANGLE_RECO_IMAX_OUT, PID_PITCH_ANGLE_RECO_MAX_OUT);
	PID_init(&pid_pitch_speed_Recognition, PID_PITCH_SPEED_RECO_MODE, PID_PITCH_SPEED_RECO_KP, PID_PITCH_SPEED_RECO_KI, PID_PITCH_SPEED_RECO_KD, PID_PITCH_SPEED_RECO_IMAX_OUT, PID_PITCH_SPEED_RECO_MAX_OUT);

	// 无视觉(正常模式)
	PID_init(&pid_yaw_angle, PID_YAW_ANGLE_MODE, PID_YAW_ANGLE_KP, PID_YAW_ANGLE_KI, PID_YAW_ANGLE_KD, PID_YAW_ANGLE_IMAX_OUT, PID_YAW_ANGLE_MAX_OUT);
	PID_init(&pid_yaw_speed, PID_YAW_SPEED_MODE, PID_YAW_SPEED_KP, PID_YAW_SPEED_KI, PID_YAW_SPEED_KD, PID_YAW_SPEED_IMAX_OUT, PID_YAW_SPEED_MAX_OUT);
	PID_init(&pid_pitch_angle, PID_PITCH_ANGLE_MODE, PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD, PID_PITCH_ANGLE_IMAX_OUT, PID_PITCH_ANGLE_MAX_OUT);
	PID_init(&pid_pitch_speed, PID_PITCH_SPEED_MODE, PID_PITCH_SPEED_KP, PID_PITCH_SPEED_KI, PID_PITCH_SPEED_KD, PID_PITCH_SPEED_IMAX_OUT, PID_PITCH_SPEED_MAX_OUT);
		//折叠模式
	PID_init(&pid_yaw_angle_pos, PID_MOTOR_ANGLE_MODE, PID_MOTOR_ANGLE_KP, PID_MOTOR_ANGLE_KI, PID_MOTOR_ANGLE_KD, PID_MOTOR_ANGLE_IOUT_MAX, PID_MOTOR_ANGLE_OUT_MAX);
	PID_init(&pid_yaw_speed_pos, PID_MOTOR_SPEED_MODE, PID_MOTOR_SPEED_KP, PID_MOTOR_SPEED_KI, PID_MOTOR_SPEED_KD, PID_MOTOR_SPEED_IOUT_MAX, PID_MOTOR_SPEED_OUT_MAX);
	PID_init(&pid_pitch_angle_fold,PID_PITCH_MOTOR_ANGLE_MODE,PID_PITCH_MOTOR_ANGLE_KP,PID_PITCH_MOTOR_ANGLE_KI,PID_PITCH_MOTOR_ANGLE_KD,PID_PITCH_MOTOR_ANGLE_IOUT_MAX,PID_PITCH_MOTOR_ANGLE_OUT_MAX);
	PID_init(&pid_pitch_speed_fold,PID_PITCH_MOTOR_SPEED_MODE,PID_PITCH_MOTOR_SPEED_KP,PID_PITCH_MOTOR_SPEED_KI,PID_PITCH_MOTOR_SPEED_KD,PID_PITCH_MOTOR_SPEED_IOUT_MAX,PID_PITCH_MOTOR_SPEED_OUT_MAX);
//掉头
	PID_init(&pid_yaw_angle_dt, PID_YAW_ANGLE_MODE_DT, PID_YAW_ANGLE_KP_DT, PID_YAW_ANGLE_KI_DT, PID_YAW_ANGLE_KD_DT, PID_YAW_ANGLE_IMAX_OUT_DT, PID_YAW_ANGLE_MAX_OUT_DT);
	PID_init(&pid_yaw_speed_dt, PID_YAW_SPEED_MODE_DT, PID_YAW_SPEED_KP_DT, PID_YAW_SPEED_KI_DT, PID_YAW_SPEED_KD_DT, PID_YAW_SPEED_IMAX_OUT_DT, PID_YAW_SPEED_MAX_OUT_DT);
	PID_init(&pid_pitch_angle_dt, PID_PITCH_ANGLE_MODE_DT, PID_PITCH_ANGLE_KP_DT, PID_PITCH_ANGLE_KI_DT, PID_PITCH_ANGLE_KD_DT, PID_PITCH_ANGLE_IMAX_OUT_DT, PID_PITCH_ANGLE_MAX_OUT_DT);
	PID_init(&pid_pitch_speed_dt, PID_PITCH_SPEED_MODE_DT, PID_PITCH_SPEED_KP_DT, PID_PITCH_SPEED_KI_DT, PID_PITCH_SPEED_KD_DT, PID_PITCH_SPEED_IMAX_OUT_DT, PID_PITCH_SPEED_MAX_OUT_DT);

// 装甲板
	PID_init(&pid_yaw_vision_armor_angle, PID_YAW_VISION_ARMOR_ANGLE_MODE, PID_YAW_VISION_ARMOR_ANGLE_KP, PID_YAW_VISION_ARMOR_ANGLE_KI, PID_YAW_VISION_ARMOR_ANGLE_KD, PID_YAW_VISION_ARMOR_ANGLE_IMAX_OUT, PID_YAW_VISION_ARMOR_ANGLE_MAX_OUT);
	PID_init(&pid_yaw_vision_armor_speed, PID_YAW_VISION_ARMOR_SPEED_MODE, PID_YAW_VISION_ARMOR_SPEED_KP, PID_YAW_VISION_ARMOR_SPEED_KI, PID_YAW_VISION_ARMOR_SPEED_KD, PID_YAW_VISION_ARMOR_SPEED_IMAX_OUT, PID_YAW_VISION_ARMOR_SPEED_MAX_OUT);
	PID_init(&pid_pitch_vision_armor_angle, PID_PITCH_VISION_ARMOR_ANGLE_MODE, PID_PITCH_VISION_ARMOR_ANGLE_KP, PID_PITCH_VISION_ARMOR_ANGLE_KI, PID_PITCH_VISION_ARMOR_ANGLE_KD, PID_PITCH_VISION_ARMOR_ANGLE_IMAX_OUT, PID_PITCH_VISION_ARMOR_ANGLE_MAX_OUT);
	PID_init(&pid_pitch_vision_armor_speed, PID_PITCH_VISION_ARMOR_SPEED_MODE, PID_PITCH_VISION_ARMOR_SPEED_KP, PID_PITCH_VISION_ARMOR_SPEED_KI, PID_PITCH_VISION_ARMOR_SPEED_KD, PID_PITCH_VISION_ARMOR_SPEED_IMAX_OUT, PID_PITCH_VISION_ARMOR_SPEED_MAX_OUT);
	// buff
	PID_init(&pid_yaw_vision_buff_angle, PID_YAW_VISION_BUFF_ANGLE_MODE, PID_YAW_VISION_BUFF_ANGLE_KP, PID_YAW_VISION_BUFF_ANGLE_KI, PID_YAW_VISION_BUFF_ANGLE_KD, PID_YAW_VISION_BUFF_ANGLE_IMAX_OUT, PID_YAW_VISION_BUFF_ANGLE_MAX_OUT);
	PID_init(&pid_yaw_vision_buff_speed, PID_YAW_VISION_BUFF_SPEED_MODE, PID_YAW_VISION_BUFF_SPEED_KP, PID_YAW_VISION_BUFF_SPEED_KI, PID_YAW_VISION_BUFF_SPEED_KD, PID_YAW_VISION_BUFF_SPEED_IMAX_OUT, PID_YAW_VISION_BUFF_SPEED_MAX_OUT);
	PID_init(&pid_pitch_vision_buff_angle, PID_PITCH_VISION_BUFF_ANGLE_MODE, PID_PITCH_VISION_BUFF_ANGLE_KP, PID_PITCH_VISION_BUFF_ANGLE_KI, PID_PITCH_VISION_BUFF_ANGLE_KD, PID_PITCH_VISION_BUFF_ANGLE_IMAX_OUT, PID_PITCH_VISION_BUFF_ANGLE_MAX_OUT);
	PID_init(&pid_pitch_vision_buff_speed, PID_PITCH_VISION_BUFF_SPEED_MODE, PID_PITCH_VISION_BUFF_SPEED_KP, PID_PITCH_VISION_BUFF_SPEED_KI, PID_PITCH_VISION_BUFF_SPEED_KD, PID_PITCH_VISION_BUFF_SPEED_IMAX_OUT, PID_PITCH_VISION_BUFF_SPEED_MAX_OUT);
	
}

/**
 * @brief 云台pid计算
 * @note
 * @param
 */float temp_angle_pid;
void gimbal_pid_calc()
{
	
    float yaw_error;  // 用于存储最短路径误差
    
    switch(mode.gimbal_state)
    {
        case GIMBAL_NORMAL:
            if(mode.controls_state==RC_ctrl)
            {
                PID_calc(&pid_pitch_angle, INS.Pitch, GIMBAL.pitch_target);
                GIMBAL.output_pitch = PID_calc(&pid_pitch_speed, INS.Gyro[1], pid_pitch_angle.out); 
											
                if(GIMBAL.IF_DT_OVER==1)
                {
                    yaw_error = shortestAngleDiff(INS.Yaw, GIMBAL.yaw_target);
                    PID_calc(&pid_yaw_angle, 0, yaw_error);
                    GIMBAL.output_yaw = PID_calc(&pid_yaw_speed, INS.Gyro[2], pid_yaw_angle.out);
                }
                else
                {
										yaw_error = shortestAngleDiff(INS.Yaw, GIMBAL.yaw_target);
                    PID_calc(&pid_yaw_angle_dt, 0, yaw_error);
                    GIMBAL.output_yaw = PID_calc(&pid_yaw_speed_dt, INS.Gyro[2], pid_yaw_angle_dt.out); 
                }
            }
            else //KEY_ctrl
            {
                PID_calc(&pid_pitch_angle, INS.Pitch, GIMBAL.lowpass_pitch_target);
                GIMBAL.output_pitch = PID_calc(&pid_pitch_speed, INS.Gyro[1], pid_pitch_angle.out);
                
                if (GIMBAL.IF_DT_OVER == 1)
                {
                    yaw_error = shortestAngleDiff(INS.Yaw, GIMBAL.yaw_target);
                    PID_calc(&pid_yaw_angle, 0, yaw_error);
                    GIMBAL.output_yaw = PID_calc(&pid_yaw_speed, INS.Gyro[2], pid_yaw_angle.out);
                }
                else
                {
										yaw_error = shortestAngleDiff(INS.Yaw, GIMBAL.yaw_target);
                    PID_calc(&pid_yaw_angle_dt, 0, yaw_error);
                    GIMBAL.output_yaw = PID_calc(&pid_yaw_speed_dt, INS.Gyro[2], pid_yaw_angle_dt.out); 
                }
            }
            break;
            
        case GIMBAL_FOLD:
					if(fabs(big_pitch._pos-FOLD_BIG_PITCH_ANGLE)<0.03f)
					{
						PID_calc(&pid_pitch_angle_fold, small_pitch.Angle, GIMBAL.pitch_target);
            GIMBAL.output_pitch = PID_calc(&pid_pitch_speed_fold, INS.Gyro[1], pid_pitch_angle_fold.out); 
					}
          else
					{
						 PID_calc(&pid_pitch_angle, INS.Pitch, GIMBAL.pitch_target);
             GIMBAL.output_pitch = PID_calc(&pid_pitch_speed, INS.Gyro[1], pid_pitch_angle.out); 
					}
            
            PID_calc(&pid_yaw_angle_pos, small_yaw._pos, GIMBAL.yaw_target);
            GIMBAL.output_yaw = PID_calc(&pid_yaw_speed, INS.Gyro[2], pid_yaw_angle_pos.out); 
            break;
            
        case GIMBAL_VISION:
            PID_calc(&pid_pitch_vision_armor_angle, INS.Pitch, GIMBAL.pitch_target);
            GIMBAL.output_pitch = PID_calc(&pid_pitch_vision_armor_speed, INS.Gyro[1], pid_pitch_vision_armor_angle.out);
            
            yaw_error = shortestAngleDiff(INS.Yaw, GIMBAL.yaw_target);
            PID_calc(&pid_yaw_vision_armor_angle, 0, yaw_error);
						////GIMBAL.yaw_target
            GIMBAL.output_yaw = PID_calc(&pid_yaw_vision_armor_speed, INS.Gyro[2], pid_yaw_vision_armor_angle.out+feedforward_control_calc(&yaw_vision_forward,Vision_Rx.v_yaw))+feedforward_control_calc(&yaw_vision_speed_forward,Vision_Rx.a_yaw);////pid_yaw_vision_armor_angle.out
//							Modeling_Parameters_yaw.target_vel=	PID_calc(&pid_yaw_angle_Recognition, 0, yaw_error)+feedforward_control_calc(&yaw_vision_forward,VisionToGimbal.yaw_vel.d);
//							Modeling_Parameters_yaw.target_acc=PID_calc(&pid_yaw_speed_Recognition, INS.Gyro[2], pid_yaw_angle_Recognition.out);
//							GIMBAL.output_yaw=Modeling_Parameters_cacl(Modeling_Parameters_yaw);    
				
				break;
            
        case GIMBAL_IDLE:
            GIMBAL.output_yaw = 0;
            GIMBAL.output_pitch = 0;
            break;
            
        default:
            break;
    }
}
/**********************************遥控器控制云台***********************************/
/**
 * @brief gimbal_mode_rc_ctrl
 * @note
 * @param
 */
void gimbal_mode_rc_ctrl()
{
	switch (mode.gimbal_state)
	{
	case GIMBAL_IDLE:
		gimbal_mode_rc_idle();
		break;
	case GIMBAL_NORMAL:
		gimbal_mode_rc_normal();
		break;
	case GIMBAL_VISION:
		gimbal_mode_rc_vision();
		break;
	case GIMBAL_FOLD:
		gimbal_mode_rc_fold();
		break;
	default:
		gimbal_mode_rc_idle();
		break;
	}
}

void gimbal_mode_rc_idle()
{
	GIMBAL.yaw_target = INS.Yaw;
	GIMBAL.pitch_target = INS.Pitch;
	GIMBAL.big_pitch_target = NORMAL_BIG_PITCH_ANGLE;
	GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
}

void gimbal_mode_rc_normal()
{
	if(GIMBAL.target_renew_flag==1)//刚切换到正常模式
	{
		GIMBAL.yaw_target=INS.Yaw;
		GIMBAL.pitch_target=INS.Pitch;
		GIMBAL.target_renew_flag=0;
	}

	yaw_limit();
	if(GIMBAL.angle_limit_flag==0)
		GIMBAL.yaw_target -= rc_ctrl.rc.ch[2] * SENSITIVITY_YAW_RC;
	
	GIMBAL.big_pitch_target=NORMAL_BIG_PITCH_ANGLE;//固定角度
	
//	if (GIMBAL.PT_flag == 0)
		GIMBAL.pitch_target -= rc_ctrl.rc.ch[3] * SENSITIVITY_PITCH_RC;
// 	else
//	{
//		if (fabs(GIMBAL.pitch_target - imu.pitch) < 2.0f && fabs(-imu.pitch) > 8)
//		{
//			if (imu.pitch < -5.0f)
//			{
//				GIMBAL.pitch_target = imu.pitch + 3.0f;
//			}
//			else if (imu.pitch > 5.0f)
//			{
//				GIMBAL.pitch_target = imu.pitch - 3.0f;
//			}			
//		}
//		else if (fabs(-imu.pitch) <= 8)
//		{
//			GIMBAL.pitch_target = 0.0f;
//			GIMBAL.PT_flag = 0;
//		}
//	}
 	
	
	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
    GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
//	turn_round();
}

void gimbal_mode_rc_vision()
{
	if(GIMBAL.target_renew_flag==1)//刚从折叠模式切换
	{
		GIMBAL.yaw_target=INS.Yaw;
		GIMBAL.target_renew_flag=0;
	}
	
	if (IF_DISCERN()) 
	{
		if (mode.vision_switch_state == VISION_ARMOR)
		{
			GIMBAL.yaw_target = GIMBAL.yaw_vision_target;
			GIMBAL.pitch_target = GIMBAL.pitch_vision_target;
			
		}
		else if(mode.vision_switch_state==VISION_SMALL_BUFF||mode.vision_switch_state==VISION_BIG_BUFF)
		{
			GIMBAL.yaw_target=GIMBAL.yaw_vision_target;
			GIMBAL.pitch_target=GIMBAL.pitch_vision_target;
		}
	}
	else
	{
		GIMBAL.yaw_target -= rc_ctrl.rc.ch[2] * SENSITIVITY_YAW_RC;
		GIMBAL.pitch_target -= rc_ctrl.rc.ch[3] * SENSITIVITY_PITCH_RC;
	}

	GIMBAL.big_pitch_target=NORMAL_BIG_PITCH_ANGLE;//固定角度

	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
    GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
}

void gimbal_mode_rc_fold()
{
	GIMBAL.yaw_target=FOLD_SMALL_YAW_ANGLE;
	GIMBAL.target_renew_flag=1;
	
	if(fabs(small_yaw._pos-FOLD_SMALL_YAW_ANGLE)<0.05f)
		GIMBAL.big_pitch_target=FOLD_BIG_PITCH_ANGLE;

	if(fabs(big_pitch._pos-FOLD_BIG_PITCH_ANGLE)<0.03f)
		GIMBAL.pitch_target=FOLD_SMALL_PITCH_ANGLE;
}

/**********************************键鼠控制云台***********************************/
/**
 * @brief gimbal_mode_key_ctrl
 * @note
 * @param
 */
void gimbal_mode_key_ctrl()
{
	switch (mode.gimbal_state)
	{
	case GIMBAL_IDLE:
		gimbal_mode_key_idle();
		break;
	case GIMBAL_NORMAL:
		gimbal_mode_key_normal();
		break;
	case GIMBAL_VISION:
		gimbal_mode_key_vision();
		break;
		case GIMBAL_FOLD:
		gimbal_mode_key_fold();
		break;
	default:
		gimbal_mode_key_idle();
		break;
	}
}

void gimbal_mode_key_idle()
{
	GIMBAL.yaw_target = INS.Yaw;
	GIMBAL.pitch_target = INS.Pitch;
	GIMBAL.big_pitch_target=NORMAL_BIG_PITCH_ANGLE;
	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
	GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
}

void gimbal_mode_key_normal()
{
	if(GIMBAL.target_renew_flag==1)//刚切换到正常模式
	{
		GIMBAL.yaw_target=INS.Yaw;
		GIMBAL.pitch_target=INS.Pitch;
		GIMBAL.target_renew_flag=0;
	}

	yaw_limit();
	turn_round();
	if(GIMBAL.angle_limit_flag==0&&GIMBAL.IF_DT==0&&GIMBAL.IF_DT_OVER==1)//掉头未触发并且没超限位
		GIMBAL.yaw_target -= rc_ctrl.mouse.vx * SENSITIVITY_YAW_MOUSE;

	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
	GIMBAL.lowpass_yaw_target = first_order_filter_cali(&yaw_lowpass_key, GIMBAL.yaw_target);

	GIMBAL.big_pitch_target=NORMAL_BIG_PITCH_ANGLE;//固定角度
	
	// if (GIMBAL.PT_flag == 0)
	// {
		GIMBAL.pitch_target -= rc_ctrl.mouse.vy * SENSITIVITY_PITCH_MOUSE;
	// }
	// else
	// {
	// 	if (fabs(GIMBAL.pitch_target - INS.Pitch) < 2.0f && fabs(INS.Pitch) > 8)
	// 	{
	// 		if (INS.Pitch < -5.0f)
	// 		{
	// 			GIMBAL.pitch_target = INS.Pitch + 3.0f;
	// 		}
	// 		else if (INS.Pitch > 5.0f)
	// 		{
	// 			GIMBAL.pitch_target = INS.Pitch - 3.0f;
	// 		}
	// 	}
	// 	else if (fabs(-INS.Pitch) <= 8)
	// 	{
	// 		GIMBAL.pitch_target = 0.0f;
	// 		GIMBAL.PT_flag = 0;
	// 	}
	// }

	GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
	GIMBAL.lowpass_pitch_target = first_order_filter_cali(&pitch_lowpass_key, GIMBAL.pitch_target);
	
}

void gimbal_mode_key_vision()
{
	if(GIMBAL.target_renew_flag==1)//刚从折叠模式切换
	{
		GIMBAL.yaw_target=INS.Yaw;
		GIMBAL.target_renew_flag=0;
	}

	if (IF_DISCERN()) 
	{
		if (mode.vision_switch_state == VISION_ARMOR)
		{
			GIMBAL.yaw_target = GIMBAL.yaw_vision_target;
			GIMBAL.pitch_target = GIMBAL.pitch_vision_target;

		}
		else if(mode.vision_switch_state==VISION_SMALL_BUFF||mode.vision_switch_state==VISION_BIG_BUFF)
		{
			GIMBAL.yaw_target=GIMBAL.yaw_vision_target;
			GIMBAL.pitch_target=GIMBAL.pitch_vision_target;
		}
	}
	else
	{
		GIMBAL.yaw_target -= rc_ctrl.mouse.vx * SENSITIVITY_YAW_MOUSE;
		GIMBAL.pitch_target -= rc_ctrl.mouse.vy * SENSITIVITY_PITCH_MOUSE;
	}

	GIMBAL.big_pitch_target=NORMAL_BIG_PITCH_ANGLE;//固定角度
	GIMBAL.yaw_target=zero_180(GIMBAL.yaw_target);
	GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
}

void gimbal_mode_key_fold()
{
	GIMBAL.yaw_target=FOLD_SMALL_YAW_ANGLE;
	GIMBAL.target_renew_flag=1;
	
	if(fabs(small_yaw._pos-FOLD_SMALL_YAW_ANGLE)<0.05f)
		GIMBAL.big_pitch_target=FOLD_BIG_PITCH_ANGLE;

	if(fabs(big_pitch._pos-FOLD_BIG_PITCH_ANGLE)<0.03f)
		GIMBAL.pitch_target=FOLD_SMALL_PITCH_ANGLE;
	turn_round();
}

/*********************************其他功能*****************************************/
/**
 * @brief pitch限位
 * @note
 * @param
 */
float pitch_protect(float data)
{
	// pitch
	if (data > 37.0f)
		data = 37.0f;
	else if (data < -37.0f)
		data = -37.0f;
	return data;
}


void turn_round_over()
{
	if(mode.gimbal_state==GIMBAL_NORMAL)
	{
		if(GIMBAL.IF_DT_OVER == 0)
		{
			GIMBAL.IF_DT = 0;
			//mode.chassis_state=CHASSIS_IDLE;
			if (fabs(INS.Yaw-GIMBAL.yaw_target) <= 5)
			{
				GIMBAL.IF_DT_OVER = 1;				
			}
			else if (fabs(INS.Yaw-GIMBAL.yaw_target) > 5)
			{
				GIMBAL.IF_DT_OVER = 0;
			}
			
		}
	}
	else if(mode.gimbal_state==GIMBAL_FOLD)
	{
		if(GIMBAL.IF_DT_OVER == 0)
		{
			GIMBAL.IF_DT = 0;
			if (fabs(USART_Rx_data.big_yaw_target-USART_Rx_data.ins_big_yaw) <= 5)
			{
				GIMBAL.IF_DT_OVER = 1;				
			}
			else if (fabs(USART_Rx_data.big_yaw_target-USART_Rx_data.ins_big_yaw) > 5)
			{
				GIMBAL.IF_DT_OVER = 0;
			}
			
		}
	}
				
}
void turn_round()
{
 	if(GIMBAL.IF_DT==1&&GIMBAL.IF_DT_OVER==0&&mode.gimbal_state==GIMBAL_NORMAL)//触发掉头
 	{		
			GIMBAL.yaw_target+=180.0f;		
 	}
	
 	turn_round_over();
}
 

int Get_Sign(float a)
{
	int x=(a<0)?(-1):1;
	return x;
}


float tq_limit(float tq)
{
	if(tq>3.5f)
		tq=3.5f;
	if(tq<-3.5f)
		tq=-3.5f;

	return tq;
}
float Modeling_Parameters_cacl(Modeling_Parameters_t Modeling_Parameters)
{
	Modeling_Parameters.target_tq = Modeling_Parameters.Inertia*Modeling_Parameters.target_acc
                                      +Modeling_Parameters.Viscosity_coefficient*Modeling_Parameters.target_vel
                                      //+Modeling_Parameters.Gravity*sin(yaw_vision_set)
                                      +Modeling_Parameters.F_Coulomb*Get_Sign(Modeling_Parameters.target_vel);
	Modeling_Parameters.target_tq = tq_limit(Modeling_Parameters.target_tq);
	
	return Modeling_Parameters.target_tq;
}

/**
 * @brief 过零处理
 * @note  
 * @param
 */
float zero_180(float angle)
{
	if(angle>180.0f)
		angle-=360.0f;
	else if(angle<-180.0f)
			angle+=360.0f;
	
	return angle;
}

float zero_PI(float angle)
{
	if(angle>3.14159f)
		angle-=6.28318f;
	else if(angle<-3.14159f)
			angle+=6.28318f;
	
	return angle;
}

/**
 * @brief 计算最短路径角度差值（针对±180范围）
 * @param current 当前角度 (-180~180)
 * @param target 目标角度 (-180~180)
 * @return 最短路径差值 (-180~180)
 */
float shortestAngleDiff(float current, float target) 
{
    float diff = target - current;
    
    if (diff > 180.0f) 
		{
        diff -= 360.0f;
    } else if (diff < -180.0f) 
		{
        diff += 360.0f;
    }
    
    return diff;
}

/**
 * @brief 记录折叠电机编码器位置
 * @note  小yaw
 * @param
 */

float record_small_yaw_pos()
{
	return small_yaw.Angle;
} 

/**
 * @brief 记录折叠电机编码器位置
 * @note  pitch
 * @param
 */

float record_big_pitch_pos()
{
	return big_pitch.Angle;
} 

/**
 * @brief 折叠和升起完成判断
 * @note  大pitch
 * @param
 */
void fold_state_judge()
{
	if(fabs(big_pitch._pos-FOLD_BIG_PITCH_ANGLE)<0.05f)
		GIMBAL.down_over=1;
	else
		GIMBAL.down_over=0;

	if(fabs(big_pitch._pos-NORMAL_BIG_PITCH_ANGLE<0.05f))
		GIMBAL.rise_over=1;
	else
		GIMBAL.rise_over=0;
}

/**
 * @brief 大小yaw差角限位
 * @note  大pitch
 * @param
 */
 void yaw_limit()
{
	if(fabs(small_yaw._pos-FOLD_SMALL_YAW_ANGLE)>YAW_LIMIT_ANGLE)
		GIMBAL.angle_limit_flag=1;
	else
		GIMBAL.angle_limit_flag=0;
}

