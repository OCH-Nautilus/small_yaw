#include "trigger_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "config.h"
#include "CAN_receive.h"
#include "bsp_transmit.h"
#include "mode_task.h"
#include "math.h"
trigger_t TRIGGER;
pid_type_def pid_trigger_angle_signle;
pid_type_def pid_trigger_speed_signle;
pid_type_def pid_trigger_speed_long;
int uuuy=0,iiio=0;

stuck_t stuck_state;
void trigger_task(void const * argument)
{iiio++;
  trigger_init();
  vTaskDelay(10);
  for(;;)
  {
		uuuy++;
    // trigger_heat();
      stick_judge();
      back_stick();
      
   ////if(mode.trigger_state==TRIGGER_BACK)
//     trigger_back();
//   else
////   {
//     once_shoot();
//     driver_idle_stop();
//     continuous_shoot();
////   }
		if (stuck_state==STUCK_ERR)
        {
            trigger_back();
        }

        trigger_state_judge();
        if (stuck_state == STUCK_NORMAL)
        {
            once_shoot();
            driver_idle_stop();
            continuous_shoot();
        }
    trigger_pid_calc();
    vTaskDelay(1);
  }
  
}


/**
 * @brief 拨弹盘初始化
 */
void trigger_init()
{
    PID_init(&pid_trigger_angle_signle, PID_TIRGGER_ONCE_ANGLE_MODE, PID_TIRGGER_ONCE_ANGLE_KP, PID_TIRGGER_ONCE_ANGLE_KI, PID_TIRGGER_ONCE_ANGLE_KD, PID_TIRGGER_ONCE_ANGLE_MAXIOUT, PID_TIRGGER_ONCE_ANGLE_MAXOUT);
    PID_init(&pid_trigger_speed_signle, PID_TIRGGER_ONCE_SPEED_MODE, PID_TIRGGER_ONCE_SPEED_KP, PID_TIRGGER_ONCE_SPEED_KI, PID_TIRGGER_ONCE_SPEED_KD, PID_TIRGGER_ONCE_SPEED_MAXIOUT, PID_TIRGGER_ONCE_SPEED_MAXOUT);
    PID_init(&pid_trigger_speed_long, PID_TIRGGER_CONTI_SPEED_MODE, PID_TIRGGER_CONTI_SPEED_KP, PID_TIRGGER_CONTI_SPEED_KI, PID_TIRGGER_CONTI_SPEED_KD, PID_TIRGGER_CONTI_SPEED_MAXIOUT, PID_TIRGGER_CONTI_SPEED_MAXOUT);

    TRIGGER.once_first_flag = 0;
    TRIGGER.once_over_flag = 1;
    // 初始化反转标志位
    TRIGGER.if_back_flag = 0;
    TRIGGER.back_over_flag = 1;
    TRIGGER.back_target_set_flag = 0;
    TRIGGER.err_back_cnt = 0;
    TRIGGER.conti_speed = SPEED_2006_CONTI;
}

/**
 * @brief 拨盘pid计算
 */
void trigger_pid_calc()
{
	 if ((mode.trigger_state == TRIGGER_SINGLE) || (mode.trigger_state == TRIGGER_IDLE) || stuck_state==STUCK_ERR)
   {
       PID_calc(&pid_trigger_angle_signle, trigger_motor.total_ecd, TRIGGER.once_target_ecd);
       TRIGGER.output = PID_calc(&pid_trigger_speed_signle, trigger_motor.speed_rpm, pid_trigger_angle_signle.out);
   }
   else if (mode.trigger_state == TRIGGER_LONG)
   {
       TRIGGER.output = PID_calc(&pid_trigger_speed_long, trigger_motor.speed_rpm, TRIGGER.conti_speed);
   }

}

/**
 * @brief 单发
 */
void once_shoot()
{
   if (TRIGGER.back_over_flag==0)
       return;

   if (mode.trigger_state == TRIGGER_SINGLE)
       TRIGGER.once_first_flag = 1;

   if (TRIGGER.once_first_flag)
   {
       TRIGGER.once_first_flag = 0;
       TRIGGER.once_target_ecd = trigger_motor.total_ecd + ONCE_SHOOT_ANGLE_MAILUN;
   }

   if (floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
       TRIGGER.once_over_flag = 1;
   else
       TRIGGER.once_over_flag = 0;
}

/**
 * @brief 连发
 */
void continuous_shoot()
{
   if (mode.trigger_state != TRIGGER_LONG)
       return;
   if (!TRIGGER.if_back_flag && TRIGGER.back_over_flag)
   {
       TRIGGER.once_target_ecd = trigger_motor.total_ecd;
   }
}

/**
 * @brief 记录目标角位置
 */
void driver_idle_stop()
{
   if (mode.trigger_state != TRIGGER_IDLE)
       return;
   if (TRIGGER.once_over_flag)
       TRIGGER.once_target_ecd = trigger_motor.total_ecd;
}

/**
 * @brief 状态检测
 */
void trigger_state_judge()
{
    if (TRIGGER.back_over_flag && !TRIGGER.if_back_flag)
        stuck_state = STUCK_NORMAL;
		else
			 stuck_state = STUCK_ERR;
}


/**
 * @brief 反转处理
 */int rttt=0;
void trigger_back()
{
   // 如果需要反转且反转目标未设置
   if (TRIGGER.if_back_flag && !TRIGGER.back_target_set_flag)
   {
       TRIGGER.once_target_ecd = trigger_motor.total_ecd - ONCE_SHOOT_ANGLE_MAILUN / 2;
       TRIGGER.back_target_set_flag = 1;
   }

   // 检查反转是否完成
   if (TRIGGER.back_target_set_flag && floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
   {rttt++;
       // 反转完成，重置所有反转相关标志      
       TRIGGER.back_over_flag = 1;
       TRIGGER.if_back_flag = 0;
       TRIGGER.back_target_set_flag = 0;
       TRIGGER.err_cnt = 0;
       TRIGGER.once_target_ecd = trigger_motor.total_ecd + ONCE_SHOOT_ANGLE_MAILUN / 2;
       TRIGGER.once_over_flag = 1;
   }
//   if(floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
//       mode.trigger_state = TRIGGER_IDLE;
}

/**
 * @brief 反转堵转检测
 */
void back_stick()
{
   // 只在反转过程中检测反转卡弹
   if (!TRIGGER.back_over_flag && TRIGGER.back_target_set_flag)
   {
       if (floatabs(TRIGGER.output) > STUCK_CURRENT_THRESHOLD && floatabs(trigger_motor.speed_rpm) < STUCK_SPEED_THRESHOLD)//trigger_motor.given_current
       {
           TRIGGER.err_back_cnt++;
       }
       else
       {
           if (TRIGGER.err_back_cnt > 0)
               TRIGGER.err_back_cnt--;
       }

       if (TRIGGER.err_back_cnt > STUCK_THRESHOLD)
       {
           // 强制标记反转完成
           TRIGGER.back_over_flag = 1;
           TRIGGER.if_back_flag = 0;
           TRIGGER.back_target_set_flag = 0;
           TRIGGER.err_back_cnt = 0;
           TRIGGER.err_cnt = 0;

           // 重置目标位置
           TRIGGER.once_target_ecd = trigger_motor.total_ecd;
       }
   }
   else
   {
       // 不在反转状态时重置反转卡弹计数
       TRIGGER.err_back_cnt = 0;
   }
}

/**
 * @brief 卡弹时间检测
 */
void stick_judge()
{
   if (!TRIGGER.if_back_flag && TRIGGER.back_over_flag) // 反转未完成前不继续检测反转
   {
       if (floatabs(trigger_motor.given_current) > STUCK_CURRENT_THRESHOLD && floatabs(trigger_motor.speed_rpm) < STUCK_SPEED_THRESHOLD)
       {
           TRIGGER.err_cnt++;
       }
       else
       {
           if (TRIGGER.err_cnt > 0)
               TRIGGER.err_cnt--;
       }

       if (TRIGGER.err_cnt > STUCK_THRESHOLD)
       {
           mode.trigger_state = TRIGGER_BACK;
           TRIGGER.if_back_flag = 1;
           TRIGGER.back_over_flag = 0;
           TRIGGER.back_target_set_flag = 0;
       }
   }
}

///**
// * @brief 热量反馈
// */
//int next_heat_change()
//{
//    int next_shoot_num = 0; // 下一秒消耗弹丸数目
//    int heat_recover = 0;
//    int heat_consume = 0;
//    int heat_change = 0;
//    next_shoot_num = (int)(-TRIGGER.conti_speed / 600 * 0.25f + 1); // TIRGGER.conti_speed为连发转速，0.25f为反馈时差，TIRGGER.conti_speed/240为每秒发弹数量
//    heat_recover = robot_status.shooter_barrel_cooling_value / 10;  // 除10代表0.1秒的冷却
//    heat_consume = next_shoot_num * 10;                             // 热量消耗
//    heat_change = heat_recover - heat_consume;
//    return heat_change;
//}

//void trigger_heat() // 热量限制
//{
//    int heat_limit = robot_status.shooter_barrel_heat_limit; // 热量上限
//    int heat = power_heat_data.shooter_17mm_1_barrel_heat;   // 当前热量

//    if (heat_limit - heat + next_heat_change() < 20)
//    {
////        shoot_state = SHOOT_IDLE;
////        TIRGGER.weak_flag = 1;
//    }
//    else
//        TRIGGER.weak_flag = 0;

//    //	if(robot_status.shooter_barrel_cooling_value == 40 && heat_limit==50)
//    //	 {
//    //		TIRGGER.conti_speed = V_TIRGGER_10;
//    //	 }
//    //	 else
//    //	 {
//    //		 if(rc_ctrl.keyboard.flag_V == 0)
//    //		 {
//    //	     TIRGGER.conti_speed = V_TIRGGER_16;
//    //		 }
//    //		 else
//    //		 {
//    //		   TIRGGER.conti_speed = V_TIRGGER_20;
//    //		 }
//    //	 }
//}



/**
 * @brief 绝对值
 */
float floatabs(float a)
{
    return (a >= 0) ? a : -a;
}


//#include "trigger_task.h"
//#include "cmsis_os.h"
//#include "pid.h"
//#include "config.h"
//#include "CAN_receive.h"
//#include "bsp_transmit.h"
//#include "mode_task.h"
//#include "math.h"
//trigger_t TRIGGER;
//pid_type_def pid_trigger_angle_signle;
//pid_type_def pid_trigger_speed_signle;
//pid_type_def pid_trigger_speed_long;
//int uuuy=0,iiio=0;
//void trigger_task(void const * argument)
//{iiio++;
//  trigger_init();
//  vTaskDelay(10);
//  for(;;)
//  {
//		uuuy++;
//    // trigger_heat();
//      stick_judge();
//      back_stick();
//      
//   //if(mode.trigger_state==TRIGGER_BACK)
//     trigger_back();
////   else
////   {
//     once_shoot();
//     driver_idle_stop();
//     continuous_shoot();
////   }
//    trigger_pid_calc();
//    vTaskDelay(1);
//  }
//  
//}


///**
// * @brief 拨弹盘初始化
// */
//void trigger_init()
//{
//    PID_init(&pid_trigger_angle_signle, PID_TIRGGER_ONCE_ANGLE_MODE, PID_TIRGGER_ONCE_ANGLE_KP, PID_TIRGGER_ONCE_ANGLE_KI, PID_TIRGGER_ONCE_ANGLE_KD, PID_TIRGGER_ONCE_ANGLE_MAXIOUT, PID_TIRGGER_ONCE_ANGLE_MAXOUT);
//    PID_init(&pid_trigger_speed_signle, PID_TIRGGER_ONCE_SPEED_MODE, PID_TIRGGER_ONCE_SPEED_KP, PID_TIRGGER_ONCE_SPEED_KI, PID_TIRGGER_ONCE_SPEED_KD, PID_TIRGGER_ONCE_SPEED_MAXIOUT, PID_TIRGGER_ONCE_SPEED_MAXOUT);
//    PID_init(&pid_trigger_speed_long, PID_TIRGGER_CONTI_SPEED_MODE, PID_TIRGGER_CONTI_SPEED_KP, PID_TIRGGER_CONTI_SPEED_KI, PID_TIRGGER_CONTI_SPEED_KD, PID_TIRGGER_CONTI_SPEED_MAXIOUT, PID_TIRGGER_CONTI_SPEED_MAXOUT);

//    TRIGGER.once_first_flag = 0;
//    TRIGGER.once_over_flag = 1;
//    // 初始化反转标志位
//    TRIGGER.if_back_flag = 0;
//    TRIGGER.back_over_flag = 1;
//    TRIGGER.back_target_set_flag = 0;
//    TRIGGER.err_back_cnt = 0;
//    TRIGGER.conti_speed = SPEED_2006_CONTI;
//}

///**
// * @brief 拨盘pid计算
// */
//void trigger_pid_calc()
//{
// if ((mode.trigger_state == TRIGGER_SINGLE) || (mode.trigger_state == TRIGGER_IDLE) || mode.trigger_state == TRIGGER_BACK)
//   {
//       PID_calc(&pid_trigger_angle_signle, trigger_motor.total_ecd, TRIGGER.once_target_ecd);
//       TRIGGER.output = PID_calc(&pid_trigger_speed_signle, trigger_motor.speed_rpm, pid_trigger_angle_signle.out);
//   }
//   else if (mode.trigger_state == TRIGGER_LONG)
//   {
//       TRIGGER.output = PID_calc(&pid_trigger_speed_long, trigger_motor.speed_rpm, TRIGGER.conti_speed);
//   }

//}

///**
// * @brief 单发
// */
//void once_shoot()
//{
//   if (TRIGGER.back_over_flag==0)
//       return;

//   if (mode.trigger_state == TRIGGER_SINGLE)
//       TRIGGER.once_first_flag = 1;

//   if (TRIGGER.once_first_flag)
//   {
//       TRIGGER.once_first_flag = 0;
//       TRIGGER.once_target_ecd = trigger_motor.total_ecd + ONCE_SHOOT_ANGLE_MAILUN;
//   }

//   if (floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
//       TRIGGER.once_over_flag = 1;
//   else
//       TRIGGER.once_over_flag = 0;
//}

///**
// * @brief 连发
// */
//void continuous_shoot()
//{
//   if (mode.trigger_state != TRIGGER_LONG)
//       return;
//   if (!TRIGGER.if_back_flag && TRIGGER.back_over_flag)
//   {
//       TRIGGER.once_target_ecd = trigger_motor.total_ecd;
//   }
//}

///**
// * @brief 记录目标角位置
// */
//void driver_idle_stop()
//{
//   if (mode.trigger_state != TRIGGER_IDLE)
//       return;
//   if (TRIGGER.once_over_flag)
//       TRIGGER.once_target_ecd = trigger_motor.total_ecd;
//}

///**
// * @brief 反转处理
// */int rttt=0;
//void trigger_back()
//{
//   // 如果需要反转且反转目标未设置
//   if (TRIGGER.if_back_flag && !TRIGGER.back_target_set_flag)
//   {
//       TRIGGER.once_target_ecd = trigger_motor.total_ecd - ONCE_SHOOT_ANGLE_MAILUN / 2;
//       TRIGGER.back_target_set_flag = 1;
//   }

//   // 检查反转是否完成
//   if (TRIGGER.back_target_set_flag && floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
//   {rttt++;
//       // 反转完成，重置所有反转相关标志      
//       TRIGGER.back_over_flag = 1;
//       TRIGGER.if_back_flag = 0;
//       TRIGGER.back_target_set_flag = 0;
//       TRIGGER.err_cnt = 0;
//       TRIGGER.once_target_ecd = trigger_motor.total_ecd + ONCE_SHOOT_ANGLE_MAILUN / 2;
//       TRIGGER.once_over_flag = 1;
//   }
////   if(floatabs(TRIGGER.once_target_ecd - trigger_motor.total_ecd) < Judge_AngleErr)
////       mode.trigger_state = TRIGGER_IDLE;
//}

///**
// * @brief 反转堵转检测
// */
//void back_stick()
//{
//   // 只在反转过程中检测反转卡弹
//   if (!TRIGGER.back_over_flag && TRIGGER.back_target_set_flag)
//   {
//       if (floatabs(TRIGGER.output) > STUCK_CURRENT_THRESHOLD && floatabs(trigger_motor.speed_rpm) < STUCK_SPEED_THRESHOLD)//trigger_motor.given_current
//       {
//           TRIGGER.err_back_cnt++;
//       }
//       else
//       {
//           if (TRIGGER.err_back_cnt > 0)
//               TRIGGER.err_back_cnt--;
//       }

//       if (TRIGGER.err_back_cnt > STUCK_THRESHOLD)
//       {
//           // 强制标记反转完成
//           TRIGGER.back_over_flag = 1;
//           TRIGGER.if_back_flag = 0;
//           TRIGGER.back_target_set_flag = 0;
//           TRIGGER.err_back_cnt = 0;
//           TRIGGER.err_cnt = 0;

//           // 重置目标位置
//           TRIGGER.once_target_ecd = trigger_motor.total_ecd;
//       }
//   }
//   else
//   {
//       // 不在反转状态时重置反转卡弹计数
//       TRIGGER.err_back_cnt = 0;
//   }
//}

///**
// * @brief 卡弹时间检测
// */
//void stick_judge()
//{
//   if (!TRIGGER.if_back_flag && TRIGGER.back_over_flag) // 反转未完成前不继续检测反转
//   {
//       if (floatabs(trigger_motor.given_current) > STUCK_CURRENT_THRESHOLD && floatabs(trigger_motor.speed_rpm) < STUCK_SPEED_THRESHOLD)
//       {
//           TRIGGER.err_cnt++;
//       }
//       else
//       {
//           if (TRIGGER.err_cnt > 0)
//               TRIGGER.err_cnt--;
//       }

//       if (TRIGGER.err_cnt > STUCK_THRESHOLD)
//       {
//           mode.trigger_state = TRIGGER_BACK;
//           TRIGGER.if_back_flag = 1;
//           TRIGGER.back_over_flag = 0;
//           TRIGGER.back_target_set_flag = 0;
//       }
//   }
//}

/////**
//// * @brief 热量反馈
//// */
////int next_heat_change()
////{
////    int next_shoot_num = 0; // 下一秒消耗弹丸数目
////    int heat_recover = 0;
////    int heat_consume = 0;
////    int heat_change = 0;
////    next_shoot_num = (int)(-TRIGGER.conti_speed / 600 * 0.25f + 1); // TIRGGER.conti_speed为连发转速，0.25f为反馈时差，TIRGGER.conti_speed/240为每秒发弹数量
////    heat_recover = robot_status.shooter_barrel_cooling_value / 10;  // 除10代表0.1秒的冷却
////    heat_consume = next_shoot_num * 10;                             // 热量消耗
////    heat_change = heat_recover - heat_consume;
////    return heat_change;
////}

////void trigger_heat() // 热量限制
////{
////    int heat_limit = robot_status.shooter_barrel_heat_limit; // 热量上限
////    int heat = power_heat_data.shooter_17mm_1_barrel_heat;   // 当前热量

////    if (heat_limit - heat + next_heat_change() < 20)
////    {
//////        shoot_state = SHOOT_IDLE;
//////        TIRGGER.weak_flag = 1;
////    }
////    else
////        TRIGGER.weak_flag = 0;

////    //	if(robot_status.shooter_barrel_cooling_value == 40 && heat_limit==50)
////    //	 {
////    //		TIRGGER.conti_speed = V_TIRGGER_10;
////    //	 }
////    //	 else
////    //	 {
////    //		 if(rc_ctrl.keyboard.flag_V == 0)
////    //		 {
////    //	     TIRGGER.conti_speed = V_TIRGGER_16;
////    //		 }
////    //		 else
////    //		 {
////    //		   TIRGGER.conti_speed = V_TIRGGER_20;
////    //		 }
////    //	 }
////}



///**
// * @brief 绝对值
// */
//float floatabs(float a)
//{
//    return (a >= 0) ? a : -a;
//}
