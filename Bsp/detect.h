#ifndef __DETECT_TASK_H
#define __DETECT_TASK_H

#include "struct_typedef.h"

#include "stdbool.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 5

#define COMMUNICATION_NORMAL   0
#define COMMUNICATION_NONE     1

#define USE_DETECT


#define DBUS_MAX_OFFLINE_FRAME_RATE               100
#define CHASSIS_MOTOR_1_MAX_OFFLINE_FRAME_RATE    50
#define CHASSIS_MOTOR_2_MAX_OFFLINE_FRAME_RATE    50
#define CHASSIS_MOTOR_3_MAX_OFFLINE_FRAME_RATE    50
#define CHASSIS_MOTOR_4_MAX_OFFLINE_FRAME_RATE    50
#define FRIC_R_MAX_OFFLINE_FRAME_RATE             50
#define FRIC_L_MAX_OFFLINE_FRAME_RATE             50
#define TRIG_MAX_OFFLINE_FRAME_RATE               50
#define YAW_MAX_OFFLINE_FRAME_RATE                50
#define PITCH_MAX_OFFLINE_FRAME_RATE              50
#define VISION_MAX_OFFLINE_FRAME_RATE             300



//设备列表
enum TOEList
{
  DBUS_TOE = 0,
  CHASSIS_MOTOR1_TOE,
  CHASSIS_MOTOR2_TOE,
  CHASSIS_MOTOR3_TOE,
  CHASSIS_MOTOR4_TOE,
  FRICR_MOTOR_R_TOE,
  FRICR_MOTOR_L_TOE,
  TRIGGER_MOTOR_TOE,
  YAW_GIMBAL_MOTOR_TOE,
  PITCH_GIMBAL_MOTOR_TOE,
	VISION_TOE,
  //    BOARD_GYRO_TOE,
  //    BOARD_ACCEL_TOE,
  //    BOARD_MAG_TOE,
  //    REFEREE_TOE,
  //    OLED_TOE,
  ERROR_LIST_LENGHT,
};



typedef struct 
{
  /* data */
  uint32_t offline_frame_rate;                  //设备失联帧率
  uint32_t max_offline_frame_rate;              //设备最大失联帧率
  uint8_t communication_state;                  //通信状态
  bool_t (*toe_offline_data_handle_f)(void);    //设备失联数据处理
  void (*toe_unable_f)(void);                      //设备失联后失能
  void (*toe_connect_soft_restart_f)(void);          //设备失联后重新使能
}toe_offline_t;





/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
bool toe_is_error(uint8_t err);

/**
  * @brief          communication_frame_rate_update
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          通信帧率更新
  * @param[in]      toe:设备序号
  * @retval         none
  */
void communication_frame_rate_update(uint8_t toe);


extern toe_offline_t toe_offline[ERROR_LIST_LENGHT];


#endif

