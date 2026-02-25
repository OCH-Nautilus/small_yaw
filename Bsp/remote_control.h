/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "stdbool.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MIDDLE               ((uint16_t)3)
#define SWITCH_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define WHEEL_MAX_RIGHT 660
#define WHEEL_MIN_LEFT -660

#define RC_CHANNEL_MAX 660
#define RC_CHANNEL_MIN -660

#define KEY_DOWN 0 
#define KEY_UP 1

#define FLAG_ON 0 
#define FLAG_OFF 1

#define REMOTE_MODE 0
#define KEYBORD_MODE 1

/* ----------------------- Data Struct ------------------------------------- */

/* 拨轮状态枚举 */
typedef enum 
{
	ZERO = 0,
	UP_LONG = 1,
	DOWN_LONG  = 2,
	UP_SHORT = 3,
	DOWN_SHORT,
}Wheel_State_t;

/* 鼠标左键状态枚举 */
typedef enum 
{
	NOP = 0,
	PUSH_SHORT = 1,
	PUSH_LONG  = 2,
}KEY_L_State_t;

typedef struct
{
	struct
	{ 
		short ch[5];

		char s[2];
	
		long int wheel;
		Wheel_State_t WHEEL_State;
	}rc;
	
	struct 
	{
		short vx;
		short vy;
		short vz;
		unsigned char press_l;
		unsigned char press_r;
		KEY_L_State_t KEY_L_State;
	}mouse;
	
	struct
	{
		unsigned short v;
		uint8_t key_W;
		uint8_t key_S;
		uint8_t key_A;
		uint8_t key_D;
		uint8_t key_Shift;
		uint8_t key_CTRL;
		uint8_t key_Q;
		uint8_t key_E;
		
		uint8_t key_R;
		uint8_t key_F;
		uint8_t key_G;
		uint8_t key_Z;
		uint8_t key_X;
		uint8_t key_C;
		uint8_t key_V;
		uint8_t key_B;
		
		uint8_t last_Z;
		uint8_t last_X;
		uint8_t last_C;
		uint8_t last_V;
		uint8_t last_Ctrl;
		uint8_t last_Q;
		uint8_t last_G;
		uint8_t last_E;
		
		uint8_t flag_Z;
		uint8_t flag_X;
		uint8_t flag_C;
		uint8_t flag_V;
		uint8_t flag_Ctrl;
		uint8_t flag_Q;
		uint8_t flag_G;
		uint8_t flag_E;
		
		float vx;
		float vy;
		
	}keyboard;
	
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);

extern uint8_t remote_source;
extern RC_ctrl_t rc_ctrl;

extern uint32_t REMOTE_Time;

extern bool judge_if_remote_offline;
extern void judge_offline(void);

//extern bool Report_IF_RC_Connect(void);

/**
 * @brief 拨轮状态辨识
 * @param 
 */
void WHEEL_STATE_Ctrl(void);
/**
 * @brief 鼠标左键状态辨识
 * @param 
 */
void KEY_STATE_Ctrl(void);

/*遥控器参数初始化*/
void Remote_Data_Init(void);

bool_t RC_offline_data_handle(void);
void RC_connect_soft_restart(void);
bool_t RC_data_is_error_handle(void);

#endif


