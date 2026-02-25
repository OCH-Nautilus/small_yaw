#include "struct_typedef.h"

typedef enum
{
	RC_ctrl=0,
	KEY_ctrl=1,
}controls_t;//操作模式


typedef enum
{
    GIMBAL_IDLE = 0,
    GIMBAL_NORMAL = 1,	
	GIMBAL_VISION = 2,
	GIMBAL_FOLD = 3,
}gimbal_state_t;//云台模式


typedef enum
{
	VISION_CLOSE = 0,
    VISION_ARMOR =1,
    VISION_SMALL_BUFF =2,
    VISION_BIG_BUFF =3,	
}vision_switch_state_t;//视觉开关


typedef enum
{
	SHOOT_IDLE = 0,
	SHOOT_OPEN = 1,
}shoot_state_t;//摩擦轮模式


typedef enum
{
	TRIGGER_IDLE = 0,
	TRIGGER_SINGLE = 1,
	TRIGGER_LONG = 2,
	TRIGGER_BACK = 3,
}trigger_state_t;//拨弹盘模式


typedef enum
{
	CHASSIS_IDLE = 0,
	CHASSIS_FOLLOW = 1,
	CHASSIS_TOP =2,
}chassis_state_t;//底盘模式


typedef enum
{
	SPEED_NORMAL = 0,
	SPEED_SHIFT =1,	
	SPEED_FLY =2,
}chassis_speed_state_t;//底盘加速度限制




typedef struct 
{
    controls_t               controls_state;
    gimbal_state_t           gimbal_state;
    vision_switch_state_t    vision_switch_state;
    shoot_state_t            shoot_state;
	trigger_state_t          trigger_state;
	chassis_state_t          chassis_state;
	chassis_speed_state_t    chassis_speed_state;
}mode_t;//模式总控

void mode_init(void);
void chassis_mode_change(void);
void chassis_rc_ctrl(void);
void shoot_rc_ctrl(void);
void system_conctrl(void);
void chassis_pc_ctrl(void);
void gimbal_rc_ctrl(void);
void gimbal_pc_ctrl(void);
void shoot_pc_ctrl(void);
void chassis_speed(void);
void vision_rc_ctrl(void);
void vision_pc_ctrl(void);
void tirgger_rc_ctrl(void);
void tirgger_pc_ctrl(void);
void Vision_to_Normal_init(void);
void Normal_to_Vision_init(void);

extern mode_t mode;

