#include "struct_typedef.h"

typedef struct
{
	int16_t err_cnt;//正转时卡弹计时
	int16_t err_back_cnt;// 反转时卡弹计时
	uint16_t if_back_flag;//触发反转标志位
	uint16_t back_over_flag;//反转完成标志位
	uint16_t back_target_set_flag;// 反转目标角度是否已设置
	uint16_t once_first_flag;//单发目标角是否设置
	int32_t once_target_ecd;   // 单发及反转目标角
	uint16_t once_over_flag;//单发完成标志位
	int16_t conti_speed;//连发速度
	uint16_t weak_flag;//热量限制标志位
	uint16_t vision_weak_flag;//视觉状态下允许发弹标志位
	int16_t output;//目标电流
}trigger_t;

typedef enum
{
	STUCK_ERR=0,
	STUCK_NORMAL=1,

}stuck_t;

void trigger_init(void);
void trigger_pid_calc(void);
void once_shoot(void);
void continuous_shoot(void);
void driver_idle_stop(void);
void trigger_back(void);
void back_stick(void);
void stick_judge(void);
float floatabs(float a);
void trigger_state_judge(void);

void trigger_heat(void);//热量限制

extern trigger_t TRIGGER;
