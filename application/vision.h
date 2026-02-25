#include "struct_typedef.h"
#include "stdbool.h"



typedef union
{
	float  d;
	uint8_t data[4];
}vision_data_float;

typedef union{
	uint16_t d;
	uint8_t  data[2];
}vision_data_u16;

typedef union{
	uint32_t d;
	uint8_t  data[4];
}vision_data_u32;


typedef struct
{
	uint8_t head[2];
	uint8_t mode;//0空闲 1自瞄 2小符 3大符
	vision_data_float q[4];//wxyz顺序
	vision_data_float yaw;//角度(弧度)
	vision_data_float yaw_vel;//速度
	vision_data_float pitch;
	vision_data_float pitch_vel;
	vision_data_float bullet_speed;//弹速
	vision_data_u16 bullet_count;//子弹累计发送次数
	uint16_t crc16;
	
}GimbalToVision_t;


typedef struct
{
	uint8_t head[2];
	uint8_t mode;//0不控制 1控制云台但不开火 2控制云台且开火
	vision_data_float yaw;
	vision_data_float yaw_vel;
	vision_data_float yaw_acc;//角加速度
	vision_data_float pitch;
	vision_data_float pitch_vel;
	vision_data_float pitch_acc;
	uint16_t crc16;
	float Yaw_add;
	float Pitch_add;
	float yaw_vision_target;
	float pitch_vision_target;
	
}VisionToGimbal_t;

typedef struct  {
    uint8_t cmd_ID;           // 命令码 ID_AIM_INFO = 0x02
    uint32_t time_stamp;      // 时间戳
    
    float yaw;                // 云台yaw角 (rad)
    float pitch;              // 云台pitch角 (rad)
    float roll;               // roll角 (rad)
    float ins_sum;            
    
    float yaw_vel;            // yaw角速度 (rad/s)
    float pitch_vel;          // pitch角速度 (rad/s)
    float roll_vel;           // roll角速度 (rad/s)
    
    float v_x;                // 机器人x速度
    float v_y;                // 机器人y速度
    float v_z;                // 机器人z速度
    
    float bullet_speed;       // 弹速 (m/s)
    float controller_delay;   // 控制器延迟 (s)
    
    uint8_t manual_reset_count;
    uint8_t detect_color;     // 敌方颜色 0=红 1=蓝
} __attribute__((packed)) SendRobotCmdData ;//发送给视觉的


typedef struct  {
    uint8_t cmd_ID;            // 命令码 ID_ROBOT_CMD = 0x01
    uint32_t time_stamp;
    uint8_t appear;            // 是否发现目标
    uint8_t shoot_rate;        // 射频
    
    float pitch;               // 最佳控制pitch (度)
    float yaw;                 // 最佳控制yaw (度)
    
    float target_yaw;          // 最佳击中yaw (度)
    float target_pitch;        // 最佳击中pitch (度)
    
    float enable_yaw_diff;     // 允许开火的yaw误差 (度)
    float enable_pitch_diff;   // 允许开火的pitch误差 (度)
    
    float v_yaw;               // yaw角速度
    float v_pitch;             // pitch角速度
	
		float a_yaw;   // yaw 方向角加速度（度/s^2），发给电控
    float a_pitch; // pitch 方向角加速度（度/s^2），发给电控
    uint8_t detect_color;      // 敌方颜色
} __attribute__((packed)) ReceiveAimINFO;//视觉发送过来的

void vision_rx(uint8_t *buff);
void vision_tx(void);
bool IF_DISCERN(void);
bool judge_shoot(void);
extern VisionToGimbal_t VisionToGimbal;
void send_vision(void);
void receive_vision(uint8_t *buff);
extern ReceiveAimINFO Vision_Rx;//视觉发送回来的
extern SendRobotCmdData Vision_Tx;//发送给视觉的
bool IF_FIRE(void);

