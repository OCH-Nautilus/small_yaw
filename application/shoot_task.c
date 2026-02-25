#include "shoot_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "config.h"
#include "mode_task.h"
#include "CAN_receive.h"



shoot_t SHOOT;
pid_type_def pid_frictiongear_l;
pid_type_def pid_frictiongear_r;

void shoot_task(void const * argument)
{
    shoot_init();
    vTaskDelay(10);
    for(;;)
    {
        temp_shoot();
        frictiongear_calc();
				shoot_speed_adjust();
        vTaskDelay(1);
    }
}

/**
 * @brief 初始化
 */
void shoot_init()
{
    PID_init(&pid_frictiongear_l,PID_FRICTIONGEAR_LEFT_MODE,PID_FRICTIONGEAR_LEFT_KP,PID_FRICTIONGEAR_LEFT_KI,PID_FRICTIONGEAR_LEFT_KD,PID_FRICTIONGEAR_LEFT_MAXIOUT,PID_FRICTIONGEAR_LEFT_MAXOUT);
    PID_init(&pid_frictiongear_r,PID_FRICTIONGEAR_RIGHT_MODE,PID_FRICTIONGEAR_RIGHT_KP,PID_FRICTIONGEAR_RIGHT_KI,PID_FRICTIONGEAR_RIGHT_KD,PID_FRICTIONGEAR_RIGHT_MAXIOUT,PID_FRICTIONGEAR_RIGHT_MAXOUT);
   
}


/**
 * @brief 摩擦轮电流计算
 */
void frictiongear_calc()
{

    if(mode.shoot_state==SHOOT_OPEN)
        SHOOT.frictiongear_speed=6600;
    else
    {
        SHOOT.frictiongear_speed=0;
    }
		
	SHOOT.shoot_target_speed=SHOOT.frictiongear_speed+SHOOT.fix_num+SHOOT.temp_num;
    SHOOT.output[0]=PID_calc(&pid_frictiongear_l,frictiongear_l.speed_rpm,SHOOT.shoot_target_speed);//l
    SHOOT.output[1]=PID_calc(&pid_frictiongear_r,frictiongear_r.speed_rpm,-SHOOT.shoot_target_speed);//r
}

/**
 * @brief 温度补偿
 */
void temp_shoot()
{
	float temp_scope = 35;
	float temp_top = 35;
	float temp_real = 0;
	
	temp_real=(frictiongear_l.temperate+frictiongear_r.temperate)/2;
	if(temp_real<temp_top)
		SHOOT.temp_num=0;
	if(temp_real>temp_top)
		SHOOT.temp_num= (temp_real - temp_top)/temp_scope*( -168);
	if(temp_real>temp_top+temp_scope)
		SHOOT.temp_num=-168;
}

/**
 * @brief 射速补偿
 */
void shoot_speed_compensation(float shoot_max_speed,float shoot_min_speed,float real_speed,float up_num,float down_num,float *fix)
{
	SHOOT.last_speed[2]=SHOOT.last_speed[1];
	SHOOT.last_speed[1]=SHOOT.last_speed[0];
	SHOOT.last_speed[0]=real_speed;
	
	if(SHOOT.last_speed[0]<shoot_min_speed&&SHOOT.last_speed[0]>10)
		*fix=*fix+up_num;
	if(SHOOT.last_speed[0]>shoot_max_speed)
		*fix=*fix-down_num;
}

/**
 * @brief 射速自适应
 */
void shoot_speed_adjust()
{
	// if(SHOOT.last_shoot_num!=Report_Shoot_NUM())
	// 	shoot_speed_compensation(SHOOT_MAX_SPEED,SHOOT_MIN_SPEED,shoot_data.initial_speed,SHOOT_SPEED_ADD,SHOOT_SPEED_MINUS,&SHOOT.fix_num);
	
	// if(SHOOT.last_speed[0]!=0&&SHOOT.last_speed[1]!=0&&SHOOT.last_speed[2]!=0)
	// 	SHOOT.shoot_average_speed=(SHOOT.last_speed[0]+SHOOT.last_speed[1]+SHOOT.last_speed[2])/3;
	// else
	// 	SHOOT.shoot_average_speed=SHOOT_AVERAGE_SPEED;
	
	// SHOOT.last_shoot_num=Report_Shoot_NUM();
}
