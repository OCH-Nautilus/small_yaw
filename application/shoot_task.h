#include "struct_typedef.h"


typedef struct
{
	float last_speed[3];
	uint32_t last_shoot_num;
	float shoot_average_speed;
	float fix_num;
	float temp_num;
	float shoot_target_speed;
    int frictiongear_speed;
    int output[2];
}shoot_t;



extern shoot_t SHOOT;

void shoot_init(void);
void frictiongear_calc(void);
void temp_shoot(void);
void shoot_speed_adjust(void);

