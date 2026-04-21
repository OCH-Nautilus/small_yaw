/* pid.h ----------------------------------------------------------
 *
 * 简�? PID 控制�?
 * �?持“位�?式�?(mode = 0) 与“�?�量式�?(mode = 1)
 * ---------------------------------------------------------------*/

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/*----------------- �? -----------------*/
/* �? input 限制�? [-max, max] 区间 */
//#define LimitMax(input, max)            \
//    do {                                \
//        if ((input) > (max))            \
//            (input) = (max);            \
//        else if ((input) < -(max))      \
//            (input) = -(max);           \
//    } while (0)

/*----------------- 结构体定�? -----------------*/
typedef struct
{
    uint8_t mode;      /* 0: 位置�?, 1: 增量�? */
    float   Kp;
    float   Ki;
    float   Kd;

    float   max_out;   /* 输出限幅 */
    float   max_iout;  /* �?分项限幅 */

    float   set;       /* 设定�? */
    float   ref;       /* 反�?��? */

    float   out;       /* 最终输�? */
    float   Pout;
    float   Iout;
    float   Dout;

    float   error[3];  /* error[0] 最�?, error[1] 上一�?, error[2] 上上�? */
} pid_type_def;

/*----------------- API 声明 -----------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* 初�?�化 PID 结构�? */
extern void  PID_init(pid_type_def *pid,
                      uint8_t       mode,
                      float         Kp,
                      float         Ki,
                      float         Kd,
                      float         max_iout,
                      float         max_out);

/* 计算一�? PID 输出
 * ref : 当前测量�?
 * set : 设定�?
 * 返回  : �?次�?�算得到的输出�?
 */
typedef struct 
{
    /* data */
    float disturb;       //????
    float last_disturb;  //????
    float pre_disturb;   //????
    float alpha;         //a
    float belta;         //b
    float output;
    float outmax;
}feedforward_control_t;


extern float PID_calc(pid_type_def *pid, float ref, float set);
void feedforward_control_init(feedforward_control_t *str, float alpha, float belta, float outmax);
float feedforward_control_calc(feedforward_control_t *str, float disturb);

#ifdef __cplusplus
}
#endif

#endif  /* __PID_H */
