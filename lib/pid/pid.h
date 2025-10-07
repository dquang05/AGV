#ifndef PID_H
#define PID_H
#include "driver/timer.h"

typedef void (*FuncPtr)(void);

struct pid_conf
{
    float sampT;
    float Kp;
    float Ki;
    float Kd;
    float *desire;
    float *actual;

    float alpha;
    float beta;
    float gama;
    float up_lim;
    float down_lim;

    volatile float error = 0;
    volatile float error1 = 0;
    volatile float error2 = 0;
};

struct timer_conf
{
    timer_group_t timer_gp = TIMER_GROUP_0;
    timer_idx_t timer_idx = TIMER_0;
};

void pid_timer_set(timer_conf timer, FuncPtr cfn, float sampT);
void pid_start(void);
void pid_stop();

class pid_gen
{

public:
    pid_conf _conf;
    volatile float output;

    void pid_begin(float sampT, float KP, float KI, float KD, float *desire, float *actual, float down_lim, float up_lim);
    float pid_calculate(float e, float e1, float e2);
    void pid_func(void);
    void pid_coef(void);
};

#endif