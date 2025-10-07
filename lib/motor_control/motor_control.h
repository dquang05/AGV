#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define dir11 23
#define dir12 5
#define dir21 2
#define dir22 4
#define radius 10

extern volatile int error1;
extern volatile int error2;

extern volatile float cur_speed1;
extern volatile float cur_speed2;

extern volatile float output1;
extern volatile float output2;

void motor_begin(void);
void motor_run(bool en);
void motor_speed_set(int v, int w);
void speed_control(void);

#endif