#include <Arduino.h>
#include "motor_control.h"
#include "pwmgen.h"
#include "pid.h"

pwm_gen motor1;
pwm_gen motor2;

pid_gen pid_ctr1;
pid_gen pid_ctr2;

volatile int error1 = 0;
volatile int error2 = 0;

volatile float set_speed1 = 0;
volatile float set_speed2 = 0;

volatile float cur_speed1 = 0;
volatile float cur_speed2 = 0;

float desire = 10;
float actual = 0;

volatile float output1 = 0;
volatile float output2 = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void speed_control(void)
{
    portENTER_CRITICAL(&mux);
    cur_speed1 = signal_count_0;
    cur_speed2 = signal_count_1;
    signal_count_0 = 0;
    signal_count_1 = 0;
    portEXIT_CRITICAL(&mux);

    pid_ctr1._conf.error = set_speed1 - cur_speed1;
    pid_ctr2._conf.error = set_speed2 - cur_speed2;

    float var1 = pid_ctr1.pid_calculate(pid_ctr1._conf.error,
                                        pid_ctr1._conf.error1,
                                        pid_ctr1._conf.error2);

    float var2 = pid_ctr2.pid_calculate(pid_ctr2._conf.error,
                                        pid_ctr2._conf.error1,
                                        pid_ctr2._conf.error2);

    pid_ctr1._conf.error2 = pid_ctr1._conf.error1;
    pid_ctr1._conf.error1 = pid_ctr1._conf.error;

    pid_ctr2._conf.error2 = pid_ctr2._conf.error1;
    pid_ctr2._conf.error1 = pid_ctr2._conf.error;

    if (output1 > 100)
    {
        output1 = 100;
    }
    else if (output1 < 0)
    {
        output1 = 0;
    }
    else
    {
        output1 = var1;
    }

    if (output2 > 100)
    {
        output2 = 100;
    }
    else if (output2 < 0)
    {
        output2 = 0;
    }
    else
    {
        output2 = var2;
    }

    pid_ctr1.output = output1;
    pid_ctr2.output = output2;

    error1 = pid_ctr1._conf.error;
    error2 = pid_ctr2._conf.error;

    // error1 = signal_count_0;
    // error2 = signal_count_1;

    motor1.set_duty(pid_ctr1.output);
    motor2.set_duty(pid_ctr2.output);

    signal_count_0 = 0;
    signal_count_1 = 0;
}

void direc_control(void)
{
}

void motor_begin(void)
{
    // Frequency: 500Hz, Duty: 50%
    motor1.begin(1, 500, 50);
    motor2.begin(2, 500, 50);

    motor1.capture_config(0);
    motor2.capture_config(1);

    pinMode(dir11, OUTPUT);
    pinMode(dir12, OUTPUT);
    pinMode(dir21, OUTPUT);
    pinMode(dir22, OUTPUT);

    digitalWrite(dir11, LOW);
    digitalWrite(dir12, LOW);
    digitalWrite(dir21, LOW);
    digitalWrite(dir22, LOW);

    timer_conf timer;

    pid_ctr1.pid_begin(0.05, 0.167, 10.2, 100.2, &desire, &actual, 0, 100);
    pid_ctr2.pid_begin(0.05, 0.167, 10.2, 100.2, &desire, &actual, 0, 100);

    // pid_ctr1.pid_begin(0.05, 0.156, 0.52, 0.26, &desire, &actual,0,100);
    // pid_ctr2.pid_begin(0.05, 0.156, 0.52, 0.26, &desire, &actual,0,100);

    FuncPtr fn = speed_control;

    pid_timer_set(timer, fn, 0.05);

    delay(100);
}

void motor_run(bool en)
{
    if (en)
    {
        // Run
        pid_start();

        motor1.start_pwm();
        motor2.start_pwm();
    }
    else
    {
        // Stop
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);

        pid_stop();

        motor1.stop_pwm();
        motor2.stop_pwm();

        pid_ctr1._conf.error = 0;
        pid_ctr1._conf.error1 = 0;
        pid_ctr1._conf.error2 = 0;

        pid_ctr2._conf.error = 0;
        pid_ctr2._conf.error1 = 0;
        pid_ctr2._conf.error2 = 0;
    }
}

void motor_speed_set(int v, int w)
{
    set_speed1 = float((2 * v + w * radius) / 2);
    set_speed2 = float((2 * v - w * radius) / 2);
    if (set_speed1 == 0)
    {
        digitalWrite(dir11, LOW);
        digitalWrite(dir12, LOW);
    }
    else
    {
        if (set_speed1 > 0)
        {
            digitalWrite(dir11, HIGH);
            digitalWrite(dir12, LOW);
        }
        else
        {
            digitalWrite(dir11, LOW);
            digitalWrite(dir12, HIGH);
            set_speed1 = -set_speed1;
        }
    }

    if (set_speed2 == 0)
    {
        digitalWrite(dir21, LOW);
        digitalWrite(dir22, LOW);
    }
    else
    {
        if (set_speed2 > 0)
        {
            digitalWrite(dir21, HIGH);
            digitalWrite(dir22, LOW);
        }
        else
        {
            digitalWrite(dir21, LOW);
            digitalWrite(dir22, HIGH);
            set_speed2 = -set_speed2;
        }
    }
}