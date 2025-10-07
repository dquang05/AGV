#include <Arduino.h>
#include "pid.h"
#include "driver/timer.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

pid_conf _conf;
FuncPtr fn;

timer_conf _timer;

static bool IRAM_ATTR timer_isr_2(void *para) {
    noInterrupts();
    timer_spinlock_take(_timer.timer_gp);
    timer_group_clr_intr_status_in_isr(_timer.timer_gp, _timer.timer_idx);
    timer_group_enable_alarm_in_isr(_timer.timer_gp, _timer.timer_idx);

    fn();
    // Serial.println("-");

    timer_spinlock_give(_timer.timer_gp);
    interrupts();
    return true;
}

void pid_timer_set(timer_conf timer, FuncPtr cfn, float sampT){
    timer_config_t timer_conf{
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_START,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80 
    };

    _timer = timer;

    timer_init(_timer.timer_gp, _timer.timer_idx, &timer_conf);
    timer_set_counter_value(_timer.timer_gp, _timer.timer_idx, 0);
    timer_set_alarm_value(_timer.timer_gp, _timer.timer_idx, uint64_t(sampT*TIMER_BASE_CLK/timer_conf.divider));
    timer_enable_intr(_timer.timer_gp, _timer.timer_idx);

    fn = cfn;

    timer_isr_callback_add(_timer.timer_gp, _timer.timer_idx, timer_isr_2, NULL, 0);

    // esp_err_t rs1 = esp_intr_alloc(ETS_INTERNAL_TIMER0_INTR_SOURCE, 
    //                      ESP_INTR_FLAG_LEVEL1, 
    //                      timer_isr_2, 
    //                      NULL, 
    //                      NULL);
    
    
    // Serial.println(rs1);
    
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void pid_start(void){
    timer_start(_timer.timer_gp, _timer.timer_idx);
}

void pid_stop(void){
    timer_pause(_timer.timer_gp, _timer.timer_idx);
}

void pid_gen::pid_begin(float sampT, float KP, float KI, float KD, float *desire,float *actual, float down_lim, float up_lim){
    _conf.sampT = sampT;
    _conf.Kp = KP;
    _conf.Ki = KI;
    _conf.Kd = KD;
    _conf.desire = desire;
    _conf.actual = actual;
    _conf.up_lim = up_lim;
    _conf.down_lim = down_lim;
    pid_coef();   
}

void pid_gen::pid_coef(void){
    _conf.alpha = 2*_conf.sampT*_conf.Kp+_conf.Ki*_conf.sampT*_conf.sampT+2*_conf.Kd;
	_conf.beta = _conf.sampT*_conf.sampT*_conf.Ki -4*_conf.Kd -2*_conf.sampT*_conf.Kp;
	_conf.gama = 2*_conf.Kd;
}

float_t pid_gen::pid_calculate(float e, float e1,float e2){
	float cur_out = (_conf.alpha*e+_conf.beta*e1+_conf.gama*e2+2*_conf.sampT*output)/(2*_conf.sampT);
    return cur_out;
}

void pid_gen::pid_func(void){
    _conf.error = *(_conf.actual)-(*(_conf.desire));
    float val = pid_calculate(_conf.error,_conf.error1,_conf.error2);
    if (val > _conf.up_lim){
        output =  _conf.up_lim;
    }
    else if (val < _conf.down_lim){
        output = _conf.down_lim;
    }
    else{
        output = val;
    }

    _conf.error2 = _conf.error1;
    _conf.error1 = _conf.error;
}