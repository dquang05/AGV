#include <Arduino.h>
#include "pwmgen.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "soc/mcpwm_periph.h"


#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CORE_TO_BIND 1

volatile uint32_t signal_count_0 = 0;
volatile uint32_t signal_count_1 = 0;

void IRAM_ATTR capture_isr_handler_0(void *arg) {
    signal_count_0++;
    MCPWM0.int_clr.cap0_int_clr = 1;
}

void IRAM_ATTR capture_isr_handler_1(void *arg) {
    signal_count_1++;
    MCPWM1.int_clr.cap1_int_clr = 1;
}


void pwm_gen::begin(int chan, uint32_t frequency, float duty){
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.frequency = frequency;
    pwm_config.cmpr_a = duty;
    switch (chan){
        case 1:
            channel.num = 1;
            channel.phase = MCPWM0A;
            channel.pwm_pin = GPIO_NUM_15;
            channel.timer = MCPWM_TIMER_0;
            channel.cap_pin =  GPIO_NUM_36;
            channel.cap_sig = MCPWM_SELECT_CAP0;
            break;

        case 2:
            channel.num = 2;
            channel.phase = MCPWM1A;
            channel.pwm_pin = GPIO_NUM_17;
            channel.timer = MCPWM_TIMER_1;
            channel.cap_pin =  GPIO_NUM_39;
            channel.cap_sig = MCPWM_SELECT_CAP1;
            break;

        case 3:
            channel.num = 3;
            channel.phase = MCPWM2A;
            channel.pwm_pin = GPIO_NUM_19;
            channel.timer = MCPWM_TIMER_2;
            channel.cap_pin =  GPIO_NUM_34;
            channel.cap_sig = MCPWM_SELECT_CAP2;
            break;

        default:
            break;

    }
    mcpwm_gpio_init(channel.unit, channel.phase, channel.pwm_pin);
    mcpwm_init(channel.unit, channel.timer, &pwm_config);
    vTaskDelay(pdMS_TO_TICKS(100));
}
//Old capture_config function of ESP-IDF v4.4.1
// void pwm_gen::capture_config(int unit){
//     switch (unit){
//         case 0:
//             mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, channel.cap_pin);
//             mcpwm_capture_enable(MCPWM_UNIT_0,MCPWM_SELECT_CAP0,channel.cap_edg, 1);
//             mcpwm_isr_register(MCPWM_UNIT_0, capture_isr_handler_0, NULL, ESP_INTR_FLAG_LEVEL2, NULL);
//             MCPWM0.int_ena.cap0_int_ena = 1;
//             break;
//         case 1:
//             mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, channel.cap_pin);
//             mcpwm_capture_enable(MCPWM_UNIT_1,MCPWM_SELECT_CAP1,channel.cap_edg, 1);
//             mcpwm_isr_register(MCPWM_UNIT_1, capture_isr_handler_1, NULL, ESP_INTR_FLAG_LEVEL2, NULL);
//             MCPWM1.int_ena.cap1_int_ena = 1;
//             break;

//         default:
//             break;
//     }

//     vTaskDelay(pdMS_TO_TICKS(100));

// }

// New capture_config function for ESP-IDF v5.0 and later
void pwm_gen::capture_config(int unit) {
    
    mcpwm_capture_config_t cap_config = {
        .cap_edge = channel.cap_edg, 
        .cap_prescale = 1,           
        .capture_cb = NULL          
    };

    switch (unit) {
        case 0:
            mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, channel.cap_pin);
            mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &cap_config); // Thay thế
            mcpwm_isr_register(MCPWM_UNIT_0, capture_isr_handler_0, NULL, ESP_INTR_FLAG_LEVEL2, NULL);
            MCPWM0.int_ena.cap0_int_ena = 1;
            break;
        case 1:
            mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, channel.cap_pin);
            mcpwm_capture_enable_channel(MCPWM_UNIT_1, MCPWM_SELECT_CAP1, &cap_config); // Thay thế
            mcpwm_isr_register(MCPWM_UNIT_1, capture_isr_handler_1, NULL, ESP_INTR_FLAG_LEVEL2, NULL);
            MCPWM1.int_ena.cap1_int_ena = 1;
            break;
        default:
            break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

void pwm_gen::set_frequency(uint32_t frequency){
    mcpwm_set_frequency(channel.unit,channel.timer,frequency);
}

void pwm_gen::set_duty(float duty){
    mcpwm_set_duty(channel.unit,channel.timer,channel.generator,duty);
}
 
void pwm_gen::start_pwm(void){
    mcpwm_start(channel.unit, channel.timer);  
}

void pwm_gen::stop_pwm(void){
    mcpwm_stop(channel.unit, channel.timer);    
}

uint32_t pwm_gen::take_value(void){
    uint32_t captured_value;
    captured_value = mcpwm_capture_signal_get_value(channel.unit, channel.cap_sig);
    if (pre_capture<captured_value){
        speed = captured_value - pre_capture;
        pre_capture = captured_value;
    }
    return speed/80000000;
}