#include <stdio.h> 
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "pid_ctrl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pid_ctrl.h" 
#include "driver/pulse_cnt.h"
#include "mcpwm_timer.h"

#include "mcpwm_types.h"
#include "mcpwm.h"
#include "mcpwm_gen.h" 
#include "mcpwm_oper.h"


#define MOTOR_TAG "MOTOR_TAG"
#define MCPWM_CLK_FREQ 1000000 // Clock Frequency = 1 MHz
#define GPIO_A 19
#define GPIO_B 21
#define GPIO_C
#define GPIO_D 

 
/*
mcpwm_timer_config_t mcpwm_params = { 
    .group_id = 0, 
    .clk_src = MCPWM_CAPTURE_CLK_SRC_APB, 
    .resolution_hz = MCPWM_CLK_FREQ, 
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP, 
    .period_ticks =  1000000,


}; 



mcpwm_operator_config_t pwm_op = { 
    .group_id = 0, 
    .flags.update_gen_action_on_tep = 1, 
    .flags.update_dead_time_on_tep = 1,

}; 

mcpwm_generator_config_t mcpwm_gen_A = { 
    .gen_gpio_num = GPIO_A, 

}; 

mcpwm_generator_config_t mcpwm_gen_B = { 
    .gen_gpio_num = GPIO_B,
}; 

mcpwm_gen_timer_event_action_t gen_A_timer = { 
    .direction = MCPWM_TIMER_DIRECTION_UP, 
    .event = MCPWM_TIMER_EVENT_FULL, 
    .action = MCPWM_GEN_ACTION_HIGH,
}; 
*/


/*mcpwm_pin_config_t pwm_setup = { 
    .mcpwm0a_out_num = GPIO_A, 
    .mcpwm0b_out_num = GPIO_B, 
}; 

mcpwm_config_t motor_init_set = { 
    .frequency = MCPWM_CLK_FREQ, 
    .cmpr_a = 50.0, 
    .cmpr_b = 50.0,
    .counter_mode = MCPWM_UP_COUNTER, 
    .duty_mode = MCPWM_DUTY_MODE_0
}; 

void app_main(uint8_t adc_val, bool init){ 

    mcpwm_timer_handle_t timer_unit; 
    mcpwm_oper_handle_t oper_unit; 
    mcpwm_gen_handle_t gen_ret_A; 
    mcpwm_gen_handle_t gen_ret_B; 
    mcpwm_timer_start_stop_cmd_t command = MCPWM_TIMER_START_NO_STOP; 
    ESP_ERROR_CHECK(mcpwm_new_timer(&mcpwm_params, &timer_unit)); 
    ESP_ERROR_CHECK(mcpwm_new_operator(&pwm_op, &oper_unit)); 
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_unit,&mcpwm_gen_A,&gen_ret_A)); 
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_unit,&mcpwm_gen_B,&gen_ret_B)); 
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_unit)); 
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_unit, command)); 
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_unit,timer_unit)); 
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(gen_ret_A,gen_A_timer));*/



    /*mcpwm_unit_t unit_a = MCPWM_UNIT_0; 
    mcpwm_timer_t Timer_0 = MCPWM_TIMER_0; 
    ESP_ERROR_CHECK(mcpwm_set_pin(unit_a, &pwm_setup)); 
    ESP_ERROR_CHECK(mcpwm_init(unit_a, Timer_0, &motor_init_set)); 
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit_a,MCPWM0A,GPIO_A)); 
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit_a,MCPWM0B,GPIO_B));
    ESP_ERROR_CHECK(mcpwm_set_duty(unit_a,Timer_0,MCPWM_OPR_A,50.0)); 
    ESP_ERROR_CHECK(mcpwm_set_duty(unit_a, Timer_0, MCPWM_OPR_B,50.0)); 
    ESP_ERROR_CHECK(mcpwm_start(unit_a,Timer_0)); 
    ESP_ERROR_CHECK(mcpwm_set_signal_high(unit_a,Timer_0,MCPWM_GEN_A));
    ESP_LOGI(MOTOR_TAG, "Works!");







}; */

/* brushed dc motor control example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "mcpwm.h"
#include "soc/mcpwm_periph.h"
#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}
/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}
/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}
/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}
/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}

