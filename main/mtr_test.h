#include <stdio.h> 
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"

static const char *MOTOR_TAG = "ESP32_BDC"; 

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 
#define BDC_MCPWM_GPIO_A 32
#define BDC_MCPWM_GPIO_B 33
#define BDC_MCPWM_GPIO_C 25
#define BDC_MCPWM_GPIO_D 26

static void dead_time_configuration()

mcpwm_timer_config_t mcpwm_timer = { 
    .group_id = 0;
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT; 
    .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;
}; 

mcpwm_operator_config_t new_oper = { 
    .group_id = 0; 
    
}; 

mcpwm_operator_config_t new_oper_b = { 
    .group_id = 1; 
};

mcpwm_generator_config_t new_gen_a = { 
    .gen_gpio_num = BDC_MCPWM_GPIO_A; 
}

mcpwm_generator_config_t new_gen_b = { 
    .gen_gpio_num = BDC_MCPWM_GPIO_B; 
}

mcpwm_dead_time_config_t new_dead_time = { 
    .posedge_delay_ticks = 2;  // For now, determine later on the appropriate tick delay. 
    .negedge_delay_ticks = 0; 
}; 

mcpwm_comparator_config_t new_comparator = { 
    .update_cmp_on_tez = true;
}; 

mcpwm_gen_timer_event_action_t gen_actions = { 
    .direction = MCPWM_TIMER_DIRECTION_UP; 
    .event = MCPWM_TIMER_EVENT_EMPTY; 
    .action = MCPWM_GEN_ACTION_HIGH;

}; 

mcpwm_gen_compare_event_action_t compare_actions_a = { 
    .direction = MCPWM_TIMER_DIRECTION_UP; 
    .action = MCPWM_GEN_ACTION_LOW; 
}; 

mcpwm_gen_compare_event_action_t compare_actions_b = { 
    .direction = MCPWM_TIMER_DIRECTION_UP; 
    .action = MCPWM_GEN_ACTION_LOW; 
}; 



void new_speed(bdc_motor_handle_t motor, uint8_t speed_key, int8_t servo_value){ 
    int32_t new_speed_calibrate; 
   

   if(servo_value == -90 || servo_value == 90){ 
    new_speed_calibrate = 2000; 
   }
   /*
   1- Determine period of MCPWM timers. 
   2- Determine values of A and B (Use comparator). 
   3- mcpwm_gen_compare_event_action_t -> Setup parameters to determine the count directiom, timer event and action. 
   
   
   
   
   
   */
ESP_LOGI(MOTOR_TAG,"Speed value: %d",speed_key); 

    if(speed_key <= 130 && speed_key >= 120){ 
        new_speed_calibrate = 0;
    }
  else{
   new_speed_calibrate = (speed_key*20000)/244-10000;
  };
    

    /*if(servo_value == 90 || servo_value == -90){ 
       speed_key = 2000; 
    }*/
    /*else if(servo_value == -90){ 
        speed_key = -400; 
    }*/

    if(new_speed_calibrate < 0){ 
        new_speed_calibrate *= -1; 
        //ESP_ERROR_CHECK(bdc_motor_forward(motor)); 
    }

    /*else { 
        ESP_ERROR_CHECK(bdc_motor_reverse(motor)); 
    }*/

ESP_ERROR_CHECK(bdc_motor_forward(motor)); 
    
     ESP_LOGI(MOTOR_TAG,"%ld",new_speed_calibrate); 
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor,new_speed_calibrate)); 
}





void motor_init(bdc_motor_handle_t *mtr_ret){
/*bdc_motor_mcpwm_config_t timer_init = { 
    .group_id = 0, 
    .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
};*/ 

mcpwm_timer_handle_t ret_tim = NULL; 
mcpwm_oper_handle_t ret_oper = NULL; 
mcpwm_oper_handle_t ret_oper_b = NULL; 
mcpwm_cmpr_handle_t cmp_a = NULL; 
mcpwm_cmpr_handle_t cmp_b = NULL; 
mcpwm_cmpr_handle_t cmp_c = NULL; 
mcpwm_cmpr_handle_t cmp_d = NULL; 
mcpwm_gen_handle_t gen_a = NULL; 
mcpwm_gen_handle_t gen_b = NULL; 
//******************************************
mcpwm_new_timer(&mcpwm_timer,&ret_tim); 
mcpwm_new_operator(&new_oper,&ret_oper); 
mcpwm_new_operator(&new_oper_b,&ret_oper_b); 
mcpwm_new_comparator(ret_oper,&new_comparator,&cmp_a); 
mcpwm_new_comparator(ret_oper,&new_comparator,&cmp_b); 
mcpwm_new_generator(ret_oper,&new_gen_a,&gen_a); 
mcpwm_new_generator(ret_oper,&new_gen_b,&gen_b); 
//*******************************************

mcpwm_timer_enable(ret_tim); 
mcpwm_timer_start_stop(ret_tim,MCPWM_TIMER_START_NO_STOP); 
mcpwm_operator_connect_timer(ret_oper,ret_tim); 
//********************************************
mcpwm_comparator_set_compare_value(cmp_a,0); 
mcpwm_comparator_set_compare_value(cmp_b,0); 
//********************************************
mcpwm_generator_set_actions_on_timer_event(gen_a,gen_actions,MCPWM_GEN_TIMER_EVENT_ACTION_END()); 
compare_actions_a.comparator = cmp_a; 
compare_actions_b.comparator = cmp_b; 
mcpwm_generator_set_actions_on_compare_event(gen_a,compare_actions_a,MCPWM_GEN_COMPARE_EVENT_ACTION_END()); 
mcpwm_generator_set_actions_on_timer_event(gen_b,gen_actions,MCPWM_GEN_TIMER_EVENT_ACTION_END()); 
mcpwm_generator_set_actions_on_timer_event(gen_b,compare_actions_b,MCPWM_GEN_COMPARE_EVENT_ACTION_END); 
//********************************************** 









/*bdc_motor_config_t pwm_init = { 
    .pwma_gpio_num = BDC_MCPWM_GPIO_A, 
    .pwmb_gpio_num = BDC_MCPWM_GPIO_B, 
    .pwm_freq_hz = 500, 
}; 

mcpwm_timer_config_t timer_con = { 
    .group_id = 0, 
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, 
    .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ, 
    .period_ticks = BDC_MCPWM_TIMER_RESOLUTION_HZ/pwm_init.pwm_freq_hz, 
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
}; */



bdc_motor_handle_t motor = NULL; 
//ESP_ERROR_CHECK(mcpwm_new_timer(&timer_con,&ret_tim))
ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&pwm_init,&timer_init,&motor)); 
ESP_LOGI(MOTOR_TAG, "Motor initialized"); 
ESP_ERROR_CHECK(bdc_motor_enable(motor)); 
//ESP_ERROR_CHECK(bdc_motor_forward(motor)); 
ESP_LOGI(MOTOR_TAG, "Motor moved forward"); 
*mtr_ret = motor; 



}; 