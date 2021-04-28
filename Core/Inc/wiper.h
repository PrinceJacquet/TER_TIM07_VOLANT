/*
 * wiper.h
 *
 *  Created on: Apr 21, 2021
 *      Author: jacqu
 */

#ifndef INC_WIPER_H_
#define INC_WIPER_H_

// INCLUDES
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"



typedef struct {
	GPIO_TypeDef * wiper_port;
	uint16_t wiper_pin;
	TIM_HandleTypeDef * wiper_timer_handler;
	uint32_t wiper_channel;

}wiper_init_t;//*_t for types


typedef enum
{
	power_on,
	power_off
}power_state_t; //*_t for types


typedef enum
{
	max_speed,
	min_speed,
	mean_speed
}wiper_speed_t; //*_t for types


void wiper_power_set(power_state_t power_state);

void wiper_pwm_start(wiper_speed_t speed, wiper_init_t  wiper_struct );

void wiper_pwm_stop(wiper_init_t  wiper_struct );


#endif /* INC_WIPER_H_ */
