/*
 * wiper.c
 *
 *  Created on: Apr 21, 2021
 *      Author: jacqu
 */

#include "wiper.h"


wiper_init_t g_wiper_object; //g_* for global variables

 void wiper_power_set(power_state_t power_state)
{
	if (power_state == power_on)
	{
		HAL_GPIO_WritePin(wiper_power_GPIO_Port,wiper_power_Pin, GPIO_PIN_SET);
	}
	if (power_state == power_off)
	{
		HAL_GPIO_WritePin(wiper_power_GPIO_Port,wiper_power_Pin, GPIO_PIN_RESET);
	}
	else
	{
		//error
		printf("something went wrong with funtion power_wiper");
	}
}

 void wiper_pwm_start(wiper_speed_t speed, wiper_init_t wiper_struct )
{
	 TIM2->CCR4=50;
	 HAL_TIM_Base_Start(&htim4);
	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


}


/*

HAL_Delay(1000);

while(CH4_DC < 100)
{
	TIM2->CCR4 = CH4_DC;
	CH4_DC += 1;
	HAL_Delay(10);
}
while(CH4_DC > 50)
{
	TIM2->CCR4 = CH4_DC;
	CH4_DC -= 1;
	HAL_Delay(10);
}

*/
