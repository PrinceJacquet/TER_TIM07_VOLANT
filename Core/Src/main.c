/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "epd2in7.h"
//#include "epdif.h"
//#include "epdpaint.h"
//#include "imagedata.h"

#include "wiper.h"
#include <stdlib.h>
#include <stdio.h>

#include "../../User/Inc/EPD_Test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define COLORED      0
#define UNCOLORED    1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
	//  unsigned char* frame_buffer = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
 // EPD epd;
 // if (EPD_Init(&epd) != 0) {
 //   printf("e-Paper init failed\n");
 //   return -1;
//  }

//  Paint paint;
//  Paint_Init(&paint, frame_buffer, epd.width, epd.height);
//  Paint_Clear(&paint, COLORED);
 // Paint_DrawRectangle(&paint, epd.width, epd.height, 0,0 , 0xFF);


  /* Draw something to the frame buffer */
  /* For simplicity, the arguments are explicit numerical coordinates */
 // Paint_SetRotate(&paint, ROTATE_270);
 // Paint_DrawStringAt(&paint, 18, 18, "0025", &Font72, UNCOLORED);

  /* Display the frame_buffer */
//  EPD_DisplayFrame(&epd, frame_buffer);
//  HAL_Delay(2000);
  /* Display the image buffer */
  //EPD_DisplayFrame(&epd, IMAGE_DATA);
  //int x = 0;
  //char Message[5];


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //int CH4_DC = 10;

  extern  wiper_init_t g_wiper_object;

  g_wiper_object.wiper_channel=TIM_CHANNEL_4;
  g_wiper_object.wiper_pin = wiper_power_Pin;
  g_wiper_object.wiper_port=wiper_power_GPIO_Port;
  g_wiper_object.wiper_timer_handler=&htim2;

  //wiper_power_set(power_on);
  HAL_GPIO_WritePin(wiper_power_GPIO_Port,wiper_power_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  //CAN
  CAN_TxHeaderTypeDef can_header;
  can_header.DLC=4;
  can_header.StdId=0x100;
  can_header.ExtId=0x100;
  can_header.IDE=CAN_ID_EXT;
  can_header.RTR=CAN_RTR_DATA;
  can_header.TransmitGlobalTime=DISABLE;


  //CAN_RxHeaderTypeDef   RxHeader;
  uint8_t               TxData[8] ={1,2,3,4,5,6,7,8};
 // uint8_t               RxData[8];
  uint32_t              Mailbox = 0;

 // char data[8] = "HELLO  ";

  HAL_CAN_Start(&hcan);

  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*Accueil de l'ecran*/



	// sprintf(Message,"%d",x++);
	//Paint_Clear(&paint, COLORED);
	 // my_EPD_DisplayFrame(&epd, 18, 18, frame_buffer, 0);
	HAL_Delay(50);

	//Paint_Clear(&paint, UNCOLORED);
	//Paint_DrawFilledCircle(&paint, 18 + (176-18-18)/2, 120, 40, COLORED);
	//my_EPD_DisplayFrame(&epd, frame_buffer, 1);

	//Paint_DrawStringAt(&paint, 50, 50, Message, &Font72, UNCOLORED);
	//Paint_DrawFilledRectangle(&paint, 18, 60, 176-18, 180, UNCOLORED);
	//EPD_DisplayFrame(&epd, frame_buffer);









	  if(HAL_CAN_AddTxMessage(&hcan, &can_header, TxData, &Mailbox) != HAL_OK){
		  Error_Handler();
	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }

	  while(HAL_CAN_IsTxMessagePending(&hcan, Mailbox));

	  HAL_Delay(1000);

	 /* while(CH4_DC < 100)
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
	 //wiper_pwm_start(max_speed, g_wiper_object );



*/


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
