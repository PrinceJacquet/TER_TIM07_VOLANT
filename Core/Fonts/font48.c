/**
  ******************************************************************************
  * @file    font48.c
  * @author  Sofiane AOUCI
  * @version V1.0.0
  * @date    27-July-2020
  * @brief   This file provides text font48.
  ******************************************************************************
  **/

/* Includes ------------------------------------------------------------------*/
#include "fonts.h"

const uint8_t Font48_Table [] =
{
	// @1152 '0' (17 pixels wide)
		0,	0,	0,	0,	0,	0,
		0,	0,	7,	224,	0,	0,
		0,	0,	31,	248,	0,	0,
		0,	0,	60,	60,	0,	0,
		0,	0,	120,	14,	0,	0,
		0,	0,	240,	15,	0,	0,
		0,	1,	224,	7,	128,	0,
		0,	3,	224,	3,	192,	0,
		0,	3,	192,	3,	192,	0,
		0,	7,	192,	3,	224,	0,
		0,	7,	192,	3,	224,	0,
		0,	15,	128,	1,	240,	0,
		0,	15,	128,	1,	240,	0,
		0,	15,	128,	1,	240,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	0,	248,	0,
		0,	31,	128,	0,	248,	0,
		0,	31,	128,	0,	248,	0,
		0,	31,	0,	0,	252,	0,
		0,	63,	0,	0,	252,	0,
		0,	63,	0,	0,	252,	0,
		0,	63,	0,	0,	252,	0,
		0,	63,	0,	0,	252,	0,
		0,	63,	0,	0,	252,	0,
		0,	63,	0,	0,	248,	0,
		0,	31,	0,	0,	248,	0,
		0,	31,	0,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	31,	128,	1,	248,	0,
		0,	15,	128,	1,	240,	0,
		0,	15,	128,	1,	240,	0,
		0,	15,	128,	1,	240,	0,
		0,	7,	192,	3,	224,	0,
		0,	7,	192,	3,	224,	0,
		0,	3,	192,	3,	192,	0,
		0,	3,	224,	7,	192,	0,
		0,	1,	224,	7,	128,	0,
		0,	0,	240,	15,	0,	0,
		0,	0,	240,	30,	0,	0,
		0,	0,	124,	60,	0,	0,
		0,	0,	31,	248,	0,	0,
		0,	0,	7,	192,	0,	0,
		0,	0,	0,	0,	0,	0,
		0,	0,	0,	0,	0,	0
};

sFONT Font48 = {
  Font48_Table,
  41, /* Width */
  48, /* Height */
};

/************************ (C) COPYRIGHT TIM UPS INSA *****END OF FILE****/
