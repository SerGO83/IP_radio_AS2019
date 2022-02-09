/*
usage:
initialiation  button:

	SG_Button_add(KEY1_GPIO_Port, KEY1_Pin);
	SG_Button_add(KEY2_GPIO_Port, KEY2_Pin);


then in main cycle use this construction:

		switch (keyscan()) {
			case 1:
				HAL_GPIO_TogglePin(D1_LED_GPIO_Port, D1_LED_Pin);
				break;
			case 2:
				HAL_GPIO_TogglePin(D2_LED_GPIO_Port, D2_LED_Pin);
				break;
			case 3:
				HAL_GPIO_TogglePin(D4_LED_GPIO_Port, D4_LED_Pin);
				break;
			default:
				break;
		}


*/


//#include "stdio.h"
#include "stm32f7xx_hal.h"
#include "stm32f767xx.h"

typedef struct
{
	GPIO_TypeDef  *GPIOx;
	uint16_t		pin;
	_Bool			pressed;
	int32_t 		cnt_pressed;
} Button_TypeDef;

void SG_Button_add (GPIO_TypeDef* GPIOx, uint16_t pin);
int keyscan(void);
