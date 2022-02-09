#include "button.h"

Button_TypeDef ButtonArray[3];
uint16_t ButtonIndex = 0;
uint8_t key_count;

void SG_Button_add (GPIO_TypeDef*  GPIOx, uint16_t pin){
	ButtonArray[ButtonIndex].GPIOx = GPIOx;
	ButtonArray[ButtonIndex].pin = pin;
	ButtonIndex++;
};

int keyscan(void){
	//просканируем все кнопки, и посчитаем скольку они уже нажаты
	for (key_count = 0; key_count < 3; key_count++){
		if (!HAL_GPIO_ReadPin(ButtonArray[key_count].GPIOx, ButtonArray[key_count].pin))
			{
			ButtonArray[key_count].cnt_pressed++;
//			HAL_GPIO_WritePin(D1_LED_GPIO_Port, D1_LED_Pin, GPIO_PIN_RESET);
			}
		else{
//			HAL_GPIO_WritePin(D1_LED_GPIO_Port, D1_LED_Pin, GPIO_PIN_SET);
			ButtonArray[key_count].pressed = 0;
			ButtonArray[key_count].cnt_pressed = 0;
		}
	}

	for (key_count = 0; key_count < 3; key_count++)
	{
		if ((ButtonArray[key_count].cnt_pressed > 20) & (ButtonArray[key_count].pressed==0))
		{
			ButtonArray[key_count].pressed = 1;
			return key_count+1;
			//выполним команды в зависимости от нажатой кнопки.
		/*	switch (i) {
				case 0:
//					return 1;
					HAL_GPIO_TogglePin(D1_LED_GPIO_Port, D1_LED_Pin);
					break;
				case 1:
//					return 2;
					HAL_GPIO_TogglePin(D2_LED_GPIO_Port, D2_LED_Pin);
					break;
				case 2:
//					return 3;
					HAL_GPIO_TogglePin(D4_LED_GPIO_Port, D4_LED_Pin);
					break;
				default:
					break;
			}*/
		break;
		}
	}
	return 0;
}


