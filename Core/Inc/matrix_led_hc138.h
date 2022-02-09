#define __HC138_G1_ON() 	HAL_GPIO_WritePin(HC138_G1_GPIO_Port, HC138_G1_Pin, GPIO_PIN_SET)


void HC138_Res_All_Cols(void);
void HC138_Set_Col(int col);
void HC138_Set_Row(int row);
void HC138_ReSet_Col(int col);


