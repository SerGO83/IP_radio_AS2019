#include "main.h"

#define __HC138_A_ON() 	HAL_GPIO_WritePin(HC138_A_GPIO_Port, HC138_A_Pin, GPIO_PIN_SET)
#define __HC138_B_ON() 	HAL_GPIO_WritePin(HC138_B_GPIO_Port, HC138_B_Pin, GPIO_PIN_SET)
#define __HC138_C_ON() 	HAL_GPIO_WritePin(HC138_C_GPIO_Port, HC138_C_Pin, GPIO_PIN_SET)
#define __HC138_G1_ON() 	HAL_GPIO_WritePin(HC138_G1_GPIO_Port, HC138_G1_Pin, GPIO_PIN_SET)
#define __HC138_A_OFF() 	HAL_GPIO_WritePin(HC138_A_GPIO_Port, HC138_A_Pin, GPIO_PIN_RESET)
#define __HC138_B_OFF() 	HAL_GPIO_WritePin(HC138_B_GPIO_Port, HC138_B_Pin, GPIO_PIN_RESET)
#define __HC138_C_OFF() 	HAL_GPIO_WritePin(HC138_C_GPIO_Port, HC138_C_Pin, GPIO_PIN_RESET)
#define __HC138_G1_OFF() 	HAL_GPIO_WritePin(HC138_G1_GPIO_Port, HC138_G1_Pin, GPIO_PIN_RESET)


void HC138_Res_All_Cols(void){
	HAL_GPIO_WritePin(COL_01_GPIO_Port,COL_01_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_02_GPIO_Port,COL_02_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_03_GPIO_Port,COL_03_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_04_GPIO_Port,COL_04_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_05_GPIO_Port,COL_05_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_06_GPIO_Port,COL_06_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_07_GPIO_Port,COL_07_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_08_GPIO_Port,COL_08_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_09_GPIO_Port,COL_09_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_10_GPIO_Port,COL_10_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_11_GPIO_Port,COL_11_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_12_GPIO_Port,COL_12_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_13_GPIO_Port,COL_13_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_14_GPIO_Port,COL_14_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_15_GPIO_Port,COL_15_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_16_GPIO_Port,COL_16_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_17_GPIO_Port,COL_17_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_18_GPIO_Port,COL_18_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_19_GPIO_Port,COL_19_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_20_GPIO_Port,COL_20_Pin,GPIO_PIN_RESET);
}


void HC138_Set_Col(int col){
	switch (col)
  {
  	case 0 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_01_GPIO_Port,COL_01_Pin,GPIO_PIN_SET);
  		break;
  	case 1 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_02_GPIO_Port,COL_02_Pin,GPIO_PIN_SET);
  		break;
  	case 2 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_03_GPIO_Port,COL_03_Pin,GPIO_PIN_SET);
  		break;
  	case 3 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_04_GPIO_Port,COL_04_Pin,GPIO_PIN_SET);
  		break;
  	case 4 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_05_GPIO_Port,COL_05_Pin,GPIO_PIN_SET);
  		break;
  	case 5 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_06_GPIO_Port,COL_06_Pin,GPIO_PIN_SET);
  		break;
  	case 6 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_07_GPIO_Port,COL_07_Pin,GPIO_PIN_SET);
  		break;
  	case 7 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_08_GPIO_Port,COL_08_Pin,GPIO_PIN_SET);
  		break;
  	case 8 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_09_GPIO_Port,COL_09_Pin,GPIO_PIN_SET);
  		break;
  	case 9 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_10_GPIO_Port,COL_10_Pin,GPIO_PIN_SET);
  		break;
  	case 10:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_11_GPIO_Port,COL_11_Pin,GPIO_PIN_SET);
  		break;
  	case 11:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_12_GPIO_Port,COL_12_Pin,GPIO_PIN_SET);
  		break;
  	case 12:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_13_GPIO_Port,COL_13_Pin,GPIO_PIN_SET);
  		break;
  	case 13:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_14_GPIO_Port,COL_14_Pin,GPIO_PIN_SET);
  		break;
  	case 14:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_15_GPIO_Port,COL_15_Pin,GPIO_PIN_SET);
  		break;
  	case 15:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_16_GPIO_Port,COL_16_Pin,GPIO_PIN_SET);
  		break;
  	case 16:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_17_GPIO_Port,COL_17_Pin,GPIO_PIN_SET);
  		break;
  	case 17:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_18_GPIO_Port,COL_18_Pin,GPIO_PIN_SET);
  		break;
  	case 18:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_19_GPIO_Port,COL_19_Pin,GPIO_PIN_SET);
  		break;
  	case 19:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_20_GPIO_Port,COL_20_Pin,GPIO_PIN_SET);
  		break;
  	default:
  		break;
  }
}

void HC138_ReSet_Col(int col){
	switch (col)
  {
  	case 0 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_01_GPIO_Port,COL_01_Pin,GPIO_PIN_RESET);
  		break;
  	case 1 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_02_GPIO_Port,COL_02_Pin,GPIO_PIN_RESET);
  		break;
  	case 2 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_03_GPIO_Port,COL_03_Pin,GPIO_PIN_RESET);
  		break;
  	case 3 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_04_GPIO_Port,COL_04_Pin,GPIO_PIN_RESET);
  		break;
  	case 4 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_05_GPIO_Port,COL_05_Pin,GPIO_PIN_RESET);
  		break;
  	case 5 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_06_GPIO_Port,COL_06_Pin,GPIO_PIN_RESET);
  		break;
  	case 6 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_07_GPIO_Port,COL_07_Pin,GPIO_PIN_RESET);
  		break;
  	case 7 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_08_GPIO_Port,COL_08_Pin,GPIO_PIN_RESET);
  		break;
  	case 8 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_09_GPIO_Port,COL_09_Pin,GPIO_PIN_RESET);
  		break;
  	case 9 :
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_10_GPIO_Port,COL_10_Pin,GPIO_PIN_RESET);
  		break;
  	case 10:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_11_GPIO_Port,COL_11_Pin,GPIO_PIN_RESET);
  		break;
  	case 11:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_12_GPIO_Port,COL_12_Pin,GPIO_PIN_RESET);
  		break;
  	case 12:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_13_GPIO_Port,COL_13_Pin,GPIO_PIN_RESET);
  		break;
  	case 13:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_14_GPIO_Port,COL_14_Pin,GPIO_PIN_RESET);
  		break;
  	case 14:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_15_GPIO_Port,COL_15_Pin,GPIO_PIN_RESET);
  		break;
  	case 15:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_16_GPIO_Port,COL_16_Pin,GPIO_PIN_RESET);
  		break;
  	case 16:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_17_GPIO_Port,COL_17_Pin,GPIO_PIN_RESET);
  		break;
  	case 17:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_18_GPIO_Port,COL_18_Pin,GPIO_PIN_RESET);
  		break;
  	case 18:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_19_GPIO_Port,COL_19_Pin,GPIO_PIN_RESET);
  		break;
  	case 19:
			//HC138_Res_All_Cols();
			HAL_GPIO_WritePin(COL_20_GPIO_Port,COL_20_Pin,GPIO_PIN_RESET);
  		break;
  	default:
  		break;
  }
}


void HC138_Set_Row(int row){
	switch (row)
  {
  	case 0 :
			__HC138_A_OFF();
			__HC138_B_OFF();
			__HC138_C_OFF();
  		break;
  	case 1 :
			__HC138_A_ON();
			__HC138_B_OFF();
			__HC138_C_OFF();
  		break;
  	case 2 :
			__HC138_A_OFF();
			__HC138_B_ON();
			__HC138_C_OFF();
  		break;
  	case 3 :
			__HC138_A_ON();
			__HC138_B_ON();
			__HC138_C_OFF();
  		break;
  	case 4 :
			__HC138_A_OFF();
			__HC138_B_OFF();
			__HC138_C_ON();
  		break;
  	case 5 :
			__HC138_A_ON();
			__HC138_B_OFF();
			__HC138_C_ON();
  		break;
  	case 6 :
			__HC138_A_OFF();
			__HC138_B_ON();
			__HC138_C_ON();
  		break;
  	default:
  		break;
  }
}
