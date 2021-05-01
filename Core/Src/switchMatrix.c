#include "switchMatrix.h"

void bWriteColumnState(int column, uint8_t state){
	switch (column){
		case 1 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,state);
				break;
		case 2 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,state);
				break;
		case 3:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,state);
				break;
		case 4 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,state);
				break;
		case 5 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,state);
				break;
		case 6 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,state);
				break;
		case 7 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,state);
				break;
		case 8 :
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,state);
				break;
			}
}


int bReadRowState(int row){
	int state=0;
	switch (row){
		case 1 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
			break;
		case 2 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
			break;
		case 3:
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
			break;
		case 4 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
			break;
		case 5 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
			break;
		case 6 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
			break;
		case 7 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
			break;
		case 8 :
			state=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
			break;
		case 9:
			state=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
			break;
	}
	return state;
}


int intReadRowState(void){
	int i,row=0;
	for (i=1;i<=9;i++)
	{
		if (bReadRowState(i))
		{
			row=i;
			break;
		}
	}
	return row;
}



