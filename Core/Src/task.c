#include "task.h"
#include "stdint.h"

uint32_t delta;
uint16_t i;
Task_TypeDef TaskArray[ MAXnTASKS ];  //максимальное количество задач задается параметром MAXnTASKS в .h файле
uint16_t TaskIndex = 0;   //счетчик задач, используется в функции SG_Task_Add


Task_StatusTypeDef SG_Task_add (void (*routine)(void), int period){

	
	if (TaskIndex<MAXnTASKS)
		{
			TaskArray[TaskIndex].routine = routine;
			TaskArray[TaskIndex].period = period;
			TaskIndex++;
		}else
		{
			Error_Handler();
			return TASK_LIMIT;
		}
	return TASK_OK;
};


//функция проверяет натикал ли системный таймер для выполнения задачи, если да то устанавливается флаг needRun, который позже проверяется
void checkTask (void){
	i = 0;
	while (TaskArray[i].routine != NULL){
	  delta = HAL_GetTick() - TaskArray[i].lastTimeStart;
		if (delta >= TaskArray[i].period) {
			TaskArray[i].needRun = 1;
			TaskArray[i].lastTimeStart = HAL_GetTick();
		}
		i++;
	}
};

//функция проверяет необходимость запуска задачи, проверкой флага needRun, если установлен, то задача запускается, и флаг сбрасывается.
void runTask (void) { 
	i = 0;
	while (TaskArray[i].routine != NULL){
		if (TaskArray[i].needRun) {
					TaskArray[i].needRun = 0;
					TaskArray[i].routine();
		}
		i++;
	}
}

/*
функция заполняет в структуру задач текущее значение времени.
заполняется не весь массив TaskArray, а только заполненные элементы
*/
void updateTimerTask(void){
	i=0;
	while (TaskArray[i].routine != NULL){
		TaskArray[i].lastTimeStart = HAL_GetTick();
		i++;
	}
}

