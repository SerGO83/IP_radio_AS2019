#include "stdio.h"
#include "stm32f7xx_hal.h"

#define MAXnTASKS  333			//количество задач

typedef enum
{
	TASK_OK					= 0x00U,
	TASK_LIMIT			= 0x01U
} Task_StatusTypeDef;

typedef struct
{
	void (*routine)(void);
	
	uint32_t period;
	
	uint32_t lastTimeStart;
	
	_Bool needRun;
	
} Task_TypeDef;

typedef struct
{
//	uint32_t maxNTask;  //в этом параметре будет храниться значение количества задач
	uint32_t index;			//текщее значение задач, нужно для инициализации
	
} Task_InitTypeDef;


Task_StatusTypeDef SG_Task_add ( void (*routine)(void), int period );
//Task_StatusTypeDef SG_Task_Init (Task_TypeDef *Task);
void updateTimerTask(void);
void checkTask (void);
void runTask (void);
