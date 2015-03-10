#include <mytasks.h>




void balance_bar(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	ModeInit();
	xLastWakeTime = xTaskGetTickCount(); 

	while(1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		
		ExecuteMode(GetMode());
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, 5/ portTICK_RATE_MS);
	}	
}





