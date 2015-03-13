#include <mytasks.h>
#include <Motor.h>



void balance_bar(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	uint8_t mode;
	xLastWakeTime = xTaskGetTickCount();

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	
	vTaskDelayUntil(&xLastWakeTime, 2000/portTICK_RATE_MS);//wait for 2s
	
	
	
	mode = GetMode();
//	mode = 3;
	
	
	//Config PIDBar
//PID_Init(&PIDBar, 256, 10, 1, 80, 1000, -1000);
	
	
	PID_Init(&PIDBar, 256, 10, 1, 80, 1000, -1000);
	tarvforPIDBar = 256;
	
	
	
	
	
	
	
	

	TIM2->CNT = 0;




	if((mode != 0)&&(mode != 2))
		SwayUp();





//	SetPWM(0);
//	while(1);







	//Config PIDDir
//	PID_Init(&PIDDir,  0, 0.002, 0, 0.02, 10, -10);
	PID_Init(&PIDDir,  0, 0.001, 0, 0.01, 10, -10);
	
	TIM4CNT = 0;
	TIM4->CNT = 0xffff/2;
	
	
	while(1)
	{
		LED_ON;
		
		
//		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
//		{
//			SetPWM(0);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
//			while(1);
//		}
		


		ExecuteMode(mode);
		LED_OFF;
		vTaskDelayUntil(&xLastWakeTime, 5/ portTICK_RATE_MS);
	}
}





