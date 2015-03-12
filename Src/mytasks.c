#include <mytasks.h>
#include <Motor.h>



void balance_bar(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	uint8_t mode;
	xLastWakeTime = xTaskGetTickCount();

	
	
	vTaskDelayUntil(&xLastWakeTime, 2000/portTICK_RATE_MS);//wait for 2s
	
	
	
	mode = GetMode();
	
	
	//Config PIDBar
	PID_Init(&PIDBar, 256, 35, 0, 43, 1000, -1000);
//	PIDInitStruct.Kp = 35;
//	PIDInitStruct.Ki = 0.00;
//	PIDInitStruct.Kd = 43;
//	PIDInitStruct.TargetValue = 256;
//	PID_Init(&PIDBar, &PIDInitStruct);
	TIM2->CNT = 0;
//TIM2->CNT = 255;



	if((mode != 0)&&(mode != 1)&&(mode != 5))
		SwayUp();





//	SetPWM(0);
//	while(1);







	//Config PIDDir
	PID_Init(&PIDDir, 0, 0, 0, 0.002, 5, -5);
//	PIDInitStruct.Kp = 0;
//	PIDInitStruct.Ki = 0;
//	PIDInitStruct.Kd = 0.002;
//	PIDInitStruct.TargetValue = 0xffff/2;
//	PID_Init(&PIDDir, &PIDInitStruct);
	TIM4CNT = 0;
	TIM4->CNT = 0xffff/2;
	
	
	while(1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		
		
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
		{
			SetPWM(0);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
			while(1);
		}
		

		mode = 3;
		ExecuteMode(mode);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//		vTaskDelayUntil(&xLastWakeTime, 5/ portTICK_RATE_MS);
		vTaskDelayUntil(&xLastWakeTime, 2/ portTICK_RATE_MS);
	}
}





