#include <mytasks.h>
#include <Motor.h>






PIDInitStructTypeDef PIDInitStruct;//初始化变量


void balance_bar(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	uint8_t mode;
	xLastWakeTime = xTaskGetTickCount();

	
	
	vTaskDelayUntil(&xLastWakeTime, 2000/portTICK_RATE_MS);//wait for 2s
	
	
	
	mode = GetMode();
	
	
	//Config PIDBar
	PIDInitStruct.Kp = 20;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 10;
	PIDInitStruct.TargetValue = 255;
	PID_Init(&PIDBar, &PIDInitStruct);
	TIM2->CNT = 0;	

	//if//////////////////////////////////////////
	SwayUp();/////////////////////////////////////////////////





			SetPWM(0);
			while(1);







	//Config PIDDir
	PIDInitStruct.Kp = 0;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 0.001;
	PIDInitStruct.TargetValue = 0xffff/2;
	PID_Init(&PIDDir, &PIDInitStruct);
	TIM4->CNT = 0xffff/2;
	
	
	while(1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
		{
			SetPWM(0);
			while(1);
		}

		ExecuteMode(mode);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, 5/ portTICK_RATE_MS);
	}
}





