/*
1．基本要求  
(mode0)（1）摆杆从处于自然下垂状态（摆角0°）开始，驱动电机带动旋转臂作 往复旋转使摆杆摆动，并尽快使摆角达到或超过-60°~ +60°；  
(mode1)（2）从摆杆处于自然下垂状态开始，尽快增大摆杆的摆动幅度，直至完成 圆周运动；   
(mode2)（3）在摆杆处于自然下垂状态下，外力拉起摆杆至接近165°位置，外力 撤除同时，
					启动控制旋转臂使摆杆保持倒立状态时间不少于5s；期间旋转臂的转动角度不大于90°。 

2．发挥部分  
(mode3)（1）从摆杆处于自然下垂状态开始，控制旋转臂作往复旋转运动，尽快使 摆杆摆起倒立，保持倒立状态时间不少于10s；   
(mode4)（2）在摆杆保持倒立状态下，施加干扰后摆杆能继续保持倒立或2s内恢复 倒立状态；  
(mode5)（3）在摆杆保持倒立状态的前提下，旋转臂作圆周运动，并尽快使单方向 转过角度达到或超过360°； 
(mode6)（4）其他。
*/




#include <mode.h>




PIDStructTypeDef PIDBar;//摆杆PID
PIDStructTypeDef PIDDir;//方向角PID（防止杆旋转太多）
PIDInitStructTypeDef PIDInitStruct;//初始化变量


void ModeInit(void)
{
//	PIDInitStruct.Kp = 0;
//	PIDInitStruct.Ki = 0;
//	PIDInitStruct.Kd = 0;

//	PIDInitStruct.Kp = 60;
//	PIDInitStruct.Ki = 0;
//	PIDInitStruct.Kd = 100;

	//Config PIDBar
	PIDInitStruct.Kp = 20;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 10;
	PIDInitStruct.OutputMax = 1000;
	PIDInitStruct.OutputMin = -1000;
	PIDInitStruct.TargetValue = 255;
	PID_Init(&PIDBar, &PIDInitStruct);
	TIM2->CNT = 255;	
	
	
	//Config PIDDir
//	PIDInitStruct.Kp = 0.01;
	PIDInitStruct.Kp = 0.00;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 0;
	PIDInitStruct.OutputMax = 1000;
	PIDInitStruct.OutputMin = -1000;
	PIDInitStruct.TargetValue = 0xffff/2;
	PID_Init(&PIDDir, &PIDInitStruct);
	TIM4->CNT = 0xffff/2;
	
}


uint8_t GetMode(void)
{
	uint8_t mode;
	mode = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13));
	
	return mode;
}



void ExecuteMode(uint8_t mode)
{
	switch(mode)
	{
		case 0:		mode0(); break;
		case 1:		mode1(); break;
		case 2:		mode2(); break;
		case 3:		mode3(); break;
		case 4:		mode4(); break;
		case 5:		mode5(); break;
		case 6:		mode6(); break;
		default:	modedefault(); break;
	}	
}



//////////////////////////////////////////////////MODE 00000000000/////////////////////////////
//(mode0)（1）摆杆从处于自然下垂状态（摆角0°）开始，驱动电机带动旋转臂作 往复旋转使摆杆摆动，并尽快使摆角达到或超过-60°~ +60°；
void mode0(void)
{
}
///////////////////////////////////////////////END MODE 00000000000/////////////////////////////







//////////////////////////////////////////////////MODE 11111111111/////////////////////////////
//(mode1)（2）从摆杆处于自然下垂状态开始，尽快增大摆杆的摆动幅度，直至完成 圆周运动；   
void mode1(void)
{
}
///////////////////////////////////////////////END MODE 11111111111/////////////////////////////






//////////////////////////////////////////////////MODE 2222222222/////////////////////////////
//(mode2)（3）在摆杆处于自然下垂状态下，外力拉起摆杆至接近165°位置，外力 撤除同时，
//						启动控制旋转臂使摆杆保持倒立状态时间不少于5s；期间旋转臂的转动角度不大于90°。 
void mode2(void)
{
	PID_Cal(&PIDBar,TIM2->CNT);
	PID_Cal(&PIDDir,TIM4->CNT);
	SetPWM(PIDBar.OutputValue + PIDDir.OutputValue);
}
///////////////////////////////////////////////END MODE 2222222222/////////////////////////////








/////////////////////////////////////////////////MODE 33333333333333/////////////////////////////
//(mode3)（1）从摆杆处于自然下垂状态开始，控制旋转臂作往复旋转运动，尽快使摆杆摆起倒立，保持倒立状态时间不少于10s； 
int twickcount = 0;
void mode3(void)
{
		if((TIM2->CNT<206)||(TIM2->CNT>306))//在PID范围之外
		{
			twickcount++;
			if(twickcount >= 100)
				twickcount = 0;
			if(twickcount<50)
				SetPWM(-400);
			else
				SetPWM(400);
		}
		else
		{
			PID_Cal(&PIDBar,TIM2->CNT);
			SetPWM(PIDBar.OutputValue);
		}
}
//////////////////////////////////////////////END MODE 33333333333333/////////////////////////////






/////////////////////////////////////////////////MODE 444444444444444/////////////////////////////
//(mode4)（2）在摆杆保持倒立状态下，施加干扰后摆杆能继续保持倒立或2s内恢复 倒立状态； 
void mode4(void)
{
}
//////////////////////////////////////////////END MODE 44444444444444/////////////////////////////






/////////////////////////////////////////////////MODE 555555555555555/////////////////////////////
//(mode5)（3）在摆杆保持倒立状态的前提下，旋转臂作圆周运动，并尽快使单方向 转过角度达到或超过360°； 
void mode5(void)
{
}
/////////////////////////////////////////////END MODE 555555555555555/////////////////////////////







/////////////////////////////////////////////////MODE 666666666666666/////////////////////////////
//(mode6)（4）其他。
void mode6(void)
{
	modedefault();
}
/////////////////////////////////////////////END MODE 6666666666666666/////////////////////////////








/////////////////////////////////////////////////MODE default/////////////////////////////////////
//电机停止转动
void modedefault(void)
{
	SetPWM(0);
}
/////////////////////////////////////////////END MODE default//////////////////////////////////////




