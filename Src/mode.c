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

int32_t TIM4CNT;



//摆动杆子，使其达到0位置
void SwayUp(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	if((TIM2->CNT>512*45/360) &&	(TIM2->CNT<512*315/360))
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		return;//起始角度太大，直接跳出
	}
	SetPWM(-1000);
	PIDBar.OutputValue =  -1000;
	while(TIM2->CNT <= (512*11/360))//等待到达45°位置
		PIDBar.PresentValue = TIM2->CNT;
	SetPWM(0);
	PIDBar.OutputValue =  0;
	while(TIM2->CNT <= (512*180/360))//等待杆子甩上来
		PIDBar.PresentValue = TIM2->CNT;
//	SetPWM(+1000);
//	while(TIM2->CNT <= (512*120/360))
		;
	//杆子已竖直
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
}










uint8_t GetMode(void)
{
	uint8_t mode;
	mode = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15));
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
}
///////////////////////////////////////////////END MODE 2222222222/////////////////////////////








/////////////////////////////////////////////////MODE 33333333333333/////////////////////////////
//(mode3)（1）从摆杆处于自然下垂状态开始，控制旋转臂作往复旋转运动，尽快使摆杆摆起倒立，保持倒立状态时间不少于10s；
double tarvforPIDBar;
void mode3(void)
{
	int32_t temp;
	
	if((TIM2->CNT > (256+128)) || (TIM2->CNT < (256-128)))//不可控
	{
		SetPWM(0);
		LED_OFF;
		while(1);
	}
	
		
	temp = TIM4->CNT;
	TIM4->CNT = 0xffff/2;
	TIM4CNT += temp-0xffff/2;
	PID_Cal(&PIDDir,TIM4CNT);
	
	
	PIDBar.TargetValue = tarvforPIDBar - PIDDir.OutputValue;
	
	PID_Cal(&PIDBar,TIM2->CNT);
	
	SetPWM(PIDBar.OutputValue);
}
//////////////////////////////////////////////END MODE 33333333333333/////////////////////////////






/////////////////////////////////////////////////MODE 444444444444444/////////////////////////////
//(mode4)（2）在摆杆保持倒立状态下，施加干扰后摆杆能继续保持倒立或2s内恢复 倒立状态； 
void mode4(void)
{
	mode3();
}
//////////////////////////////////////////////END MODE 44444444444444/////////////////////////////






/////////////////////////////////////////////////MODE 555555555555555/////////////////////////////
//(mode5)（3）在摆杆保持倒立状态的前提下，旋转臂作圆周运动，并尽快使单方向 转过角度达到或超过360°； 
void mode5(void)
{
	TIM4->CNT --;
	mode3();
}
/////////////////////////////////////////////END MODE 555555555555555/////////////////////////////







/////////////////////////////////////////////////MODE 666666666666666/////////////////////////////
//(mode6)（4）其他。
void mode6(void)
{
}
/////////////////////////////////////////////END MODE 6666666666666666/////////////////////////////








/////////////////////////////////////////////////MODE default/////////////////////////////////////
//电机停止转动
void modedefault(void)
{
	SetPWM(0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	while(1);
}
/////////////////////////////////////////////END MODE default//////////////////////////////////////




