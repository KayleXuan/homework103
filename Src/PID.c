/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @brief   位置式PID算法实现
  ******************************************************************************
  * @attention
	*
  *	Never Change OutputValue in PIDStructTypeDef!!!!!!!!!!!!!!
	* Please Limit Output Outside PID_Cal() if necessary
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:声明PIDInitStructTypeDef结构体并进行配置。
	*	2:声明PIDStructTypeDef结构体。
	* 3:用PID_Init函数初始化PID变量。
	*	4:loop:用PID_Cal()计算。
	******************************************************************************
	*	@original
	*	@version V1.0
  * @date    8/3/2015
	*
	*	@update
	*	@version V1.1
  * @date    11/3/2015
	*	@info 1:修复了输出限幅会导致静态误差的Bug
	*				2:取消了少见的USE_Traditional_PID的宏定义
	*/ 



#include <PID.h>







void PID_Init(PIDStructTypeDef* PIDStruct, PIDInitStructTypeDef* PIDInitStruct)
{
	//A = Kp+Ki+Kd
	//B = Kp+2Kd
	//C = Kd
	PIDStruct->A = PIDInitStruct->Kp + PIDInitStruct->Ki + PIDInitStruct->Kd;
	PIDStruct->B = PIDInitStruct->Kp + 2*PIDInitStruct->Kd;
	PIDStruct->C = PIDInitStruct->Kd;

	PIDStruct->Ea = 0;
	PIDStruct->Eb = 0;
	PIDStruct->Ec = 0;
	PIDStruct->TargetValue = PIDInitStruct->TargetValue;
	PIDStruct->PresentValue = PIDInitStruct->TargetValue;
	PIDStruct->OutputValue = 0;
}



//计算PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput)
{
	double adj;
	PIDStruct->PresentValue = NewValueInput;
	PIDStruct->Ea =PIDStruct->TargetValue - PIDStruct->PresentValue ;       //差值运算

	adj = PIDStruct->A * PIDStruct->Ea - PIDStruct->B * PIDStruct->Eb + PIDStruct->C*PIDStruct->Ec;//delta计算
	PIDStruct->Ec = PIDStruct->Eb;
	PIDStruct->Eb = PIDStruct->Ea;
	
  PIDStruct->OutputValue += adj;        //调整输出
}

