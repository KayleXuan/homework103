/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @brief   位置式PID算法实现
  ******************************************************************************
  * @attention
	*
	*
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:声明PIDInitStructTypeDef结构体并进行配置。
	* 2:用PID_Init函数初始化PID变量。
	*	3:loop:用PID_Cal()计算。
	******************************************************************************
	*	@original
	*	@version 	V1.0
  * @date    	8/3/2015
	*	@info			1:初稿完成
	*
	*	@update
	*	@version 	V1.1
  * @date    	11/3/2015
	*	@info 		1:修复了输出限幅会导致静态误差的Bug
	*						2:取消了少见的USE_Traditional_PID的宏定义
	*
	*	@update
	*	@version 	V1.2
  * @date    	12/3/2015
	*	@info 		1:改增量式PID为位置式PID，增加了一些用于调试的量
	*						2:取消了PIDInitStructTypeDef
	*/ 



#include <PID.h>




//初始化PID变量
void PID_Init(PIDStructTypeDef* PIDStruct,double TargetValue, double Kp, double Ki, double Kd, double omax, double omin)
{
	PIDStruct->TargetValue = TargetValue;
	PIDStruct->PresentValue = TargetValue;
	PIDStruct->Kp = Kp;
	PIDStruct->Ki = Ki;
	PIDStruct->Kd = Kd;
	PIDStruct->OutputMax = omax;
	PIDStruct->OutputMin = omin;
	
	
	PIDStruct->Error = 0;
	PIDStruct->Error_Add = 0;
	PIDStruct->LastError = 0;
	PIDStruct->Output4p = 0;
	PIDStruct->Output4i = 0;
	PIDStruct->Output4d = 0;
	PIDStruct->OutputValue = 0;
}



//计算PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput)
{
	PIDStruct->PresentValue = NewValueInput;
	PIDStruct->Error = PIDStruct->TargetValue - PIDStruct->PresentValue ;       //差值运算
	PIDStruct->Error_Add += PIDStruct->Error;			//误差累加
	
	PIDStruct->Output4p = PIDStruct->Kp*PIDStruct->Error;
	PIDStruct->Output4i = PIDStruct->Ki*PIDStruct->Error_Add;
	PIDStruct->Output4d = PIDStruct->Kd*(PIDStruct->Error - PIDStruct->LastError);
	
	PIDStruct->OutputValue = PIDStruct->Output4p + PIDStruct->Output4i + PIDStruct->Output4d;
	if(PIDStruct->OutputValue > PIDStruct->OutputMax)
		PIDStruct->OutputValue = PIDStruct->OutputMax;
	else if(PIDStruct->OutputValue < PIDStruct->OutputMin)
		PIDStruct->OutputValue = PIDStruct->OutputMin;
	
	
	PIDStruct->LastError = PIDStruct->Error;
}

