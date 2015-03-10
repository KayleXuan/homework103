/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @version V1.0.0
  * @date    8/3/2015
  * @brief   位置式PID算法实现
  ******************************************************************************
  * @attention
	*
  *	none
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:选择使用Kp,T,Ti,Td还是使用Kp,Ki,Kd参数,如使用Kp,T,Ti,Td参数，在PID.C取消注释
	*		#define USE_Traditional_PID。
	*	2:声明PIDStructInitTypeDef结构体并进行配置。
	*	3:声明PIDStructInitTypeDef结构体。
	* 4:用PID_Init函数初始化PID变量。
	*	5:loop:用PID_Cal()计算。
	******************************************************************************
  */ 


//#define USE_Traditional_PID





#include <PID.h>







void PID_Init(PIDStructTypeDef* PIDStruct, PIDInitStructTypeDef* PIDInitStruct)
{
	#ifdef USE_Traditional_PID
	//A = Kp*(1+T/Ti+Td/Ti)
	//B = Kp(1+2*Td/T)
	//C = Kp*Td/T
	PIDStruct->A = PIDInitStruct->Kp*(1+PIDInitStruct->T/PIDInitStruct->Ti+PIDInitStruct->Td/PIDInitStruct->T);
	PIDStruct->B = PIDInitStruct->Kp*(1+2*PIDInitStruct->Td/PIDInitStruct->T);
	PIDStruct->C = PIDInitStruct->Kp*PIDInitStruct->Td/PIDInitStruct->T;
	#else
	//A = Kp+Ki+Kd
	//B = Kp+2Kd
	//C = Kd
	PIDStruct->A = PIDInitStruct->Kp + PIDInitStruct->Ki + PIDInitStruct->Kd;
	PIDStruct->B = PIDInitStruct->Kp + 2*PIDInitStruct->Kd;
	PIDStruct->C = PIDInitStruct->Kd;
	#endif
	
	PIDStruct->Ea = 0;
	PIDStruct->Eb = 0;
	PIDStruct->Ec = 0;
	PIDStruct->TargetValue = PIDInitStruct->TargetValue;
	PIDStruct->PresentValue = PIDInitStruct->TargetValue;
	PIDStruct->OutputValue = 0;
	PIDStruct->OutputMax = PIDInitStruct->OutputMax;
	PIDStruct->OutputMin = PIDInitStruct->OutputMin;
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
 
  if(PIDStruct->OutputValue > PIDStruct->OutputMax) 
		PIDStruct->OutputValue = PIDStruct->OutputMax;    //输出值限幅
  if(PIDStruct->OutputValue < PIDStruct->OutputMin) 
		PIDStruct->OutputValue = PIDStruct->OutputMin;    //输出值限幅
}

