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
	*	1:配置使用Kp,T,Ti,Td还是使用Kp,Ki,Kd参数,如使用Kp,T,Ti,Td参数，在PID.C取消注释
	*		#define USE_Traditional_PID。
	*	2:声明PIDInitStructTypeDef结构体并进行配置。
	*	3:声明PIDStructTypeDef结构体。
	* 4:用PID_Init函数初始化PID变量。
	*	5:loop:用PID_Cal()计算。
	******************************************************************************
	*/ 




#ifndef PID_H
	#define PID_H

#include <stm32f1xx_hal.h>





////////////////////////////////////////////////////PID说明/////////////////////////////////////////////////////
//u(t)——控制器(也称调节器)的输出；
//e(t)——控制器的输入（常常是设定值与被控量之差，即e(t)=r(t)-c(t)）；
//Kp——控制器的比例放大系数；
//Ti ——控制器的积分时间
//Td——控制器的微分时间。



//增量式PID
//deltaU(k) = A*e(k) - B*e(k-1) + C*e(k-2)
//U(k) = U(k-1) + delatU(k)
//A = Kp*(1+T/Ti+Td/T)
//B = Kp(1+2*Td/T)
//C = Kp*Td/T
//一般情况下T已经确定，所以改变Kp，Ti，Td即可



//或者使用Kp,Ki,Kd系数
//其中
//Kp = Kp
//Ki = Kp*T/Ti
//Kd = Kp*Td/T
//A = Kp+Ki+Kd
//B = Kp+2Kd
//C = Kd



//deltaU(k) = A*e(k) - B*e(k-1) + C*e(k-2)
////////////////////////////////////////////////////END_PID说明/////////////////////////////////////////////////////




//PID初始化变量

typedef struct
{
	int16_t TargetValue;		

#ifdef USE_Traditional_PID
	double Kp;		//P系数
	double T;     //处理周期T
	double Ti;		//Ti系数
	double Td;		//Td系数
#else
	double Kp;		//P系数
	double Ki;		//I系数
	double Kd;		//D系数
#endif
//输出值限幅
	int16_t OutputMax;
	int16_t OutputMin;
	
} PIDInitStructTypeDef;








//PID结构体声明
typedef struct
{
//每次手动改变的量	
	double TargetValue;	
	double PresentValue;	

//初始化时计算的量
	double A;		//A系数
	double B;		//B系数
	double C;		//C系数
	
	//输出值限幅
	int16_t OutputMax;
	int16_t OutputMin;
	
	
//SpeedPID_Cal函数计算出的量
	double Ea;	//本次误差
	double Eb;	//上次误差
	double Ec;	//上上次误差	
	double OutputValue;	//输出执行量				
} PIDStructTypeDef;




//初始化PID变量
void PID_Init(PIDStructTypeDef* PIDStruct, PIDInitStructTypeDef* PIDInitStruct);
//计算PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput);








#endif

