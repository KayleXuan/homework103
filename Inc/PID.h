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
	double Kp;		//P系数
	double Ki;		//I系数
	double Kd;		//D系数
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
	
	
//SpeedPID_Cal函数计算出的量(绝对不要手动改变它们!!!)
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

