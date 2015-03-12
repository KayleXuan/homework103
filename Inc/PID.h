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




#ifndef PID_H
	#define PID_H

#include <stm32f1xx_hal.h>



//PID结构体声明
typedef struct
{

	//可改变的量	
	double TargetValue;	
	double PresentValue;	


		
	double Kp;
	double Ki;
	double Kd;
	
	double OutputMax;
	double OutputMin;
	
	
	
//SpeedPID_Cal函数计算出的量(绝对不要手动改变它们!!!)
	double Error;				//本次误差
	double LastError;		//上次误差
	double Error_Add;		//误差累加项
	double Output4p;		//Kp的输出
	double Output4i;		//Ki的输出
	double Output4d;		//Kd的输出
	double OutputValue;	//输出执行量				
} PIDStructTypeDef;




//初始化PID变量
void PID_Init(PIDStructTypeDef* PIDStruct,double TargetValue, double Kp, double Ki, double Kd, double omax, double omin);
//计算PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput);








#endif

