/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @version V1.0.0
  * @date    8/3/2015
  * @brief   λ��ʽPID�㷨ʵ��
  ******************************************************************************
  * @attention
	*
  *	none
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:����ʹ��Kp,T,Ti,Td����ʹ��Kp,Ki,Kd����,��ʹ��Kp,T,Ti,Td��������PID.Cȡ��ע��
	*		#define USE_Traditional_PID��
	*	2:����PIDInitStructTypeDef�ṹ�岢�������á�
	*	3:����PIDStructTypeDef�ṹ�塣
	* 4:��PID_Init������ʼ��PID������
	*	5:loop:��PID_Cal()���㡣
	******************************************************************************
	*/ 




#ifndef PID_H
	#define PID_H

#include <stm32f1xx_hal.h>





////////////////////////////////////////////////////PID˵��/////////////////////////////////////////////////////
//u(t)����������(Ҳ�Ƶ�����)�������
//e(t)���������������루�������趨ֵ�뱻����֮���e(t)=r(t)-c(t)����
//Kp�����������ı����Ŵ�ϵ����
//Ti �����������Ļ���ʱ��
//Td������������΢��ʱ�䡣



//����ʽPID
//deltaU(k) = A*e(k) - B*e(k-1) + C*e(k-2)
//U(k) = U(k-1) + delatU(k)
//A = Kp*(1+T/Ti+Td/T)
//B = Kp(1+2*Td/T)
//C = Kp*Td/T
//һ�������T�Ѿ�ȷ�������Ըı�Kp��Ti��Td����



//����ʹ��Kp,Ki,Kdϵ��
//����
//Kp = Kp
//Ki = Kp*T/Ti
//Kd = Kp*Td/T
//A = Kp+Ki+Kd
//B = Kp+2Kd
//C = Kd



//deltaU(k) = A*e(k) - B*e(k-1) + C*e(k-2)
////////////////////////////////////////////////////END_PID˵��/////////////////////////////////////////////////////




//PID��ʼ������

typedef struct
{
	int16_t TargetValue;		

#ifdef USE_Traditional_PID
	double Kp;		//Pϵ��
	double T;     //��������T
	double Ti;		//Tiϵ��
	double Td;		//Tdϵ��
#else
	double Kp;		//Pϵ��
	double Ki;		//Iϵ��
	double Kd;		//Dϵ��
#endif
//���ֵ�޷�
	int16_t OutputMax;
	int16_t OutputMin;
	
} PIDInitStructTypeDef;








//PID�ṹ������
typedef struct
{
//ÿ���ֶ��ı����	
	double TargetValue;	
	double PresentValue;	

//��ʼ��ʱ�������
	double A;		//Aϵ��
	double B;		//Bϵ��
	double C;		//Cϵ��
	
	//���ֵ�޷�
	int16_t OutputMax;
	int16_t OutputMin;
	
	
//SpeedPID_Cal�������������
	double Ea;	//�������
	double Eb;	//�ϴ����
	double Ec;	//���ϴ����	
	double OutputValue;	//���ִ����				
} PIDStructTypeDef;




//��ʼ��PID����
void PID_Init(PIDStructTypeDef* PIDStruct, PIDInitStructTypeDef* PIDInitStruct);
//����PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput);








#endif

