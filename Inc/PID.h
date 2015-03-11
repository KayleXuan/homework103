/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @brief   λ��ʽPID�㷨ʵ��
  ******************************************************************************
  * @attention
	*
  *	Never Change OutputValue in PIDStructTypeDef!!!!!!!!!!!!!!
	* Please Limit Output Outside PID_Cal() if necessary
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:����PIDInitStructTypeDef�ṹ�岢�������á�
	*	2:����PIDStructTypeDef�ṹ�塣
	* 3:��PID_Init������ʼ��PID������
	*	4:loop:��PID_Cal()���㡣
	******************************************************************************
	*	@original
	*	@version V1.0
  * @date    8/3/2015
	*
	*	@update
	*	@version V1.1
  * @date    11/3/2015
	*	@info 1:�޸�������޷��ᵼ�¾�̬����Bug
	*				2:ȡ�����ټ���USE_Traditional_PID�ĺ궨��
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
	double Kp;		//Pϵ��
	double Ki;		//Iϵ��
	double Kd;		//Dϵ��
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
	
	
//SpeedPID_Cal�������������(���Բ�Ҫ�ֶ��ı�����!!!)
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

