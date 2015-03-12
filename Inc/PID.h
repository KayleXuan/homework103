/**
  ******************************************************************************
  * @file    PID.h
  * @author  Kayle Xuan
  * @brief   λ��ʽPID�㷨ʵ��
  ******************************************************************************
  * @attention
	*
	*
	*
  *
  ******************************************************************************
	* @HowToUse
	*	1:����PIDInitStructTypeDef�ṹ�岢�������á�
	* 2:��PID_Init������ʼ��PID������
	*	3:loop:��PID_Cal()���㡣
	******************************************************************************
	*	@original
	*	@version 	V1.0
  * @date    	8/3/2015
	*	@info			1:�������
	*
	*	@update
	*	@version 	V1.1
  * @date    	11/3/2015
	*	@info 		1:�޸�������޷��ᵼ�¾�̬����Bug
	*						2:ȡ�����ټ���USE_Traditional_PID�ĺ궨��
	*
	*	@update
	*	@version 	V1.2
  * @date    	12/3/2015
	*	@info 		1:������ʽPIDΪλ��ʽPID��������һЩ���ڵ��Ե���
	*						2:ȡ����PIDInitStructTypeDef
	*/ 




#ifndef PID_H
	#define PID_H

#include <stm32f1xx_hal.h>



//PID�ṹ������
typedef struct
{

	//�ɸı����	
	double TargetValue;	
	double PresentValue;	


		
	double Kp;
	double Ki;
	double Kd;
	
	double OutputMax;
	double OutputMin;
	
	
	
//SpeedPID_Cal�������������(���Բ�Ҫ�ֶ��ı�����!!!)
	double Error;				//�������
	double LastError;		//�ϴ����
	double Error_Add;		//����ۼ���
	double Output4p;		//Kp�����
	double Output4i;		//Ki�����
	double Output4d;		//Kd�����
	double OutputValue;	//���ִ����				
} PIDStructTypeDef;




//��ʼ��PID����
void PID_Init(PIDStructTypeDef* PIDStruct,double TargetValue, double Kp, double Ki, double Kd, double omax, double omin);
//����PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput);








#endif

