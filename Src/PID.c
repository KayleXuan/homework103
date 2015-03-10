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
	*	1:ѡ��ʹ��Kp,T,Ti,Td����ʹ��Kp,Ki,Kd����,��ʹ��Kp,T,Ti,Td��������PID.Cȡ��ע��
	*		#define USE_Traditional_PID��
	*	2:����PIDStructInitTypeDef�ṹ�岢�������á�
	*	3:����PIDStructInitTypeDef�ṹ�塣
	* 4:��PID_Init������ʼ��PID������
	*	5:loop:��PID_Cal()���㡣
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



//����PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput)
{
	double adj;
	PIDStruct->PresentValue = NewValueInput;
	PIDStruct->Ea =PIDStruct->TargetValue - PIDStruct->PresentValue ;       //��ֵ����

	adj = PIDStruct->A * PIDStruct->Ea - PIDStruct->B * PIDStruct->Eb + PIDStruct->C*PIDStruct->Ec;//delta����
	PIDStruct->Ec = PIDStruct->Eb;
	PIDStruct->Eb = PIDStruct->Ea;
	
  PIDStruct->OutputValue += adj;        //�������
 
  if(PIDStruct->OutputValue > PIDStruct->OutputMax) 
		PIDStruct->OutputValue = PIDStruct->OutputMax;    //���ֵ�޷�
  if(PIDStruct->OutputValue < PIDStruct->OutputMin) 
		PIDStruct->OutputValue = PIDStruct->OutputMin;    //���ֵ�޷�
}

