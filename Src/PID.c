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
}

