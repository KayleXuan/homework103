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



#include <PID.h>




//��ʼ��PID����
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



//����PID
void PID_Cal(PIDStructTypeDef* PIDStruct, double NewValueInput)
{
	PIDStruct->PresentValue = NewValueInput;
	PIDStruct->Error = PIDStruct->TargetValue - PIDStruct->PresentValue ;       //��ֵ����
	PIDStruct->Error_Add += PIDStruct->Error;			//����ۼ�
	
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

