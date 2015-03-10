/*
1������Ҫ��  
(mode0)��1���ڸ˴Ӵ�����Ȼ�´�״̬���ڽ�0�㣩��ʼ���������������ת���� ������תʹ�ڸ˰ڶ���������ʹ�ڽǴﵽ�򳬹�-60��~ +60�㣻  
(mode1)��2���Ӱڸ˴�����Ȼ�´�״̬��ʼ����������ڸ˵İڶ����ȣ�ֱ����� Բ���˶���   
(mode2)��3���ڰڸ˴�����Ȼ�´�״̬�£���������ڸ����ӽ�165��λ�ã����� ����ͬʱ��
					����������ת��ʹ�ڸ˱��ֵ���״̬ʱ�䲻����5s���ڼ���ת�۵�ת���ǶȲ�����90�㡣 

2�����Ӳ���  
(mode3)��1���Ӱڸ˴�����Ȼ�´�״̬��ʼ��������ת����������ת�˶�������ʹ �ڸ˰����������ֵ���״̬ʱ�䲻����10s��   
(mode4)��2���ڰڸ˱��ֵ���״̬�£�ʩ�Ӹ��ź�ڸ��ܼ������ֵ�����2s�ڻָ� ����״̬��  
(mode5)��3���ڰڸ˱��ֵ���״̬��ǰ���£���ת����Բ���˶���������ʹ������ ת���Ƕȴﵽ�򳬹�360�㣻 
(mode6)��4��������
*/




#include <mode.h>




PIDStructTypeDef PIDBar;//�ڸ�PID
PIDStructTypeDef PIDDir;//�����PID����ֹ����ת̫�ࣩ
PIDInitStructTypeDef PIDInitStruct;//��ʼ������


void ModeInit(void)
{
//	PIDInitStruct.Kp = 0;
//	PIDInitStruct.Ki = 0;
//	PIDInitStruct.Kd = 0;

//	PIDInitStruct.Kp = 60;
//	PIDInitStruct.Ki = 0;
//	PIDInitStruct.Kd = 100;

	//Config PIDBar
	PIDInitStruct.Kp = 20;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 10;
	PIDInitStruct.OutputMax = 1000;
	PIDInitStruct.OutputMin = -1000;
	PIDInitStruct.TargetValue = 255;
	PID_Init(&PIDBar, &PIDInitStruct);
	TIM2->CNT = 255;	
	
	
	//Config PIDDir
//	PIDInitStruct.Kp = 0.01;
	PIDInitStruct.Kp = 0.00;
	PIDInitStruct.Ki = 0;
	PIDInitStruct.Kd = 0;
	PIDInitStruct.OutputMax = 1000;
	PIDInitStruct.OutputMin = -1000;
	PIDInitStruct.TargetValue = 0xffff/2;
	PID_Init(&PIDDir, &PIDInitStruct);
	TIM4->CNT = 0xffff/2;
	
}


uint8_t GetMode(void)
{
	uint8_t mode;
	mode = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
	mode <<=1;
	mode += (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13));
	
	return mode;
}



void ExecuteMode(uint8_t mode)
{
	switch(mode)
	{
		case 0:		mode0(); break;
		case 1:		mode1(); break;
		case 2:		mode2(); break;
		case 3:		mode3(); break;
		case 4:		mode4(); break;
		case 5:		mode5(); break;
		case 6:		mode6(); break;
		default:	modedefault(); break;
	}	
}



//////////////////////////////////////////////////MODE 00000000000/////////////////////////////
//(mode0)��1���ڸ˴Ӵ�����Ȼ�´�״̬���ڽ�0�㣩��ʼ���������������ת���� ������תʹ�ڸ˰ڶ���������ʹ�ڽǴﵽ�򳬹�-60��~ +60�㣻
void mode0(void)
{
}
///////////////////////////////////////////////END MODE 00000000000/////////////////////////////







//////////////////////////////////////////////////MODE 11111111111/////////////////////////////
//(mode1)��2���Ӱڸ˴�����Ȼ�´�״̬��ʼ����������ڸ˵İڶ����ȣ�ֱ����� Բ���˶���   
void mode1(void)
{
}
///////////////////////////////////////////////END MODE 11111111111/////////////////////////////






//////////////////////////////////////////////////MODE 2222222222/////////////////////////////
//(mode2)��3���ڰڸ˴�����Ȼ�´�״̬�£���������ڸ����ӽ�165��λ�ã����� ����ͬʱ��
//						����������ת��ʹ�ڸ˱��ֵ���״̬ʱ�䲻����5s���ڼ���ת�۵�ת���ǶȲ�����90�㡣 
void mode2(void)
{
	PID_Cal(&PIDBar,TIM2->CNT);
	PID_Cal(&PIDDir,TIM4->CNT);
	SetPWM(PIDBar.OutputValue + PIDDir.OutputValue);
}
///////////////////////////////////////////////END MODE 2222222222/////////////////////////////








/////////////////////////////////////////////////MODE 33333333333333/////////////////////////////
//(mode3)��1���Ӱڸ˴�����Ȼ�´�״̬��ʼ��������ת����������ת�˶�������ʹ�ڸ˰����������ֵ���״̬ʱ�䲻����10s�� 
int twickcount = 0;
void mode3(void)
{
		if((TIM2->CNT<206)||(TIM2->CNT>306))//��PID��Χ֮��
		{
			twickcount++;
			if(twickcount >= 100)
				twickcount = 0;
			if(twickcount<50)
				SetPWM(-400);
			else
				SetPWM(400);
		}
		else
		{
			PID_Cal(&PIDBar,TIM2->CNT);
			SetPWM(PIDBar.OutputValue);
		}
}
//////////////////////////////////////////////END MODE 33333333333333/////////////////////////////






/////////////////////////////////////////////////MODE 444444444444444/////////////////////////////
//(mode4)��2���ڰڸ˱��ֵ���״̬�£�ʩ�Ӹ��ź�ڸ��ܼ������ֵ�����2s�ڻָ� ����״̬�� 
void mode4(void)
{
}
//////////////////////////////////////////////END MODE 44444444444444/////////////////////////////






/////////////////////////////////////////////////MODE 555555555555555/////////////////////////////
//(mode5)��3���ڰڸ˱��ֵ���״̬��ǰ���£���ת����Բ���˶���������ʹ������ ת���Ƕȴﵽ�򳬹�360�㣻 
void mode5(void)
{
}
/////////////////////////////////////////////END MODE 555555555555555/////////////////////////////







/////////////////////////////////////////////////MODE 666666666666666/////////////////////////////
//(mode6)��4��������
void mode6(void)
{
	modedefault();
}
/////////////////////////////////////////////END MODE 6666666666666666/////////////////////////////








/////////////////////////////////////////////////MODE default/////////////////////////////////////
//���ֹͣת��
void modedefault(void)
{
	SetPWM(0);
}
/////////////////////////////////////////////END MODE default//////////////////////////////////////




