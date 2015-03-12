/*
1������Ҫ��  
(mode0)��1���ڸ˴Ӵ�����Ȼ�´�״̬���ڽ�0�㣩��ʼ���������������ת���� ������תʹ�ڸ˰ڶ���������ʹ�ڽǴﵽ�򳬹�-60��~ +60�㣻  
(mode1)��2���Ӱڸ˴�����Ȼ�´�״̬��ʼ����������ڸ˵İڶ����ȣ�ֱ����� Բ���˶���   
(mode2)��3���ڰڸ˴�����Ȼ�´�״̬�£���������ڸ����ӽ�165��λ�ã����� ����ͬʱ������������ת��ʹ�ڸ˱��ֵ���״̬ʱ�䲻����5s���ڼ���ת�۵�ת���ǶȲ�����90�㡣 

2�����Ӳ���  
(mode3)��1���Ӱڸ˴�����Ȼ�´�״̬��ʼ��������ת����������ת�˶�������ʹ �ڸ˰����������ֵ���״̬ʱ�䲻����10s��   
(mode4)��2���ڰڸ˱��ֵ���״̬�£�ʩ�Ӹ��ź�ڸ��ܼ������ֵ�����2s�ڻָ� ����״̬��  
(mode5)��3���ڰڸ˱��ֵ���״̬��ǰ���£���ת����Բ���˶���������ʹ������ ת���Ƕȴﵽ�򳬹�360�㣻 
(mode6)��4��������

����start���ڿ����Ƿ�ת��(���Ͳ������)

����IO�ھ�������1Ϊ�͵�ƽ
*/


#ifndef MODE_H
	#define MODE_H

#include <stm32f1xx_hal.h>
#include <PID.h>
#include <Motor.h>

extern PIDStructTypeDef PIDBar;//�ڸ�PID
extern PIDStructTypeDef PIDDir;//�����PID����ֹ����ת̫�ࣩ

uint8_t GetMode(void);
void ExecuteMode(uint8_t mode);


void SwayUp(void);












void mode0(void);
void mode1(void);
void mode2(void);	
void mode3(void);
void mode4(void);
void mode5(void);
void mode6(void);
void modedefault(void);


#endif


