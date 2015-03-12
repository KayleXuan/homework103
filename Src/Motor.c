#include <Motor.h>







#define PWM_MAX 1000
//�趨���ת�٣�dutycycle��ΧΪ -999��999
void SetPWM(double dutycycle)
{
	if(dutycycle < 0)
	{
		if(dutycycle < -PWM_MAX)
			dutycycle = -PWM_MAX;
		TIM3->CCR1 = -dutycycle;
		TIM3->CCR2 = 0;
	}
	if(dutycycle > 0)
	{
		if(dutycycle > PWM_MAX)
			dutycycle = PWM_MAX;
		TIM3->CCR1 = 0;
		TIM3->CCR2 = dutycycle;
	}
	if(dutycycle == 0)
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
	}
}
	








