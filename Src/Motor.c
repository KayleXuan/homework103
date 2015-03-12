#include <Motor.h>







#define PWM_MAX 1000
//设定电机转速，dutycycle范围为 -999到999
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
	








