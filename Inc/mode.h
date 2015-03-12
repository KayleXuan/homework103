/*
1．基本要求  
(mode0)（1）摆杆从处于自然下垂状态（摆角0°）开始，驱动电机带动旋转臂作 往复旋转使摆杆摆动，并尽快使摆角达到或超过-60°~ +60°；  
(mode1)（2）从摆杆处于自然下垂状态开始，尽快增大摆杆的摆动幅度，直至完成 圆周运动；   
(mode2)（3）在摆杆处于自然下垂状态下，外力拉起摆杆至接近165°位置，外力 撤除同时，启动控制旋转臂使摆杆保持倒立状态时间不少于5s；期间旋转臂的转动角度不大于90°。 

2．发挥部分  
(mode3)（1）从摆杆处于自然下垂状态开始，控制旋转臂作往复旋转运动，尽快使 摆杆摆起倒立，保持倒立状态时间不少于10s；   
(mode4)（2）在摆杆保持倒立状态下，施加干扰后摆杆能继续保持倒立或2s内恢复 倒立状态；  
(mode5)（3）在摆杆保持倒立状态的前提下，旋转臂作圆周运动，并尽快使单方向 转过角度达到或超过360°； 
(mode6)（4）其他。

其中start用于控制是否转动(拉低才有输出)

所有IO口均上拉，1为低电平
*/


#ifndef MODE_H
	#define MODE_H

#include <stm32f1xx_hal.h>
#include <PID.h>
#include <Motor.h>

extern PIDStructTypeDef PIDBar;//摆杆PID
extern PIDStructTypeDef PIDDir;//方向角PID（防止杆旋转太多）

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


