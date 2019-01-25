#ifndef __MATRIX_INVERSE_SOLUTION_H
#define __MATRIX_INVERSE_SOLUTION_H
#include "sys.h"
#include "pwm_init.h"

#define GY_GRAPHICAL_METHOD 0
#define KM_INVERSE 1
#define PARA 1
#define PARA_AND_LINE 1
#define LINEUP 66
#define LINEDOWN 67
#define TURNLEFT 98
#define LINERIGHT 99

//#define EN_GENERATE_STEP 1
//#define INIT_X0 65 //initial position
//#define INIT_Y0 32 //initial position
//#define INIT_Z0 -32 //initial position
#define STEPNUM 100
#define LSTEPNUM1 150//直线功能1的步数 
#define LSTEPNUM2 25//直线功能2的步数
#define LSTEPNUM3 20//直线功能3的步数
#define LSTEPNUM4 30//直线功能4的步数

#define TSTEPNUM1 100//转弯功能1的步数
#define TSTEPNUM2 100//转弯功能2的步数
#define TSTEPNUM3 100//转弯功能3的步数


#define LTOTALSTEPS 940 //一个循环内三个功能的总步数
#define TTOTALSTEPS 4*(LSTEPNUM1+LSTEPNUM2+LSTEPNUM3)
/*****-----直线部分------*****/
#define LCYCLE1_F1SART 0
#define LCYCLE1_F2SART LSTEPNUM1
#define LCYCLE1_F3SART (LCYCLE1_F2SART + LSTEPNUM2)
#define LCYCLE1_F4SART (LCYCLE1_F3SART + LSTEPNUM3)

#define LCYCLE2_F1SART (LCYCLE1_F4SART + LSTEPNUM3)
#define LCYCLE2_F2SART (LCYCLE2_F1SART + LSTEPNUM1)
#define LCYCLE2_F3SART (LCYCLE2_F2SART + LSTEPNUM2)
#define LCYCLE2_F4SART (LCYCLE2_F3SART + LSTEPNUM3)

#define LCYCLE3_F1SART (LCYCLE2_F4SART + LSTEPNUM3)
#define LCYCLE3_F2SART (LCYCLE3_F1SART + LSTEPNUM1)
#define LCYCLE3_F3SART (LCYCLE3_F2SART + LSTEPNUM2)
#define LCYCLE3_F4SART (LCYCLE3_F3SART + LSTEPNUM3)

#define LCYCLE4_F1SART (LCYCLE3_F4SART + LSTEPNUM3)
#define LCYCLE4_F2SART (LCYCLE4_F1SART + LSTEPNUM1)
#define LCYCLE4_F3SART (LCYCLE4_F2SART + LSTEPNUM2)
#define LCYCLE4_F4SART (LCYCLE4_F3SART + LSTEPNUM3)

/***--转弯部分--****/
#define TCYCLE1_F1SART 0
#define TCYCLE1_F2SART TSTEPNUM1
#define TCYCLE1_F3SART (TCYCLE1_F2SART + TSTEPNUM2)

#define TCYCLE2_F1SART (TCYCLE1_F3SART + TSTEPNUM3)
#define TCYCLE2_F2SART (TCYCLE2_F1SART + TSTEPNUM1)
#define TCYCLE2_F3SART (TCYCLE2_F2SART + TSTEPNUM2)

#define TCYCLE3_F1SART (TCYCLE2_F3SART + TSTEPNUM3)
#define TCYCLE3_F2SART (TCYCLE3_F1SART + TSTEPNUM1)
#define TCYCLE3_F3SART (TCYCLE3_F2SART + TSTEPNUM2)

#define TCYCLE4_F1SART (TCYCLE3_F3SART + TSTEPNUM3)
#define TCYCLE4_F2SART (TCYCLE4_F1SART + TSTEPNUM1)
#define TCYCLE4_F3SART (TCYCLE4_F2SART + TSTEPNUM2)

#define CYCLE1st 0
#define CYCLE2nd 1
#define CYCLE3rd 2
#define CYCLE4th 3 //三角步态，四个循环

#define WIDTH 30
#define HIGHTS 35
#define JonintNum 12
#define LEGNUM 4
#define LegRF 0
#define LegLF 1
#define LegRR 2
#define LegLR 3
typedef struct Kinematics
{
//	int32_t LineUptheta1[LEGNUM][LTOTALSTEPS];
//	int32_t LineUptheta2[LEGNUM][LTOTALSTEPS];
//	int32_t LineUptheta3[LEGNUM][LTOTALSTEPS];
//	
//	int32_t LineBacktheta1[LEGNUM][LTOTALSTEPS];
//	int32_t LineBacktheta2[LEGNUM][LTOTALSTEPS];
//	int32_t LineBacktheta3[LEGNUM][LTOTALSTEPS];
//	
//	int32_t TurnLefttheta1[LEGNUM][4*STEPNUM];
//	int32_t TurnLefttheta2[LEGNUM][4*STEPNUM];
//	int32_t TurnLefttheta3[LEGNUM][4*STEPNUM];
//	
//	int32_t TurnRighttheta1[LEGNUM][4*STEPNUM];
//	int32_t TurnRighttheta2[LEGNUM][4*STEPNUM];
//	int32_t TurnRighttheta3[LEGNUM][4*STEPNUM];
	
	float init_x0[LEGNUM];
	float init_y0[LEGNUM];
	float init_z0[LEGNUM];
	float LineUppx[LEGNUM][LTOTALSTEPS];
	float LineUppy[LEGNUM][LTOTALSTEPS];
	float LineUppz[LEGNUM][LTOTALSTEPS];  
	float StartAngle[JonintNum];
	uint32_t StepNum;//数据步距
}KinematicsArm;

void Inverse_Kinematic(int8_t LegNum);
void InitRobotPosion(void);
void Position_Genarate(float Width,float Hights,int8_t CycleNum,int8_t LegNum);
void FullStepCycle(void);
void StartAngleInit(void);
void F1GenerateHalfParabolic(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode);
void F2GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode);
void F3GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode);
void F4GenerateLine(float Width, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode);
void F0GenerateLine(int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode);

#endif