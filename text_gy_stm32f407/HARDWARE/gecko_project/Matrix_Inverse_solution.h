#ifndef __MATRIX_INVERSE_SOLUTION_H
#define __MATRIX_INVERSE_SOLUTION_H
#include "sys.h"
#include "pwm_init.h"

#define GY_GRAPHICAL_METHOD 0
#define KM_INVERSE 1
//#define EN_GENERATE_STEP 1
//#define INIT_X0 65 //initial position
//#define INIT_Y0 32 //initial position
//#define INIT_Z0 -32 //initial position
#define STEPNUM 100//nums of generated steps 
#define CYCLE1st 0
#define CYCLE2nd 1
#define CYCLE3rd 2
#define CYCLE4th 3
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
	int32_t theta1[LEGNUM][4*STEPNUM];
	int32_t theta2[LEGNUM][4*STEPNUM];
	int32_t theta3[LEGNUM][4*STEPNUM];
	float init_x0[LEGNUM];
	float init_y0[LEGNUM];
	float init_z0[LEGNUM];
	float px[LEGNUM][4*STEPNUM];
	float py[LEGNUM][4*STEPNUM];
	float pz[LEGNUM][4*STEPNUM];
}KinematicsArm;

void Inverse_Kinematic(int8_t LegNum);
void InitRobotPosion(void);
void Position_Genarate(float Width,float Hights,int8_t CycleNum,int8_t LegNum);
void FullStepCycle(void);
#endif