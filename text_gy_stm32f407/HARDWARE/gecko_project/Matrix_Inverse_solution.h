#ifndef __MATRIX_INVERSE_SOLUTION_H
#define __MATRIX_INVERSE_SOLUTION_H
#include "sys.h"
#define GY_GRAPHICAL_METHOD 0
#define KM_INVERSE 1

typedef struct Kinematics
{
	int32_t theta1[100];
	int32_t theta2[100];
	int32_t theta3[100];
	
}KinematicsArm;
void Inverse_Kinematic(void);
#endif