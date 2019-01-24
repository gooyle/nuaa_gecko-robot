#include "Matrix_Inverse_solution.h"
#include "math.h"
/*-----------------------------------------------------------------------*
*Input:x0,y0,z0
*output:theta1,theta2,theta3
*------------------------------------------------------------------------*/
KinematicsArm KMGecko;

void Inverse_Kinematic(int8_t LegNum)
{
	int i = 0;
	float a1 = 1.0;//link lengthen
	float a2 = 1.0;//link lengthen
	float a3 = 1.0;//link lengthen
	float a4 = 1.0;//link lengthen
	float a5 = 0.0;//link lengthen
	
	float l1 = 65;//link lengthen
	float l2 = 32;//link lengthen
	float l3 = 32;//link lengthen
	
	float x0 = KMGecko.init_x0[LegNum];//x value of end-effect in basic frame
	float y0 = KMGecko.init_y0[LegNum];//y value of end-effect in basic frame
	float z0 = KMGecko.init_z0[LegNum];//z value of end-effect in basic frame
	
	float theta1 = 0.0;//joint 1 variable
	float theta2 = 0.0;//joint 2 variable
	float theta3 = 0.0;//joint 3 variable
	
	float r =0.0; //Graphical Method variable
	float r1 =0.0;//Graphical Method variable
	float phi1 = 0.0;//Graphical Method variable
	float phi2 = 0.0;//Graphical Method variable
	float phi3 = 0.0;//Graphical Method variable
	float phi4 = 0.0;//Graphical Method variable
	float phi5 = 0.0;//Graphical Method variable
	float phi6 = 0.0;//Graphical Method variable
	/****************--------生成步态------------*********************/
	for(i = 0; i< STEPNUM +1; i++)
	{
		x0 = KMGecko.init_x0[LegNum];
		y0 = KMGecko.py[LegNum][i];
		z0 = KMGecko.pz[LegNum][i];
		/*******Inverse Kinematics*********/
		#if GY_GRAPHICAL_METHOD
		r = sqrt(x0*x0 + (z0 - a1)*(z0 - a1));//eq.1
		phi1 = atan2(z0 - a1, x0);//radians units
		r1 = sqrt(r*r + a2*a2 -2*r*a2*cos(phi1));
		phi3 = atan2(z0-a1,x0-a1);
		phi2 = acos((r1*r1 +a3*a3-a4*a4)/(2*r1*a3));
		theta2 = phi3 - phi2;
		phi4 = acos((a4*a4 + a3*a3 - r1*r1)/(2*a4*a3));
		theta3 = 3.1415 - phi4;
		phi5 = atan2(a5,a2+a3*cos(theta2)+a4*cos(theta3));
		phi6 = atan2(y0,x0);
		theta1 = phi6 - phi5;
		#endif
		#if KM_INVERSE
		theta1 = asin((l1*l1 + l2*l2 + l3*l3 - x0*x0 -y0*y0 - z0*z0)/(2*l1*l2));
		theta2 = asin((x0*x0 + y0*y0 + z0*z0 +l1*l1 -l2*l2 - l3*l3)/(2*l1*sqrt(x0*x0 + y0*y0 +z0*z0 - l3*l3))) - atan2(sqrt(x0*x0 + z0*z0 -l3*l3),y0);
		theta3 = asin((-l3)/(sqrt(x0*x0 + z0*z0))) - atan2(z0,x0);
		#endif
	/**-----------角度转换---------**/
		KMGecko.theta1[LegNum][i] = (int)((theta1/3.1415)*180);
		KMGecko.theta2[LegNum][i] = (int)((theta2/3.1415)*180);
		KMGecko.theta3[LegNum][i] = (int)((theta3/3.1415)*180);
	}
}

/********************************************************************************************************
*date: 2018-10-10
*function: Adjust Robot to initial posion and set px[],py[],pz[] as zero
*author: NUAA google
********************************************************************************************************/ 
void InitRobotPosion(void)
{
	int i =0;
	for(i = 0; i<LEGNUM;i++)
	KMGecko.init_x0[i] = 65;
	for(i = 0; i<LEGNUM;i++)
	KMGecko.init_y0[i] = 32;
	for(i = 0; i<LEGNUM;i++)
	KMGecko.init_z0[i] = -32;
	
	Angle(KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(KMGecko.StartAngle[LF_J1],LF_J1);
	Angle(KMGecko.StartAngle[LF_J2],LF_J2);
	
	Angle(KMGecko.StartAngle[RF_J1],RF_J1);
	Angle(KMGecko.StartAngle[RF_J2],RF_J2);		
	Angle(KMGecko.StartAngle[RF_J3],RF_J3);
	
	Angle(KMGecko.StartAngle[RR_J1],RR_J1);
	Angle(KMGecko.StartAngle[RR_J2],RR_J2);
	Angle(KMGecko.StartAngle[RR_J3],RR_J3);

	Angle(KMGecko.StartAngle[LR_J1],LR_J1);
	Angle(KMGecko.StartAngle[LR_J2],LR_J2);
	Angle(KMGecko.StartAngle[LR_J3],LR_J3);
}

/********************************************************************************************************
*date：2018-10-10
*function:generate angle 
*input:Width ,Hights,LegNum
*output:single 
*author：NUAA google
********************************************************************************************************/
//void Position_Genarate(float Width,float Hights,int8_t CycleNum,int8_t LegNum)
//{
//	float i = 0;
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//			for(j = 0;j<STEPNUM;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				KMGecko.py[LegNum][j] = i;
//				
//				#if PARA
//				KMGecko.pz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(STEPNUM1-1));
//				#endif
//				#if PARA_AND_LINE
//				if(Width > 0)
//					if((KMGecko.py[LegNum][j] <= (int)(Width/2)))
//					{
//						KMGecko.pz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//						KMGecko.py[LegNum][j] = i + KMGecko.init_y0[LegNum];
//						i = i + ((Width*1.0)/(STEPNUM1-1))/2;
//					}
//					if(KMGecko.py[LegNum][j] > (int)(Width/2))
//					{
//						
//					}
//				#endif
//			}	
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//			for(j = 0;j<STEPNUM;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				KMGecko.py[LegNum][j+STEPNUM] = i;
//				KMGecko.pz[LegNum][j+STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+STEPNUM] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(STEPNUM-1));
//			}	
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//			for(j = 0;j<STEPNUM;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				KMGecko.py[LegNum][j+2*STEPNUM] = i;
//				KMGecko.pz[LegNum][j+2*STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+2*STEPNUM] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(STEPNUM-1));
//			}	
//	}
//		if (CycleNum == CYCLE4th)
//	{
//				for(j = 0;j<STEPNUM;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				KMGecko.py[LegNum][j+3*STEPNUM] = i;
//				KMGecko.pz[LegNum][j+3*STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+3*STEPNUM] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(STEPNUM-1));
//			}	
//		}
//	}

/********************************************************************************************************
*date：2018-10-10
*function:complate full Step Cycle
*author：NUAA google
********************************************************************************************************/
void FullStepCycle(void)
{
	/**----------RF setted as swing leg----------**/
	F0GenerateLine(CYCLE1st,LegRF, LCYCLE1_F4SART,LINEUP);
	F1GenerateHalfParabolic(WIDTH, HIGHTS, CYCLE1st,LegRF, LSTEPNUM1,LINEUP);
	F2GenerateHalfTriangle(WIDTH/4, HIGHTS,CYCLE1st,LegRF, LSTEPNUM2,LINEUP);
	F3GenerateHalfTriangle(WIDTH/4, HIGHTS,CYCLE1st,LegRF, LSTEPNUM3,LINEUP);
	F4GenerateLine(WIDTH/4,CYCLE1st,LegRF, LSTEPNUM4,LINEUP);
	
	
	Position_Genarate(WIDTH,HIGHTS,CYCLE1st,LegRF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegLF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegLR);
	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegRR);
	
	/**--------refresh the frame in StepCycle1-----**/
	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][STEPNUM - 1];
	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][STEPNUM - 1];
	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][STEPNUM - 1];
	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][STEPNUM - 1];	
	
		/**----------LF setted as support leg----------**/
	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegRF);
	Position_Genarate(WIDTH,HIGHTS,CYCLE2nd,LegLF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegLR);
	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegRR);
	
	/**--------refresh the frame in StepCycle2-----**/
	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][2*STEPNUM - 1];
	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][2*STEPNUM - 1];
	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][2*STEPNUM - 1];
	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][2*STEPNUM - 1];	
	
			/**----------RR setted as support leg----------**/
	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegRF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegLF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegLR);
	Position_Genarate(WIDTH,HIGHTS,CYCLE3rd,LegRR);
	
	/**--------refresh the frame in StepCycle3-----**/
	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][3*STEPNUM - 1];
	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][3*STEPNUM - 1];
	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][3*STEPNUM - 1];
	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][3*STEPNUM - 1];	
	
			/**----------LR setted as support leg----------**/
	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegRF);
	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegLF);
	Position_Genarate(WIDTH,HIGHTS,CYCLE4th,LegLR);
	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegRR);
	
	/**--------refresh the frame in StepCycle1-----**/
	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][4*STEPNUM - 1];
	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][4*STEPNUM - 1];
	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][4*STEPNUM - 1];
	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][4*STEPNUM - 1];	

}
/**初始位置矫零*/
void StartAngleInit(void)
{
	KMGecko.StartAngle[RF_J1] = 0;
	KMGecko.StartAngle[LF_J1] = 0;
	KMGecko.StartAngle[RR_J1] = 0;
	KMGecko.StartAngle[LR_J1] = 0;
	KMGecko.StartAngle[RF_J2] = 0;
	KMGecko.StartAngle[LF_J2] = 0;
	KMGecko.StartAngle[RR_J2] = 0;
	KMGecko.StartAngle[LR_J2] = 0;
	KMGecko.StartAngle[RF_J3] = 0;
	KMGecko.StartAngle[LF_J3] = 0;
	KMGecko.StartAngle[RR_J3] = 0;
	KMGecko.StartAngle[LR_J3] = 0;
}

/********************************************************************************************************
*date：2019-1-23
*function:generate angle parabolic
*input:Width ,Hights,LegNum,DataNumber：STEPNUM1,STEPNUM2,STEPNUM3
*output:角度 
*author：NUAA google
*explain:设计此部分为第一部分数据，因此，存储在第一部分
********************************************************************************************************/
void F1GenerateHalfParabolic(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
{
	
	float i = 0;
	int j = 0;
	if (CycleNum == CYCLE1st)
	{
			if(RunMode == LINEUP)
			{
				for(j = 0;j<DataNumber;j++)
				{
					/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
					//KMGecko.LineUppy[LegNum][j] = i;
					KMGecko.LineUppz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
					KMGecko.LineUppy[LegNum][j] = i + KMGecko.init_y0[LegNum];
					i = i + ((Width*1.0)/(DataNumber-1))/2;//半个抛物线，除以2
				}	
			}
			
	}
		if (CycleNum == CYCLE2nd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+CYCLE2_F1SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE2_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE2_F1SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1))/2;
			}
		}			
	}
		if (CycleNum == CYCLE3rd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+CYCLE3_F1SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE3_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE3_F1SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1))/2;
			}
		}
	}
		if (CycleNum == CYCLE4th)
	{
		if(RunMode == LINEUP)
		{
				for(j = 0;j<DataNumber;j++)
			{
				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
				KMGecko.py[LegNum][j+LCYCLE4_F1SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE4_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE4_F1SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1))/2;
			}
		}
		}
}

/********************************************************************************************************
*date：2019-1-23
*function:generate angle triangle
*input:Width ,Hights,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
*output:角度 
*author：NUAA google
*explain：等腰三角型，顶角朝向右.此部分为单端直线。第二个功能函数，DataNumber所定义的数据个数
********************************************************************************************************/
void F2GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
{
	float i = 0;
	int j = 0;
	if (CycleNum == CYCLE1st)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE1_F2SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE1_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE1_F2SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//
			}
		}
			
	}
		if (CycleNum == CYCLE2nd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE2_F2SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE2_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE2_F2SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}	
		}
	}
		if (CycleNum == CYCLE3rd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE3_F2SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE3_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE3_F2SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}
		}			
	}
		if (CycleNum == CYCLE4th)
	{
		if(RunMode == LINEUP)
		{
				for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE4_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}
		}
		}
}
/********************************************************************************************************
*date：2019-1-23
*function:generate angle triangle
*input:Width ,Hights,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
*output:角度 
*author：NUAA google
*explain：等腰三角型，顶角朝向右.此部分为单端直线。第三个功能函数，DataNumber所定义的数据个数
********************************************************************************************************/
void F3GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
{
	float i = 0;
	int j = 0;
	if (CycleNum == CYCLE1st)
	{
		if(RunMode == LINEUP)
		{			
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE1_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//
			}
		}
			
	}
		if (CycleNum == CYCLE2nd)
	{
			if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE2_F3SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE2_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE2_F3SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}
		}			
	}
		if (CycleNum == CYCLE3rd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE3_F3SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE3_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE3_F3SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}
		}			
	}
		if (CycleNum == CYCLE4th)
	{
				if(RunMode == LINEUP)
		{
				for(j = 0;j<DataNumber;j++)
			{
				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE4_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE4_F3SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));
			}	
		}
		}
}

/********************************************************************************************************
*date：2019-1-23
*function:generate angle triangle
*input:Width,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
*output:角度 
*author：NUAA google
*explain：直线返回函数，第三个功能函数，DataNumber所定义的数据个数
********************************************************************************************************/
void F4GenerateLine(float Width, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode)
{
	float i = 0;
	int j = 0;
	if (CycleNum == CYCLE1st)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy面上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE1_F4SART] = KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE1_F4SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
			}
		}			
	}
		if (CycleNum == CYCLE2nd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE2_F4SART] = (KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
			}
		}			
	}
		if (CycleNum == CYCLE3rd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE3_F4SART] = KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
			}
		}			
	}
		if (CycleNum == CYCLE4th)
	{
		if(RunMode == LINEUP)
		{
				for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i;
				KMGecko.pz[LegNum][j+LCYCLE4_F4SART] = KMGecko.init_z0[LegNum];
				KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i + KMGecko.init_y0[LegNum];
				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
			}	
		}
		}
}

/********************************************************************************************************
*date：2019-1-23
*function:GENERATE INITPosition
*input:LegNum,CycleNum,LegNum,DataNumber:STEPNUM1+STEPNUM2+STEPNUM3
*output:角度 
*author：NUAA google
*explain：直线返回函数，第三个功能函数，DataNumber所定义的数据个数
********************************************************************************************************/
void F0GenerateLine(int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode)
{
	int j = 0;
	if (CycleNum == CYCLE1st)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**---------------**/
				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
				if (LegNum == 0)
				{
				KMGecko.pz[1][j] = KMGecko.init_z0[1];
				KMGecko.py[1][j] = KMGecko.init_y0[1];
				KMGecko.pz[2][j] = KMGecko.init_z0[2];
				KMGecko.py[2][j] = KMGecko.init_y0[2];
				KMGecko.pz[3][j] = KMGecko.init_z0[3];
				KMGecko.py[3][j] = KMGecko.init_y0[3];
				}
				if (LegNum == 1)
				{
				KMGecko.pz[0][j] = KMGecko.init_z0[0];
				KMGecko.py[0][j] = KMGecko.init_y0[0];
				KMGecko.pz[2][j] = KMGecko.init_z0[2];
				KMGecko.py[2][j] = KMGecko.init_y0[2];
				KMGecko.pz[3][j] = KMGecko.init_z0[3];
				KMGecko.py[3][j] = KMGecko.init_y0[3];
				}
				if (LegNum == 2)
				{
				KMGecko.pz[1][j] = KMGecko.init_z0[1];
				KMGecko.py[1][j] = KMGecko.init_y0[1];
				KMGecko.pz[0][j] = KMGecko.init_z0[0];
				KMGecko.py[0][j] = KMGecko.init_y0[0];
				KMGecko.pz[3][j] = KMGecko.init_z0[3];
				KMGecko.py[3][j] = KMGecko.init_y0[3];
				}
				if (LegNum == 3)
				{
				KMGecko.pz[1][j] = KMGecko.init_z0[1];
				KMGecko.py[1][j] = KMGecko.init_y0[1];
				KMGecko.pz[2][j] = KMGecko.init_z0[2];
				KMGecko.py[2][j] = KMGecko.init_y0[2];
				KMGecko.pz[0][j] = KMGecko.init_z0[0];
				KMGecko.py[0][j] = KMGecko.init_y0[0];
				}
			}
		}			
	}
		if (CycleNum == CYCLE2nd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i;
				if (LegNum == 0)
				{
				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 1)
				{
				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 2)
				{
				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 3)
				{
				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
				}
			}
		}			
	}
		if (CycleNum == CYCLE3rd)
	{
		if(RunMode == LINEUP)
		{
			for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i;
				if (LegNum == 0)
				{
				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 1)
				{
				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 2)
				{
				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 3)
				{
				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
				}
			}
		}			
	}
		if (CycleNum == CYCLE4th)
	{
		if(RunMode == LINEUP)
		{
				for(j = 0;j<DataNumber;j++)
			{
				/**-------直线在zy轴上的函数表达式并离散化--------**/
				//KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i;
				if (LegNum == 0)
				{
				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 1)
				{
				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 2)
				{
				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
				}
				if (LegNum == 3)
				{
				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
				}
			}	
		}
		}
}



