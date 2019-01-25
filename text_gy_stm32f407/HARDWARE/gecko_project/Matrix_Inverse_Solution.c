#include "Matrix_Inverse_solution.h"
#include "math.h"
/*-----------------------------------------------------------------------*
*Input:x0,y0,z0
*output:theta1,theta2,theta3
*------------------------------------------------------------------------*/
KinematicsArm KMGecko;

//void Inverse_Kinematic(int8_t LegNum)
//{
//	int i = 0;
//	float a1 = 1.0;//link lengthen
//	float a2 = 1.0;//link lengthen
//	float a3 = 1.0;//link lengthen
//	float a4 = 1.0;//link lengthen
//	float a5 = 0.0;//link lengthen
//	
//	float l1 = 65;//link lengthen
//	float l2 = 32;//link lengthen
//	float l3 = 32;//link lengthen
//	
//	float x0 = KMGecko.init_x0[LegNum];//x value of end-effect in basic frame
//	float y0 = KMGecko.init_y0[LegNum];//y value of end-effect in basic frame
//	float z0 = KMGecko.init_z0[LegNum];//z value of end-effect in basic frame
//	
//	float theta1 = 0.0;//joint 1 variable
//	float theta2 = 0.0;//joint 2 variable
//	float theta3 = 0.0;//joint 3 variable
//	
//	float r =0.0; //Graphical Method variable
//	float r1 =0.0;//Graphical Method variable
//	float phi1 = 0.0;//Graphical Method variable
//	float phi2 = 0.0;//Graphical Method variable
//	float phi3 = 0.0;//Graphical Method variable
//	float phi4 = 0.0;//Graphical Method variable
//	float phi5 = 0.0;//Graphical Method variable
//	float phi6 = 0.0;//Graphical Method variable
//	/****************--------生成步态------------*********************/
//	for(i = 0; i< STEPNUM +1; i++)
//	{
//		x0 = KMGecko.init_x0[LegNum];
//		y0 = KMGecko.LineUppy[LegNum][i];
//		z0 = KMGecko.LineUppz[LegNum][i];
//		/*******Inverse Kinematics*********/
//		#if GY_GRAPHICAL_METHOD
//		r = sqrt(x0*x0 + (z0 - a1)*(z0 - a1));//eq.1
//		phi1 = atan2(z0 - a1, x0);//radians units
//		r1 = sqrt(r*r + a2*a2 -2*r*a2*cos(phi1));
//		phi3 = atan2(z0-a1,x0-a1);
//		phi2 = acos((r1*r1 +a3*a3-a4*a4)/(2*r1*a3));
//		theta2 = phi3 - phi2;
//		phi4 = acos((a4*a4 + a3*a3 - r1*r1)/(2*a4*a3));
//		theta3 = 3.1415 - phi4;
//		phi5 = atan2(a5,a2+a3*cos(theta2)+a4*cos(theta3));
//		phi6 = atan2(y0,x0);
//		theta1 = phi6 - phi5;
//		#endif
//		#if KM_INVERSE
//		theta1 = asin((l1*l1 + l2*l2 + l3*l3 - x0*x0 -y0*y0 - z0*z0)/(2*l1*l2));
//		theta2 = asin((x0*x0 + y0*y0 + z0*z0 +l1*l1 -l2*l2 - l3*l3)/(2*l1*sqrt(x0*x0 + y0*y0 +z0*z0 - l3*l3))) - atan2(sqrt(x0*x0 + z0*z0 -l3*l3),y0);
//		theta3 = asin((-l3)/(sqrt(x0*x0 + z0*z0))) - atan2(z0,x0);
//		#endif
//	/**-----------角度转换---------**/
//		KMGecko.LineUptheta1[LegNum][i] = (int)((theta1/3.1415)*180);
//		KMGecko.LineUptheta2[LegNum][i] = (int)((theta2/3.1415)*180);
//		KMGecko.LineUptheta2[LegNum][i] = (int)((theta3/3.1415)*180);
//	}
//}

/********************************************************************************************************
*date: 2018-10-10
*function: Adjust Robot to initial posion and set px[],py[],pz[] as zero
*author: NUAA google
********************************************************************************************************/ 
void InitRobotPosion(void)
{
//	int i =0;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_x0[i] = 65;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_y0[i] = 32;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_z0[i] = -32;
	
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

///********************************************************************************************************
//*date：2018-10-10
//*function:generate angle 
//*input:Width ,Hights,LegNum
//*output:single 
//*author：NUAA google
//********************************************************************************************************/
////void Position_Genarate(float Width,float Hights,int8_t CycleNum,int8_t LegNum)
////{
////	float i = 0;
////	int j = 0;
////	if (CycleNum == CYCLE1st)
////	{
////			for(j = 0;j<STEPNUM;j++)
////			{
////				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
////				KMGecko.py[LegNum][j] = i;
////				
////				#if PARA
////				KMGecko.pz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
////				KMGecko.py[LegNum][j] = i + KMGecko.init_y0[LegNum];
////				i = i + ((Width*1.0)/(STEPNUM1-1));
////				#endif
////				#if PARA_AND_LINE
////				if(Width > 0)
////					if((KMGecko.py[LegNum][j] <= (int)(Width/2)))
////					{
////						KMGecko.pz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
////						KMGecko.py[LegNum][j] = i + KMGecko.init_y0[LegNum];
////						i = i + ((Width*1.0)/(STEPNUM1-1))/2;
////					}
////					if(KMGecko.py[LegNum][j] > (int)(Width/2))
////					{
////						
////					}
////				#endif
////			}	
////	}
////		if (CycleNum == CYCLE2nd)
////	{
////			for(j = 0;j<STEPNUM;j++)
////			{
////				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
////				KMGecko.py[LegNum][j+STEPNUM] = i;
////				KMGecko.pz[LegNum][j+STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
////				KMGecko.py[LegNum][j+STEPNUM] = i + KMGecko.init_y0[LegNum];
////				i = i + ((Width*1.0)/(STEPNUM-1));
////			}	
////	}
////		if (CycleNum == CYCLE3rd)
////	{
////			for(j = 0;j<STEPNUM;j++)
////			{
////				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
////				KMGecko.py[LegNum][j+2*STEPNUM] = i;
////				KMGecko.pz[LegNum][j+2*STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
////				KMGecko.py[LegNum][j+2*STEPNUM] = i + KMGecko.init_y0[LegNum];
////				i = i + ((Width*1.0)/(STEPNUM-1));
////			}	
////	}
////		if (CycleNum == CYCLE4th)
////	{
////				for(j = 0;j<STEPNUM;j++)
////			{
////				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
////				KMGecko.py[LegNum][j+3*STEPNUM] = i;
////				KMGecko.pz[LegNum][j+3*STEPNUM] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
////				KMGecko.py[LegNum][j+3*STEPNUM] = i + KMGecko.init_y0[LegNum];
////				i = i + ((Width*1.0)/(STEPNUM-1));
////			}	
////		}
////	}

///********************************************************************************************************
//*date：2018-10-10
//*function:complate full Step Cycle
//*author：NUAA google
//********************************************************************************************************/
////void FullStepCycle(void)
////{
////	/**----------RF setted as swing leg----------**/
////	F0GenerateLine(CYCLE1st,LegRF, LCYCLE1_F4SART,LINEUP);
////	F1GenerateHalfParabolic(WIDTH, HIGHTS, CYCLE1st,LegRF, LSTEPNUM1,LINEUP);
////	F2GenerateHalfTriangle(WIDTH/4, HIGHTS,CYCLE1st,LegRF, LSTEPNUM2,LINEUP);
////	F3GenerateHalfTriangle(WIDTH/4, HIGHTS,CYCLE1st,LegRF, LSTEPNUM3,LINEUP);
////	F4GenerateLine(WIDTH/4,CYCLE1st,LegRF, LSTEPNUM4,LINEUP);
////	
////	
////	Position_Genarate(WIDTH,HIGHTS,CYCLE1st,LegRF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegLF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegLR);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE1st,LegRR);
////	
////	/**--------refresh the frame in StepCycle1-----**/
////	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][STEPNUM - 1];
////	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][STEPNUM - 1];
////	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][STEPNUM - 1];
////	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][STEPNUM - 1];	
////	
////		/**----------LF setted as support leg----------**/
////	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegRF);
////	Position_Genarate(WIDTH,HIGHTS,CYCLE2nd,LegLF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegLR);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE2nd,LegRR);
////	
////	/**--------refresh the frame in StepCycle2-----**/
////	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][2*STEPNUM - 1];
////	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][2*STEPNUM - 1];
////	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][2*STEPNUM - 1];
////	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][2*STEPNUM - 1];	
////	
////			/**----------RR setted as support leg----------**/
////	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegRF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegLF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE3rd,LegLR);
////	Position_Genarate(WIDTH,HIGHTS,CYCLE3rd,LegRR);
////	
////	/**--------refresh the frame in StepCycle3-----**/
////	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][3*STEPNUM - 1];
////	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][3*STEPNUM - 1];
////	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][3*STEPNUM - 1];
////	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][3*STEPNUM - 1];	
////	
////			/**----------LR setted as support leg----------**/
////	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegRF);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegLF);
////	Position_Genarate(WIDTH,HIGHTS,CYCLE4th,LegLR);
////	Position_Genarate(-WIDTH/3.0,0,CYCLE4th,LegRR);
////	
////	/**--------refresh the frame in StepCycle1-----**/
////	KMGecko.init_y0[LegRF] = KMGecko.py[LegRF][4*STEPNUM - 1];
////	KMGecko.init_y0[LegLF] = KMGecko.py[LegLF][4*STEPNUM - 1];
////	KMGecko.init_y0[LegLR] = KMGecko.py[LegLR][4*STEPNUM - 1];
////	KMGecko.init_y0[LegRR] = KMGecko.py[LegRR][4*STEPNUM - 1];	

////}
///**初始位置矫零*/
//void StartAngleInit(void)
//{
//	KMGecko.StartAngle[RF_J1] = 0;
//	KMGecko.StartAngle[LF_J1] = 0;
//	KMGecko.StartAngle[RR_J1] = 0;
//	KMGecko.StartAngle[LR_J1] = 0;
//	KMGecko.StartAngle[RF_J2] = 0;
//	KMGecko.StartAngle[LF_J2] = 0;
//	KMGecko.StartAngle[RR_J2] = 0;
//	KMGecko.StartAngle[LR_J2] = 0;
//	KMGecko.StartAngle[RF_J3] = 0;
//	KMGecko.StartAngle[LF_J3] = 0;
//	KMGecko.StartAngle[RR_J3] = 0;
//	KMGecko.StartAngle[LR_J3] = 0;
//}

///********************************************************************************************************
//*date：2019-1-23
//*function:generate angle parabolic
//*input:Width ,Hights,LegNum,DataNumber：STEPNUM1,STEPNUM2,STEPNUM3
//*output:角度 
//*author：NUAA google
//*explain:设计此部分为第一部分数据，因此，存储在第一部分
//********************************************************************************************************/
//void F1GenerateHalfParabolic(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
//{
//	
//	float i = 0;
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//			if(RunMode == LINEUP)
//			{
//				for(j = 0;j<DataNumber;j++)
//				{
//					/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//					//KMGecko.LineUppy[LegNum][j] = i;
//					KMGecko.LineUppz[LegNum][j] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//					KMGecko.LineUppy[LegNum][j] = i + KMGecko.init_y0[LegNum];
//					i = i + ((Width*1.0)/(DataNumber-1))/2;//半个抛物线，除以2
//				}	
//			}
//			
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+CYCLE2_F1SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE2_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE2_F1SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1))/2;
//			}
//		}			
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+CYCLE3_F1SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE3_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE3_F1SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1))/2;
//			}
//		}
//	}
//		if (CycleNum == CYCLE4th)
//	{
//		if(RunMode == LINEUP)
//		{
//				for(j = 0;j<DataNumber;j++)
//			{
//				/**-------抛物线曲线在zy轴上的函数表达式并离散化--------**/
//				KMGecko.py[LegNum][j+LCYCLE4_F1SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE4_F1SART] = (-4.0*Hights*i*i)/(Width*Width) + (4.0*Hights*i/Width) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE4_F1SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1))/2;
//			}
//		}
//		}
//}

///********************************************************************************************************
//*date：2019-1-23
//*function:generate angle triangle
//*input:Width ,Hights,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
//*output:角度 
//*author：NUAA google
//*explain：等腰三角型，顶角朝向右.此部分为单端直线。第二个功能函数，DataNumber所定义的数据个数
//********************************************************************************************************/
//void F2GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
//{
//	float i = 0;
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE1_F2SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE1_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE1_F2SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//
//			}
//		}
//			
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE2_F2SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE2_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE2_F2SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}	
//		}
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE3_F2SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE3_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE3_F2SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}
//		}			
//	}
//		if (CycleNum == CYCLE4th)
//	{
//		if(RunMode == LINEUP)
//		{
//				for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE4_F2SART] = (-1.0*Hights*i)/Width + (1.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}
//		}
//		}
//}
///********************************************************************************************************
//*date：2019-1-23
//*function:generate angle triangle
//*input:Width ,Hights,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
//*output:角度 
//*author：NUAA google
//*explain：等腰三角型，顶角朝向右.此部分为单端直线。第三个功能函数，DataNumber所定义的数据个数
//********************************************************************************************************/
//void F3GenerateHalfTriangle(float Width, float Hights, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber,uint8_t RunMode)
//{
//	float i = 0;
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//		if(RunMode == LINEUP)
//		{			
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE1_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//
//			}
//		}
//			
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//			if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE2_F3SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE2_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE2_F3SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}
//		}			
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE3_F3SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE3_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE3_F3SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}
//		}			
//	}
//		if (CycleNum == CYCLE4th)
//	{
//				if(RunMode == LINEUP)
//		{
//				for(j = 0;j<DataNumber;j++)
//			{
//				/**-------两段直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE4_F2SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE4_F3SART] = (1.0*Hights*i)/Width - (0.5*Hights) + KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE4_F3SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));
//			}	
//		}
//		}
//}

///********************************************************************************************************
//*date：2019-1-23
//*function:generate angle triangle
//*input:Width,LegNum,CycleNum,LegNum,DataNumber:STEPNUM1,STEPNUM2,STEPNUM3
//*output:角度 
//*author：NUAA google
//*explain：直线返回函数，第三个功能函数，DataNumber所定义的数据个数
//********************************************************************************************************/
//void F4GenerateLine(float Width, int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode)
//{
//	float i = 0;
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy面上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE1_F4SART] = KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE1_F4SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
//			}
//		}			
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE2_F4SART] = (KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
//			}
//		}			
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE3_F4SART] = KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
//			}
//		}			
//	}
//		if (CycleNum == CYCLE4th)
//	{
//		if(RunMode == LINEUP)
//		{
//				for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i;
//				KMGecko.pz[LegNum][j+LCYCLE4_F4SART] = KMGecko.init_z0[LegNum];
//				KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i + KMGecko.init_y0[LegNum];
//				i = i + ((Width*1.0)/(DataNumber-1));//单条直线数据
//			}	
//		}
//		}
//}

///********************************************************************************************************
//*date：2019-1-23
//*function:GENERATE INITPosition
//*input:LegNum,CycleNum,LegNum,DataNumber:STEPNUM1+STEPNUM2+STEPNUM3
//*output:角度 
//*author：NUAA google
//*explain：直线返回函数，第三个功能函数，DataNumber所定义的数据个数
//********************************************************************************************************/
//void F0GenerateLine(int8_t CycleNum, int8_t LegNum, uint32_t DataNumber, uint8_t RunMode)
//{
//	int j = 0;
//	if (CycleNum == CYCLE1st)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**---------------**/
//				//KMGecko.py[LegNum][j+LCYCLE1_F3SART] = i;
//				if (LegNum == 0)
//				{
//				KMGecko.pz[1][j] = KMGecko.init_z0[1];
//				KMGecko.py[1][j] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j] = KMGecko.init_z0[2];
//				KMGecko.py[2][j] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j] = KMGecko.init_z0[3];
//				KMGecko.py[3][j] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 1)
//				{
//				KMGecko.pz[0][j] = KMGecko.init_z0[0];
//				KMGecko.py[0][j] = KMGecko.init_y0[0];
//				KMGecko.pz[2][j] = KMGecko.init_z0[2];
//				KMGecko.py[2][j] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j] = KMGecko.init_z0[3];
//				KMGecko.py[3][j] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 2)
//				{
//				KMGecko.pz[1][j] = KMGecko.init_z0[1];
//				KMGecko.py[1][j] = KMGecko.init_y0[1];
//				KMGecko.pz[0][j] = KMGecko.init_z0[0];
//				KMGecko.py[0][j] = KMGecko.init_y0[0];
//				KMGecko.pz[3][j] = KMGecko.init_z0[3];
//				KMGecko.py[3][j] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 3)
//				{
//				KMGecko.pz[1][j] = KMGecko.init_z0[1];
//				KMGecko.py[1][j] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j] = KMGecko.init_z0[2];
//				KMGecko.py[2][j] = KMGecko.init_y0[2];
//				KMGecko.pz[0][j] = KMGecko.init_z0[0];
//				KMGecko.py[0][j] = KMGecko.init_y0[0];
//				}
//			}
//		}			
//	}
//		if (CycleNum == CYCLE2nd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE2_F4SART] = i;
//				if (LegNum == 0)
//				{
//				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 1)
//				{
//				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 2)
//				{
//				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[3][j+LCYCLE2_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE2_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 3)
//				{
//				KMGecko.pz[1][j+LCYCLE2_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE2_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE2_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE2_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[0][j+LCYCLE2_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE2_F1SART] = KMGecko.init_y0[0];
//				}
//			}
//		}			
//	}
//		if (CycleNum == CYCLE3rd)
//	{
//		if(RunMode == LINEUP)
//		{
//			for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE3_F4SART] = i;
//				if (LegNum == 0)
//				{
//				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 1)
//				{
//				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 2)
//				{
//				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[3][j+LCYCLE3_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE3_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 3)
//				{
//				KMGecko.pz[1][j+LCYCLE3_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE3_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE3_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE3_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[0][j+LCYCLE3_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE3_F1SART] = KMGecko.init_y0[0];
//				}
//			}
//		}			
//	}
//		if (CycleNum == CYCLE4th)
//	{
//		if(RunMode == LINEUP)
//		{
//				for(j = 0;j<DataNumber;j++)
//			{
//				/**-------直线在zy轴上的函数表达式并离散化--------**/
//				//KMGecko.py[LegNum][j+LCYCLE4_F4SART] = i;
//				if (LegNum == 0)
//				{
//				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 1)
//				{
//				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 2)
//				{
//				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
//				KMGecko.pz[3][j+LCYCLE4_F1SART] = KMGecko.init_z0[3];
//				KMGecko.py[3][j+LCYCLE4_F1SART] = KMGecko.init_y0[3];
//				}
//				if (LegNum == 3)
//				{
//				KMGecko.pz[1][j+LCYCLE4_F1SART] = KMGecko.init_z0[1];
//				KMGecko.py[1][j+LCYCLE4_F1SART] = KMGecko.init_y0[1];
//				KMGecko.pz[2][j+LCYCLE4_F1SART] = KMGecko.init_z0[2];
//				KMGecko.py[2][j+LCYCLE4_F1SART] = KMGecko.init_y0[2];
//				KMGecko.pz[0][j+LCYCLE4_F1SART] = KMGecko.init_z0[0];
//				KMGecko.py[0][j+LCYCLE4_F1SART] = KMGecko.init_y0[0];
//				}
//			}	
//		}
//		}
//}

/**初始位置矫零*/
void StartAngleInit(void)
{
	KMGecko.StartAngle[RF_J1] = 45.0;
	KMGecko.StartAngle[LF_J1] = -55.0;
	KMGecko.StartAngle[RR_J1] = 35.0;
	KMGecko.StartAngle[LR_J1] = -32.0;
	KMGecko.StartAngle[RF_J2] = 0;
	KMGecko.StartAngle[LF_J2] = -15.0;
	KMGecko.StartAngle[RR_J2] = 0;
	KMGecko.StartAngle[LR_J2] = 0;
	KMGecko.StartAngle[RF_J3] = 0;
	KMGecko.StartAngle[LF_J3] = -10.0;
	KMGecko.StartAngle[RR_J3] = 0;
	KMGecko.StartAngle[LR_J3] = 0;
}


///********************************************************************************************************
//*date：2018-10-10
//*function：数据赋值
//*author：NUAA google
//********************************************************************************************************/
//	float LineUptheta1_RF[LTOTALSTEPS] = {0.00,	0.31,	0.61,	0.90,	1.18,	1.44,	1.69,	1.93,	2.16,	2.37,	2.57,	2.76,	2.94,	3.11,	3.27,	3.41,	3.55,	3.67,	3.79,	3.89,	3.99,	4.07,	4.14,	4.21,	4.26,	4.31,	4.34,	4.37,	4.39,	4.40,	4.40,	4.39,	4.37,	4.34,	4.31,	4.27,	4.22,	4.16,	4.10,	4.02,	3.94,	3.86,	3.76,	3.66,	3.55,	3.44,	3.32,	3.19,	3.05,	2.91,	2.76,	2.61,	2.45,	2.29,	2.12,	1.94,	1.76,	1.57,	1.38,	1.18,	0.98,	0.77,	0.56,	0.34,	0.12,	-0.11,	-0.34,	-0.57,	-0.81,	-1.05,	-1.30,	-1.55,	-1.81,	-2.07,	-2.33,	-2.60,	-2.87,	-3.14,	-3.42,	-3.70,	-3.98,	-4.27,	-4.56,	-4.85,	-5.15,	-5.45,	-5.75,	-6.06,	-6.37,	-6.68,	-6.99,	-7.31,	-7.62,	-7.95,	-8.27,	-8.59,	-8.92,	-9.25,	-9.59,	-9.92,	-10.26,	-10.60,	-10.94,	-11.28,	-11.63,	-11.97,	-12.32,	-12.68,	-13.03,	-13.38,	-13.74,	-14.10,	-14.46,	-14.82,	-15.19,	-15.55,	-15.92,	-16.29,	-16.66,	-17.03,	-17.41,	-17.78,	-18.16,	-18.54,	-18.92,	-19.30,	-19.69,	-20.07,	-20.46,	-20.85,	-21.24,	-21.63,	-22.02,	-22.42,	-22.82,	-23.21,	-23.61,	-24.02,	-24.42,	-24.83,	-25.23,	-25.64,	-26.05,	-26.47,	-26.88,	-27.30,	-27.72,	-28.14,	-28.56,	-28.99,	-28.99,	-28.11,	-27.91,	-28.38,	-29.53,	-31.37,	-33.97,	-37.38,	-41.74,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.28,	-47.00,	-46.73,	-46.46,	-46.19,	-45.93,	-45.66,	-45.40,	-45.14,	-44.88,	-44.62,	-44.36,	-44.11,	-43.86,	-43.60,	-43.35,	-43.10,	-42.86,	-42.61,	-42.36,	-42.12,	-41.88,	-41.64,	-41.40,	-41.16,	-40.92,	-40.69,	-40.45,	-40.22,	-39.99,	-39.75,	-39.52,	-39.30,	-39.07,	-38.84,	-38.62,	-38.39,	-38.17,	-37.95,	-37.72,	-37.50,	-37.29,	-37.07,	-36.85,	-36.63,	-36.42,	-36.21,	-35.99,	-35.78,	-35.57,	-35.36,	-35.15,	-34.94,	-34.73,	-34.53,	-34.32,	-34.11,	-33.91,	-33.71,	-33.51,	-33.30,	-33.10,	-32.90,	-32.70,	-32.51,	-32.31,	-32.11,	-31.91,	-31.72,	-31.52,	-31.33,	-31.14,	-30.95,	-30.75,	-30.56,	-30.37,	-30.18,	-30.00,	-29.81,	-29.62,	-29.43,	-29.25,	-29.06,	-28.88,	-28.70,	-28.51,	-28.33,	-28.15,	-27.97,	-27.79,	-27.61,	-27.43,	-27.25,	-27.07,	-26.89,	-26.72,	-26.54,	-26.36,	-26.19,	-26.02,	-25.84,	-25.67,	-25.50,	-25.32,	-25.15,	-24.98,	-24.81,	-24.64,	-24.47,	-24.30,	-24.14,	-23.97,	-23.80,	-23.64,	-23.47,	-23.30,	-23.14,	-22.98,	-22.81,	-22.65,	-22.49,	-22.32,	-22.16,	-22.00,	-21.84,	-21.68,	-21.52,	-21.36,	-21.20,	-21.05,	-20.89,	-20.73,	-20.57,	-20.42,	-20.26,	-20.11,	-19.95,	-19.80,	-19.64,	-19.49,	-19.34,	-19.19,	-19.03,	-18.88,	-18.73,	-18.58,	-18.43,	-18.28,	-18.13,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.84,	-17.69,	-17.54,	-17.39,	-17.25,	-17.10,	-16.95,	-16.81,	-16.66,	-16.52,	-16.38,	-16.23,	-16.09,	-15.95,	-15.80,	-15.66,	-15.52,	-15.38,	-15.24,	-15.10,	-14.96,	-14.82,	-14.68,	-14.54,	-14.40,	-14.27,	-14.13,	-13.99,	-13.86,	-13.72,	-13.58,	-13.45,	-13.31,	-13.18,	-13.04,	-12.91,	-12.78,	-12.64,	-12.51,	-12.38,	-12.25,	-12.11,	-11.98,	-11.85,	-11.72,	-11.59,	-11.46,	-11.33,	-11.20,	-11.07,	-10.95,	-10.82,	-10.69,	-10.56,	-10.44,	-10.31,	-10.18,	-10.06,	-9.93,	-9.81,	-9.68,	-9.56,	-9.43,	-9.31,	-9.19,	-9.06,	-8.94,	-8.82,	-8.70,	-8.57,	-8.45,	-8.33,	-8.21,	-8.09,	-7.97,	-7.85,	-7.73,	-7.61,	-7.50,	-7.38,	-7.26,	-7.14,	-7.02,	-6.91,	-6.79,	-6.68,	-6.56,	-6.44,	-6.33,	-6.21,	-6.10,	-5.98,	-5.87,	-5.76,	-5.64,	-5.53,	-5.42,	-5.31,	-5.19,	-5.08,	-4.97,	-4.86,	-4.75,	-4.64,	-4.53,	-4.42,	-4.31,	-4.20,	-4.09,	-3.98,	-3.88,	-3.77,	-3.66,	-3.55,	-3.45,	-3.34,	-3.23,	-3.13,	-3.02,	-2.92,	-2.81,	-2.71,	-2.60,	-2.50,	-2.40,	-2.29,	-2.19,	-2.09,	-1.98,	-1.88,	-1.78,	-1.68,	-1.58,	-1.48,	-1.38,	-1.28,	-1.18,	-1.08,	-0.98,	-0.88,	-0.78,	-0.68,	-0.58,	-0.48,	-0.39,	-0.29,	-0.19,	-0.10,	0.00};
//	float LineUptheta1_LF[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.10,	0.19,	0.29,	0.38,	0.48,	0.57,	0.67,	0.76,	0.85,	0.95,	1.04,	1.13,	1.22,	1.32,	1.41,	1.50,	1.59,	1.68,	1.77,	1.86,	1.95,	2.04,	2.13,	2.22,	2.31,	2.40,	2.49,	2.58,	2.66,	2.75,	2.84,	2.92,	3.01,	3.10,	3.18,	3.27,	3.35,	3.44,	3.52,	3.61,	3.69,	3.78,	3.86,	3.94,	4.03,	4.11,	4.19,	4.27,	4.35,	4.44,	4.52,	4.60,	4.68,	4.76,	4.84,	4.92,	5.00,	5.08,	5.16,	5.23,	5.31,	5.39,	5.47,	5.54,	5.62,	5.70,	5.77,	5.85,	5.93,	6.00,	6.08,	6.15,	6.23,	6.30,	6.38,	6.45,	6.52,	6.60,	6.67,	6.74,	6.81,	6.89,	6.96,	7.03,	7.10,	7.17,	7.24,	7.31,	7.38,	7.45,	7.52,	7.59,	7.66,	7.73,	7.80,	7.86,	7.93,	8.00,	8.07,	8.13,	8.20,	8.27,	8.33,	8.40,	8.46,	8.53,	8.59,	8.66,	8.72,	8.78,	8.85,	8.91,	8.97,	9.04,	9.10,	9.16,	9.22,	9.28,	9.34,	9.41,	9.47,	9.53,	9.59,	9.65,	9.71,	9.76,	9.82,	9.88,	9.94,	10.00,	10.06,	10.11,	10.17,	10.23,	10.28,	10.34,	10.39,	10.45,	10.50,	10.56,	10.61,	10.67,	10.72,	10.78,	10.83,	10.88,	10.94,	10.99,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.04,	11.45,	11.85,	12.24,	12.61,	12.97,	13.32,	13.66,	13.99,	14.30,	14.60,	14.89,	15.17,	15.44,	15.70,	15.94,	16.18,	16.40,	16.61,	16.82,	17.01,	17.19,	17.36,	17.52,	17.67,	17.82,	17.95,	18.07,	18.18,	18.29,	18.38,	18.47,	18.55,	18.61,	18.67,	18.72,	18.77,	18.80,	18.83,	18.85,	18.86,	18.86,	18.86,	18.85,	18.83,	18.80,	18.77,	18.73,	18.68,	18.63,	18.57,	18.50,	18.43,	18.35,	18.26,	18.17,	18.08,	17.98,	17.87,	17.76,	17.64,	17.52,	17.39,	17.25,	17.12,	16.97,	16.83,	16.68,	16.52,	16.36,	16.20,	16.03,	15.86,	15.68,	15.50,	15.32,	15.13,	14.94,	14.75,	14.55,	14.35,	14.15,	13.95,	13.74,	13.53,	13.31,	13.10,	12.88,	12.66,	12.43,	12.21,	11.98,	11.75,	11.52,	11.28,	11.04,	10.81,	10.57,	10.32,	10.08,	9.83,	9.59,	9.34,	9.09,	8.84,	8.58,	8.33,	8.08,	7.82,	7.56,	7.30,	7.04,	6.78,	6.52,	6.26,	5.99,	5.73,	5.46,	5.19,	4.93,	4.66,	4.39,	4.12,	3.85,	3.58,	3.31,	3.03,	2.76,	2.49,	2.21,	1.94,	1.67,	1.39,	1.11,	0.84,	0.56,	0.28,	0.01,	-0.27,	-0.55,	-0.83,	-1.11,	-1.39,	-1.67,	-1.95,	-2.23,	-2.51,	-2.80,	-3.08,	-3.36,	-3.36,	-2.59,	-2.42,	-2.83,	-3.83,	-5.43,	-7.63,	-10.43,	-13.87,	-17.98,	-17.98,	-17.84,	-17.69,	-17.54,	-17.39,	-17.25,	-17.10,	-16.95,	-16.81,	-16.66,	-16.52,	-16.38,	-16.23,	-16.09,	-15.95,	-15.80,	-15.66,	-15.52,	-15.38,	-15.24,	-15.10,	-14.96,	-14.82,	-14.68,	-14.54,	-14.40,	-14.27,	-14.13,	-13.99,	-13.86,	-13.72,	-13.58,	-13.45,	-13.31,	-13.18,	-13.04,	-12.91,	-12.78,	-12.64,	-12.51,	-12.38,	-12.25,	-12.11,	-11.98,	-11.85,	-11.72,	-11.59,	-11.46,	-11.33,	-11.20,	-11.07,	-10.95,	-10.82,	-10.69,	-10.56,	-10.44,	-10.31,	-10.18,	-10.06,	-9.93,	-9.81,	-9.68,	-9.56,	-9.43,	-9.31,	-9.19,	-9.06,	-8.94,	-8.82,	-8.70,	-8.57,	-8.45,	-8.33,	-8.21,	-8.09,	-7.97,	-7.85,	-7.73,	-7.61,	-7.50,	-7.38,	-7.26,	-7.14,	-7.02,	-6.91,	-6.79,	-6.68,	-6.56,	-6.44,	-6.33,	-6.21,	-6.10,	-5.98,	-5.87,	-5.76,	-5.64,	-5.53,	-5.42,	-5.31,	-5.19,	-5.08,	-4.97,	-4.86,	-4.75,	-4.64,	-4.53,	-4.42,	-4.31,	-4.20,	-4.09,	-3.98,	-3.88,	-3.77,	-3.66,	-3.55,	-3.45,	-3.34,	-3.23,	-3.13,	-3.02,	-2.92,	-2.81,	-2.71,	-2.60,	-2.50,	-2.40,	-2.29,	-2.19,	-2.09,	-1.98,	-1.88,	-1.78,	-1.68,	-1.58,	-1.48,	-1.38,	-1.28,	-1.18,	-1.08,	-0.98,	-0.88,	-0.78,	-0.68,	-0.58,	-0.48,	-0.39,	-0.29,	-0.19,	-0.10,	0.00};
//	float LineUptheta1_RR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.10,	0.19,	0.29,	0.38,	0.48,	0.57,	0.67,	0.76,	0.85,	0.95,	1.04,	1.13,	1.22,	1.32,	1.41,	1.50,	1.59,	1.68,	1.77,	1.86,	1.95,	2.04,	2.13,	2.22,	2.31,	2.40,	2.49,	2.58,	2.66,	2.75,	2.84,	2.92,	3.01,	3.10,	3.18,	3.27,	3.35,	3.44,	3.52,	3.61,	3.69,	3.78,	3.86,	3.94,	4.03,	4.11,	4.19,	4.27,	4.35,	4.44,	4.52,	4.60,	4.68,	4.76,	4.84,	4.92,	5.00,	5.08,	5.16,	5.23,	5.31,	5.39,	5.47,	5.54,	5.62,	5.70,	5.77,	5.85,	5.93,	6.00,	6.08,	6.15,	6.23,	6.30,	6.38,	6.45,	6.52,	6.60,	6.67,	6.74,	6.81,	6.89,	6.96,	7.03,	7.10,	7.17,	7.24,	7.31,	7.38,	7.45,	7.52,	7.59,	7.66,	7.73,	7.80,	7.86,	7.93,	8.00,	8.07,	8.13,	8.20,	8.27,	8.33,	8.40,	8.46,	8.53,	8.59,	8.66,	8.72,	8.78,	8.85,	8.91,	8.97,	9.04,	9.10,	9.16,	9.22,	9.28,	9.34,	9.41,	9.47,	9.53,	9.59,	9.65,	9.71,	9.76,	9.82,	9.88,	9.94,	10.00,	10.06,	10.11,	10.17,	10.23,	10.28,	10.34,	10.39,	10.45,	10.50,	10.56,	10.61,	10.67,	10.72,	10.78,	10.83,	10.88,	10.94,	10.99,	11.04,	11.04,	11.45,	11.85,	12.24,	12.61,	12.97,	13.32,	13.66,	13.99,	14.30,	14.60,	14.89,	15.17,	15.44,	15.70,	15.94,	16.18,	16.40,	16.61,	16.82,	17.01,	17.19,	17.36,	17.52,	17.67,	17.82,	17.95,	18.07,	18.18,	18.29,	18.38,	18.47,	18.55,	18.61,	18.67,	18.72,	18.77,	18.80,	18.83,	18.85,	18.86,	18.86,	18.86,	18.85,	18.83,	18.80,	18.77,	18.73,	18.68,	18.63,	18.57,	18.50,	18.43,	18.35,	18.26,	18.17,	18.08,	17.98,	17.87,	17.76,	17.64,	17.52,	17.39,	17.25,	17.12,	16.97,	16.83,	16.68,	16.52,	16.36,	16.20,	16.03,	15.86,	15.68,	15.50,	15.32,	15.13,	14.94,	14.75,	14.55,	14.35,	14.15,	13.95,	13.74,	13.53,	13.31,	13.10,	12.88,	12.66,	12.43,	12.21,	11.98,	11.75,	11.52,	11.28,	11.04,	10.81,	10.57,	10.32,	10.08,	9.83,	9.59,	9.34,	9.09,	8.84,	8.58,	8.33,	8.08,	7.82,	7.56,	7.30,	7.04,	6.78,	6.52,	6.26,	5.99,	5.73,	5.46,	5.19,	4.93,	4.66,	4.39,	4.12,	3.85,	3.58,	3.31,	3.03,	2.76,	2.49,	2.21,	1.94,	1.67,	1.39,	1.11,	0.84,	0.56,	0.28,	0.01,	-0.27,	-0.55,	-0.83,	-1.11,	-1.39,	-1.67,	-1.95,	-2.23,	-2.51,	-2.80,	-3.08,	-3.36,	-3.36,	-2.59,	-2.42,	-2.83,	-3.83,	-5.43,	-7.63,	-10.43,	-13.87,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.84,	-17.69,	-17.54,	-17.39,	-17.25,	-17.10,	-16.95,	-16.81,	-16.66,	-16.52,	-16.38,	-16.23,	-16.09,	-15.95,	-15.80,	-15.66,	-15.52,	-15.38,	-15.24,	-15.10,	-14.96,	-14.82,	-14.68,	-14.54,	-14.40,	-14.27,	-14.13,	-13.99,	-13.86,	-13.72,	-13.58,	-13.45,	-13.31,	-13.18,	-13.04,	-12.91,	-12.78,	-12.64,	-12.51,	-12.38,	-12.25,	-12.11,	-11.98,	-11.85,	-11.72,	-11.59,	-11.46,	-11.33,	-11.20,	-11.07,	-10.95,	-10.82,	-10.69,	-10.56,	-10.44,	-10.31,	-10.18,	-10.06,	-9.93,	-9.81,	-9.68,	-9.56,	-9.43,	-9.31,	-9.19,	-9.06,	-8.94,	-8.82,	-8.70,	-8.57,	-8.45,	-8.33,	-8.21,	-8.09,	-7.97,	-7.85,	-7.73,	-7.61,	-7.50,	-7.38,	-7.26,	-7.14,	-7.02,	-6.91,	-6.79,	-6.68,	-6.56,	-6.44,	-6.33,	-6.21,	-6.10,	-5.98,	-5.87,	-5.76,	-5.64,	-5.53,	-5.42,	-5.31,	-5.19,	-5.08,	-4.97,	-4.86,	-4.75,	-4.64,	-4.53,	-4.42,	-4.31,	-4.20,	-4.09,	-3.98,	-3.88,	-3.77,	-3.66,	-3.55,	-3.45,	-3.34,	-3.23,	-3.13,	-3.02,	-2.92,	-2.81,	-2.71,	-2.60,	-2.50,	-2.40,	-2.29,	-2.19,	-2.09,	-1.98,	-1.88,	-1.78,	-1.68,	-1.58,	-1.48,	-1.38,	-1.28,	-1.18,	-1.08,	-0.98,	-0.88,	-0.78,	-0.68,	-0.58,	-0.48,	-0.39,	-0.29,	-0.19,	-0.10,	0.00};
//	float LineUptheta1_LR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.31,	0.61,	0.90,	1.18,	1.44,	1.69,	1.93,	2.16,	2.37,	2.57,	2.76,	2.94,	3.11,	3.27,	3.41,	3.55,	3.67,	3.79,	3.89,	3.99,	4.07,	4.14,	4.21,	4.26,	4.31,	4.34,	4.37,	4.39,	4.40,	4.40,	4.39,	4.37,	4.34,	4.31,	4.27,	4.22,	4.16,	4.10,	4.02,	3.94,	3.86,	3.76,	3.66,	3.55,	3.44,	3.32,	3.19,	3.05,	2.91,	2.76,	2.61,	2.45,	2.29,	2.12,	1.94,	1.76,	1.57,	1.38,	1.18,	0.98,	0.77,	0.56,	0.34,	0.12,	-0.11,	-0.34,	-0.57,	-0.81,	-1.05,	-1.30,	-1.55,	-1.81,	-2.07,	-2.33,	-2.60,	-2.87,	-3.14,	-3.42,	-3.70,	-3.98,	-4.27,	-4.56,	-4.85,	-5.15,	-5.45,	-5.75,	-6.06,	-6.37,	-6.68,	-6.99,	-7.31,	-7.62,	-7.95,	-8.27,	-8.59,	-8.92,	-9.25,	-9.59,	-9.92,	-10.26,	-10.60,	-10.94,	-11.28,	-11.63,	-11.97,	-12.32,	-12.68,	-13.03,	-13.38,	-13.74,	-14.10,	-14.46,	-14.82,	-15.19,	-15.55,	-15.92,	-16.29,	-16.66,	-17.03,	-17.41,	-17.78,	-18.16,	-18.54,	-18.92,	-19.30,	-19.69,	-20.07,	-20.46,	-20.85,	-21.24,	-21.63,	-22.02,	-22.42,	-22.82,	-23.21,	-23.61,	-24.02,	-24.42,	-24.83,	-25.23,	-25.64,	-26.05,	-26.47,	-26.88,	-27.30,	-27.72,	-28.14,	-28.56,	-28.99,	-28.99,	-28.11,	-27.91,	-28.38,	-29.53,	-31.37,	-33.97,	-37.38,	-41.74,	-47.28,	-47.28,	-47.00,	-46.73,	-46.46,	-46.19,	-45.93,	-45.66,	-45.40,	-45.14,	-44.88,	-44.62,	-44.36,	-44.11,	-43.86,	-43.60,	-43.35,	-43.10,	-42.86,	-42.61,	-42.36,	-42.12,	-41.88,	-41.64,	-41.40,	-41.16,	-40.92,	-40.69,	-40.45,	-40.22,	-39.99,	-39.75,	-39.52,	-39.30,	-39.07,	-38.84,	-38.62,	-38.39,	-38.17,	-37.95,	-37.72,	-37.50,	-37.29,	-37.07,	-36.85,	-36.63,	-36.42,	-36.21,	-35.99,	-35.78,	-35.57,	-35.36,	-35.15,	-34.94,	-34.73,	-34.53,	-34.32,	-34.11,	-33.91,	-33.71,	-33.51,	-33.30,	-33.10,	-32.90,	-32.70,	-32.51,	-32.31,	-32.11,	-31.91,	-31.72,	-31.52,	-31.33,	-31.14,	-30.95,	-30.75,	-30.56,	-30.37,	-30.18,	-30.00,	-29.81,	-29.62,	-29.43,	-29.25,	-29.06,	-28.88,	-28.70,	-28.51,	-28.33,	-28.15,	-27.97,	-27.79,	-27.61,	-27.43,	-27.25,	-27.07,	-26.89,	-26.72,	-26.54,	-26.36,	-26.19,	-26.02,	-25.84,	-25.67,	-25.50,	-25.32,	-25.15,	-24.98,	-24.81,	-24.64,	-24.47,	-24.30,	-24.14,	-23.97,	-23.80,	-23.64,	-23.47,	-23.30,	-23.14,	-22.98,	-22.81,	-22.65,	-22.49,	-22.32,	-22.16,	-22.00,	-21.84,	-21.68,	-21.52,	-21.36,	-21.20,	-21.05,	-20.89,	-20.73,	-20.57,	-20.42,	-20.26,	-20.11,	-19.95,	-19.80,	-19.64,	-19.49,	-19.34,	-19.19,	-19.03,	-18.88,	-18.73,	-18.58,	-18.43,	-18.28,	-18.13,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.98,	-17.84,	-17.69,	-17.54,	-17.39,	-17.25,	-17.10,	-16.95,	-16.81,	-16.66,	-16.52,	-16.38,	-16.23,	-16.09,	-15.95,	-15.80,	-15.66,	-15.52,	-15.38,	-15.24,	-15.10,	-14.96,	-14.82,	-14.68,	-14.54,	-14.40,	-14.27,	-14.13,	-13.99,	-13.86,	-13.72,	-13.58,	-13.45,	-13.31,	-13.18,	-13.04,	-12.91,	-12.78,	-12.64,	-12.51,	-12.38,	-12.25,	-12.11,	-11.98,	-11.85,	-11.72,	-11.59,	-11.46,	-11.33,	-11.20,	-11.07,	-10.95,	-10.82,	-10.69,	-10.56,	-10.44,	-10.31,	-10.18,	-10.06,	-9.93,	-9.81,	-9.68,	-9.56,	-9.43,	-9.31,	-9.19,	-9.06,	-8.94,	-8.82,	-8.70,	-8.57,	-8.45,	-8.33,	-8.21,	-8.09,	-7.97,	-7.85,	-7.73,	-7.61,	-7.50,	-7.38,	-7.26,	-7.14,	-7.02,	-6.91,	-6.79,	-6.68,	-6.56,	-6.44,	-6.33,	-6.21,	-6.10,	-5.98,	-5.87,	-5.76,	-5.64,	-5.53,	-5.42,	-5.31,	-5.19,	-5.08,	-4.97,	-4.86,	-4.75,	-4.64,	-4.53,	-4.42,	-4.31,	-4.20,	-4.09,	-3.98,	-3.88,	-3.77,	-3.66,	-3.55,	-3.45,	-3.34,	-3.23,	-3.13,	-3.02,	-2.92,	-2.81,	-2.71,	-2.60,	-2.50,	-2.40,	-2.29,	-2.19,	-2.09,	-1.98,	-1.88,	-1.78,	-1.68,	-1.58,	-1.48,	-1.38,	-1.28,	-1.18,	-1.08,	-0.98,	-0.88,	-0.78,	-0.68,	-0.58,	-0.48,	-0.39,	-0.29,	-0.19,	-0.10,	0.00};	
//	
//	float LineUptheta2_RF[LTOTALSTEPS] = {0.00,	0.19,	0.39,	0.59,	0.79,	0.99,	1.19,	1.40,	1.60,	1.81,	2.02,	2.23,	2.44,	2.66,	2.87,	3.08,	3.30,	3.51,	3.73,	3.94,	4.16,	4.38,	4.59,	4.81,	5.02,	5.24,	5.46,	5.67,	5.89,	6.10,	6.32,	6.53,	6.74,	6.96,	7.17,	7.38,	7.59,	7.81,	8.02,	8.23,	8.44,	8.64,	8.85,	9.06,	9.27,	9.47,	9.68,	9.88,	10.08,	10.29,	10.49,	10.69,	10.89,	11.09,	11.29,	11.49,	11.69,	11.89,	12.09,	12.28,	12.48,	12.67,	12.87,	13.06,	13.26,	13.45,	13.64,	13.84,	14.03,	14.22,	14.41,	14.60,	14.80,	14.99,	15.18,	15.37,	15.56,	15.75,	15.93,	16.12,	16.31,	16.50,	16.69,	16.88,	17.07,	17.26,	17.45,	17.63,	17.82,	18.01,	18.20,	18.39,	18.58,	18.77,	18.96,	19.14,	19.33,	19.52,	19.71,	19.90,	20.09,	20.29,	20.48,	20.67,	20.86,	21.05,	21.24,	21.44,	21.63,	21.82,	22.02,	22.21,	22.40,	22.60,	22.79,	22.99,	23.19,	23.38,	23.58,	23.78,	23.98,	24.18,	24.38,	24.58,	24.78,	24.98,	25.18,	25.38,	25.59,	25.79,	25.99,	26.20,	26.41,	26.61,	26.82,	27.03,	27.24,	27.45,	27.66,	27.87,	28.08,	28.29,	28.50,	28.72,	28.93,	29.15,	29.37,	29.58,	29.80,	30.02,	30.02,	30.04,	30.04,	30.03,	30.02,	30.02,	30.10,	30.29,	30.67,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.37,	31.24,	31.10,	30.97,	30.83,	30.70,	30.57,	30.43,	30.30,	30.17,	30.04,	29.91,	29.78,	29.65,	29.52,	29.39,	29.26,	29.14,	29.01,	28.88,	28.76,	28.63,	28.51,	28.38,	28.26,	28.13,	28.01,	27.88,	27.76,	27.64,	27.52,	27.39,	27.27,	27.15,	27.03,	26.91,	26.79,	26.67,	26.55,	26.43,	26.31,	26.19,	26.07,	25.95,	25.84,	25.72,	25.60,	25.49,	25.37,	25.25,	25.14,	25.02,	24.90,	24.79,	24.67,	24.56,	24.44,	24.33,	24.22,	24.10,	23.99,	23.87,	23.76,	23.65,	23.54,	23.42,	23.31,	23.20,	23.09,	22.98,	22.86,	22.75,	22.64,	22.53,	22.42,	22.31,	22.20,	22.09,	21.98,	21.87,	21.76,	21.65,	21.54,	21.43,	21.32,	21.22,	21.11,	21.00,	20.89,	20.78,	20.67,	20.57,	20.46,	20.35,	20.24,	20.14,	20.03,	19.92,	19.82,	19.71,	19.60,	19.50,	19.39,	19.29,	19.18,	19.08,	18.97,	18.86,	18.76,	18.65,	18.55,	18.44,	18.34,	18.24,	18.13,	18.03,	17.92,	17.82,	17.71,	17.61,	17.51,	17.40,	17.30,	17.20,	17.09,	16.99,	16.89,	16.78,	16.68,	16.58,	16.48,	16.37,	16.27,	16.17,	16.07,	15.96,	15.86,	15.76,	15.66,	15.56,	15.45,	15.35,	15.25,	15.15,	15.05,	14.95,	14.84,	14.74,	14.64,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.44,	14.34,	14.24,	14.14,	14.04,	13.94,	13.84,	13.74,	13.64,	13.54,	13.44,	13.34,	13.24,	13.14,	13.04,	12.94,	12.84,	12.74,	12.64,	12.54,	12.44,	12.34,	12.24,	12.14,	12.04,	11.94,	11.84,	11.74,	11.64,	11.55,	11.45,	11.35,	11.25,	11.15,	11.05,	10.95,	10.85,	10.76,	10.66,	10.56,	10.46,	10.36,	10.26,	10.17,	10.07,	9.97,	9.87,	9.77,	9.67,	9.58,	9.48,	9.38,	9.28,	9.19,	9.09,	8.99,	8.89,	8.79,	8.70,	8.60,	8.50,	8.40,	8.31,	8.21,	8.11,	8.01,	7.92,	7.82,	7.72,	7.63,	7.53,	7.43,	7.33,	7.24,	7.14,	7.04,	6.95,	6.85,	6.75,	6.65,	6.56,	6.46,	6.36,	6.27,	6.17,	6.07,	5.98,	5.88,	5.78,	5.69,	5.59,	5.49,	5.40,	5.30,	5.20,	5.11,	5.01,	4.91,	4.82,	4.72,	4.62,	4.53,	4.43,	4.33,	4.24,	4.14,	4.04,	3.95,	3.85,	3.75,	3.66,	3.56,	3.46,	3.37,	3.27,	3.18,	3.08,	2.98,	2.89,	2.79,	2.69,	2.60,	2.50,	2.41,	2.31,	2.21,	2.12,	2.02,	1.92,	1.83,	1.73,	1.64,	1.54,	1.44,	1.35,	1.25,	1.15,	1.06,	0.96,	0.87,	0.77,	0.67,	0.58,	0.48,	0.38,	0.29,	0.19,	0.10,	0.00};
//	float LineUptheta2_LF[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	-0.10,	-0.19,	-0.29,	-0.38,	-0.48,	-0.58,	-0.67,	-0.77,	-0.87,	-0.96,	-1.06,	-1.15,	-1.25,	-1.35,	-1.44,	-1.54,	-1.64,	-1.73,	-1.83,	-1.92,	-2.02,	-2.12,	-2.21,	-2.31,	-2.41,	-2.50,	-2.60,	-2.69,	-2.79,	-2.89,	-2.98,	-3.08,	-3.18,	-3.27,	-3.37,	-3.46,	-3.56,	-3.66,	-3.75,	-3.85,	-3.95,	-4.04,	-4.14,	-4.24,	-4.33,	-4.43,	-4.52,	-4.62,	-4.72,	-4.81,	-4.91,	-5.01,	-5.10,	-5.20,	-5.30,	-5.39,	-5.49,	-5.59,	-5.68,	-5.78,	-5.88,	-5.97,	-6.07,	-6.17,	-6.26,	-6.36,	-6.46,	-6.55,	-6.65,	-6.75,	-6.84,	-6.94,	-7.04,	-7.13,	-7.23,	-7.33,	-7.42,	-7.52,	-7.62,	-7.71,	-7.81,	-7.91,	-8.00,	-8.10,	-8.20,	-8.29,	-8.39,	-8.49,	-8.59,	-8.68,	-8.78,	-8.88,	-8.97,	-9.07,	-9.17,	-9.26,	-9.36,	-9.46,	-9.56,	-9.65,	-9.75,	-9.85,	-9.94,	-10.04,	-10.14,	-10.24,	-10.33,	-10.43,	-10.53,	-10.63,	-10.72,	-10.82,	-10.92,	-11.01,	-11.11,	-11.21,	-11.31,	-11.40,	-11.50,	-11.60,	-11.70,	-11.79,	-11.89,	-11.99,	-12.09,	-12.18,	-12.28,	-12.38,	-12.48,	-12.57,	-12.67,	-12.77,	-12.87,	-12.96,	-13.06,	-13.16,	-13.26,	-13.35,	-13.45,	-13.55,	-13.65,	-13.75,	-13.84,	-13.94,	-14.04,	-14.14,	-14.23,	-14.33,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.43,	-14.25,	-14.06,	-13.88,	-13.69,	-13.49,	-13.30,	-13.10,	-12.90,	-12.69,	-12.49,	-12.28,	-12.07,	-11.86,	-11.64,	-11.43,	-11.21,	-10.99,	-10.77,	-10.55,	-10.33,	-10.11,	-9.89,	-9.67,	-9.44,	-9.22,	-8.99,	-8.77,	-8.55,	-8.32,	-8.10,	-7.87,	-7.65,	-7.42,	-7.20,	-6.98,	-6.75,	-6.53,	-6.31,	-6.09,	-5.86,	-5.64,	-5.42,	-5.21,	-4.99,	-4.77,	-4.55,	-4.34,	-4.12,	-3.91,	-3.69,	-3.48,	-3.27,	-3.06,	-2.85,	-2.64,	-2.43,	-2.22,	-2.01,	-1.81,	-1.60,	-1.40,	-1.19,	-0.99,	-0.79,	-0.59,	-0.39,	-0.19,	0.01,	0.21,	0.41,	0.60,	0.80,	1.00,	1.19,	1.39,	1.58,	1.77,	1.96,	2.16,	2.35,	2.54,	2.73,	2.92,	3.11,	3.30,	3.48,	3.67,	3.86,	4.05,	4.23,	4.42,	4.61,	4.79,	4.98,	5.16,	5.35,	5.53,	5.72,	5.90,	6.09,	6.27,	6.46,	6.64,	6.82,	7.01,	7.19,	7.38,	7.56,	7.74,	7.93,	8.11,	8.29,	8.48,	8.66,	8.84,	9.03,	9.21,	9.39,	9.58,	9.76,	9.95,	10.13,	10.31,	10.50,	10.68,	10.87,	11.05,	11.24,	11.42,	11.60,	11.79,	11.98,	12.16,	12.35,	12.53,	12.72,	12.90,	13.09,	13.28,	13.46,	13.65,	13.84,	14.02,	14.21,	14.40,	14.59,	14.77,	14.96,	15.15,	15.15,	15.25,	15.28,	15.22,	15.09,	14.92,	14.73,	14.57,	14.49,	14.54,	14.54,	14.44,	14.34,	14.24,	14.14,	14.04,	13.94,	13.84,	13.74,	13.64,	13.54,	13.44,	13.34,	13.24,	13.14,	13.04,	12.94,	12.84,	12.74,	12.64,	12.54,	12.44,	12.34,	12.24,	12.14,	12.04,	11.94,	11.84,	11.74,	11.64,	11.55,	11.45,	11.35,	11.25,	11.15,	11.05,	10.95,	10.85,	10.76,	10.66,	10.56,	10.46,	10.36,	10.26,	10.17,	10.07,	9.97,	9.87,	9.77,	9.67,	9.58,	9.48,	9.38,	9.28,	9.19,	9.09,	8.99,	8.89,	8.79,	8.70,	8.60,	8.50,	8.40,	8.31,	8.21,	8.11,	8.01,	7.92,	7.82,	7.72,	7.63,	7.53,	7.43,	7.33,	7.24,	7.14,	7.04,	6.95,	6.85,	6.75,	6.65,	6.56,	6.46,	6.36,	6.27,	6.17,	6.07,	5.98,	5.88,	5.78,	5.69,	5.59,	5.49,	5.40,	5.30,	5.20,	5.11,	5.01,	4.91,	4.82,	4.72,	4.62,	4.53,	4.43,	4.33,	4.24,	4.14,	4.04,	3.95,	3.85,	3.75,	3.66,	3.56,	3.46,	3.37,	3.27,	3.18,	3.08,	2.98,	2.89,	2.79,	2.69,	2.60,	2.50,	2.41,	2.31,	2.21,	2.12,	2.02,	1.92,	1.83,	1.73,	1.64,	1.54,	1.44,	1.35,	1.25,	1.15,	1.06,	0.96,	0.87,	0.77,	0.67,	0.58,	0.48,	0.38,	0.29,	0.19,	0.10,	0.00};
//	float LineUptheta2_RR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	-0.10,	-0.19,	-0.29,	-0.38,	-0.48,	-0.58,	-0.67,	-0.77,	-0.87,	-0.96,	-1.06,	-1.15,	-1.25,	-1.35,	-1.44,	-1.54,	-1.64,	-1.73,	-1.83,	-1.92,	-2.02,	-2.12,	-2.21,	-2.31,	-2.41,	-2.50,	-2.60,	-2.69,	-2.79,	-2.89,	-2.98,	-3.08,	-3.18,	-3.27,	-3.37,	-3.46,	-3.56,	-3.66,	-3.75,	-3.85,	-3.95,	-4.04,	-4.14,	-4.24,	-4.33,	-4.43,	-4.52,	-4.62,	-4.72,	-4.81,	-4.91,	-5.01,	-5.10,	-5.20,	-5.30,	-5.39,	-5.49,	-5.59,	-5.68,	-5.78,	-5.88,	-5.97,	-6.07,	-6.17,	-6.26,	-6.36,	-6.46,	-6.55,	-6.65,	-6.75,	-6.84,	-6.94,	-7.04,	-7.13,	-7.23,	-7.33,	-7.42,	-7.52,	-7.62,	-7.71,	-7.81,	-7.91,	-8.00,	-8.10,	-8.20,	-8.29,	-8.39,	-8.49,	-8.59,	-8.68,	-8.78,	-8.88,	-8.97,	-9.07,	-9.17,	-9.26,	-9.36,	-9.46,	-9.56,	-9.65,	-9.75,	-9.85,	-9.94,	-10.04,	-10.14,	-10.24,	-10.33,	-10.43,	-10.53,	-10.63,	-10.72,	-10.82,	-10.92,	-11.01,	-11.11,	-11.21,	-11.31,	-11.40,	-11.50,	-11.60,	-11.70,	-11.79,	-11.89,	-11.99,	-12.09,	-12.18,	-12.28,	-12.38,	-12.48,	-12.57,	-12.67,	-12.77,	-12.87,	-12.96,	-13.06,	-13.16,	-13.26,	-13.35,	-13.45,	-13.55,	-13.65,	-13.75,	-13.84,	-13.94,	-14.04,	-14.14,	-14.23,	-14.33,	-14.43,	-14.43,	-14.25,	-14.06,	-13.88,	-13.69,	-13.49,	-13.30,	-13.10,	-12.90,	-12.69,	-12.49,	-12.28,	-12.07,	-11.86,	-11.64,	-11.43,	-11.21,	-10.99,	-10.77,	-10.55,	-10.33,	-10.11,	-9.89,	-9.67,	-9.44,	-9.22,	-8.99,	-8.77,	-8.55,	-8.32,	-8.10,	-7.87,	-7.65,	-7.42,	-7.20,	-6.98,	-6.75,	-6.53,	-6.31,	-6.09,	-5.86,	-5.64,	-5.42,	-5.21,	-4.99,	-4.77,	-4.55,	-4.34,	-4.12,	-3.91,	-3.69,	-3.48,	-3.27,	-3.06,	-2.85,	-2.64,	-2.43,	-2.22,	-2.01,	-1.81,	-1.60,	-1.40,	-1.19,	-0.99,	-0.79,	-0.59,	-0.39,	-0.19,	0.01,	0.21,	0.41,	0.60,	0.80,	1.00,	1.19,	1.39,	1.58,	1.77,	1.96,	2.16,	2.35,	2.54,	2.73,	2.92,	3.11,	3.30,	3.48,	3.67,	3.86,	4.05,	4.23,	4.42,	4.61,	4.79,	4.98,	5.16,	5.35,	5.53,	5.72,	5.90,	6.09,	6.27,	6.46,	6.64,	6.82,	7.01,	7.19,	7.38,	7.56,	7.74,	7.93,	8.11,	8.29,	8.48,	8.66,	8.84,	9.03,	9.21,	9.39,	9.58,	9.76,	9.95,	10.13,	10.31,	10.50,	10.68,	10.87,	11.05,	11.24,	11.42,	11.60,	11.79,	11.98,	12.16,	12.35,	12.53,	12.72,	12.90,	13.09,	13.28,	13.46,	13.65,	13.84,	14.02,	14.21,	14.40,	14.59,	14.77,	14.96,	15.15,	15.15,	15.25,	15.28,	15.22,	15.09,	14.92,	14.73,	14.57,	14.49,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.44,	14.34,	14.24,	14.14,	14.04,	13.94,	13.84,	13.74,	13.64,	13.54,	13.44,	13.34,	13.24,	13.14,	13.04,	12.94,	12.84,	12.74,	12.64,	12.54,	12.44,	12.34,	12.24,	12.14,	12.04,	11.94,	11.84,	11.74,	11.64,	11.55,	11.45,	11.35,	11.25,	11.15,	11.05,	10.95,	10.85,	10.76,	10.66,	10.56,	10.46,	10.36,	10.26,	10.17,	10.07,	9.97,	9.87,	9.77,	9.67,	9.58,	9.48,	9.38,	9.28,	9.19,	9.09,	8.99,	8.89,	8.79,	8.70,	8.60,	8.50,	8.40,	8.31,	8.21,	8.11,	8.01,	7.92,	7.82,	7.72,	7.63,	7.53,	7.43,	7.33,	7.24,	7.14,	7.04,	6.95,	6.85,	6.75,	6.65,	6.56,	6.46,	6.36,	6.27,	6.17,	6.07,	5.98,	5.88,	5.78,	5.69,	5.59,	5.49,	5.40,	5.30,	5.20,	5.11,	5.01,	4.91,	4.82,	4.72,	4.62,	4.53,	4.43,	4.33,	4.24,	4.14,	4.04,	3.95,	3.85,	3.75,	3.66,	3.56,	3.46,	3.37,	3.27,	3.18,	3.08,	2.98,	2.89,	2.79,	2.69,	2.60,	2.50,	2.41,	2.31,	2.21,	2.12,	2.02,	1.92,	1.83,	1.73,	1.64,	1.54,	1.44,	1.35,	1.25,	1.15,	1.06,	0.96,	0.87,	0.77,	0.67,	0.58,	0.48,	0.38,	0.29,	0.19,	0.10,	0.00};
//	float LineUptheta2_LR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.19,	0.39,	0.59,	0.79,	0.99,	1.19,	1.40,	1.60,	1.81,	2.02,	2.23,	2.44,	2.66,	2.87,	3.08,	3.30,	3.51,	3.73,	3.94,	4.16,	4.38,	4.59,	4.81,	5.02,	5.24,	5.46,	5.67,	5.89,	6.10,	6.32,	6.53,	6.74,	6.96,	7.17,	7.38,	7.59,	7.81,	8.02,	8.23,	8.44,	8.64,	8.85,	9.06,	9.27,	9.47,	9.68,	9.88,	10.08,	10.29,	10.49,	10.69,	10.89,	11.09,	11.29,	11.49,	11.69,	11.89,	12.09,	12.28,	12.48,	12.67,	12.87,	13.06,	13.26,	13.45,	13.64,	13.84,	14.03,	14.22,	14.41,	14.60,	14.80,	14.99,	15.18,	15.37,	15.56,	15.75,	15.93,	16.12,	16.31,	16.50,	16.69,	16.88,	17.07,	17.26,	17.45,	17.63,	17.82,	18.01,	18.20,	18.39,	18.58,	18.77,	18.96,	19.14,	19.33,	19.52,	19.71,	19.90,	20.09,	20.29,	20.48,	20.67,	20.86,	21.05,	21.24,	21.44,	21.63,	21.82,	22.02,	22.21,	22.40,	22.60,	22.79,	22.99,	23.19,	23.38,	23.58,	23.78,	23.98,	24.18,	24.38,	24.58,	24.78,	24.98,	25.18,	25.38,	25.59,	25.79,	25.99,	26.20,	26.41,	26.61,	26.82,	27.03,	27.24,	27.45,	27.66,	27.87,	28.08,	28.29,	28.50,	28.72,	28.93,	29.15,	29.37,	29.58,	29.80,	30.02,	30.02,	30.04,	30.04,	30.03,	30.02,	30.02,	30.10,	30.29,	30.67,	31.37,	31.37,	31.24,	31.10,	30.97,	30.83,	30.70,	30.57,	30.43,	30.30,	30.17,	30.04,	29.91,	29.78,	29.65,	29.52,	29.39,	29.26,	29.14,	29.01,	28.88,	28.76,	28.63,	28.51,	28.38,	28.26,	28.13,	28.01,	27.88,	27.76,	27.64,	27.52,	27.39,	27.27,	27.15,	27.03,	26.91,	26.79,	26.67,	26.55,	26.43,	26.31,	26.19,	26.07,	25.95,	25.84,	25.72,	25.60,	25.49,	25.37,	25.25,	25.14,	25.02,	24.90,	24.79,	24.67,	24.56,	24.44,	24.33,	24.22,	24.10,	23.99,	23.87,	23.76,	23.65,	23.54,	23.42,	23.31,	23.20,	23.09,	22.98,	22.86,	22.75,	22.64,	22.53,	22.42,	22.31,	22.20,	22.09,	21.98,	21.87,	21.76,	21.65,	21.54,	21.43,	21.32,	21.22,	21.11,	21.00,	20.89,	20.78,	20.67,	20.57,	20.46,	20.35,	20.24,	20.14,	20.03,	19.92,	19.82,	19.71,	19.60,	19.50,	19.39,	19.29,	19.18,	19.08,	18.97,	18.86,	18.76,	18.65,	18.55,	18.44,	18.34,	18.24,	18.13,	18.03,	17.92,	17.82,	17.71,	17.61,	17.51,	17.40,	17.30,	17.20,	17.09,	16.99,	16.89,	16.78,	16.68,	16.58,	16.48,	16.37,	16.27,	16.17,	16.07,	15.96,	15.86,	15.76,	15.66,	15.56,	15.45,	15.35,	15.25,	15.15,	15.05,	14.95,	14.84,	14.74,	14.64,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.54,	14.44,	14.34,	14.24,	14.14,	14.04,	13.94,	13.84,	13.74,	13.64,	13.54,	13.44,	13.34,	13.24,	13.14,	13.04,	12.94,	12.84,	12.74,	12.64,	12.54,	12.44,	12.34,	12.24,	12.14,	12.04,	11.94,	11.84,	11.74,	11.64,	11.55,	11.45,	11.35,	11.25,	11.15,	11.05,	10.95,	10.85,	10.76,	10.66,	10.56,	10.46,	10.36,	10.26,	10.17,	10.07,	9.97,	9.87,	9.77,	9.67,	9.58,	9.48,	9.38,	9.28,	9.19,	9.09,	8.99,	8.89,	8.79,	8.70,	8.60,	8.50,	8.40,	8.31,	8.21,	8.11,	8.01,	7.92,	7.82,	7.72,	7.63,	7.53,	7.43,	7.33,	7.24,	7.14,	7.04,	6.95,	6.85,	6.75,	6.65,	6.56,	6.46,	6.36,	6.27,	6.17,	6.07,	5.98,	5.88,	5.78,	5.69,	5.59,	5.49,	5.40,	5.30,	5.20,	5.11,	5.01,	4.91,	4.82,	4.72,	4.62,	4.53,	4.43,	4.33,	4.24,	4.14,	4.04,	3.95,	3.85,	3.75,	3.66,	3.56,	3.46,	3.37,	3.27,	3.18,	3.08,	2.98,	2.89,	2.79,	2.69,	2.60,	2.50,	2.41,	2.31,	2.21,	2.12,	2.02,	1.92,	1.83,	1.73,	1.64,	1.54,	1.44,	1.35,	1.25,	1.15,	1.06,	0.96,	0.87,	0.77,	0.67,	0.58,	0.48,	0.38,	0.29,	0.19,	0.10,	0.00};
//	
//	float LineUptheta3_RF[LTOTALSTEPS] = {0.00,	-0.51,	-1.02,	-1.53,	-2.04,	-2.55,	-3.06,	-3.56,	-4.06,	-4.56,	-5.06,	-5.56,	-6.06,	-6.55,	-7.04,	-7.53,	-8.02,	-8.50,	-8.98,	-9.46,	-9.94,	-10.41,	-10.88,	-11.35,	-11.81,	-12.27,	-12.73,	-13.18,	-13.64,	-14.08,	-14.53,	-14.97,	-15.41,	-15.84,	-16.27,	-16.70,	-17.12,	-17.54,	-17.96,	-18.37,	-18.78,	-19.18,	-19.58,	-19.98,	-20.37,	-20.76,	-21.14,	-21.52,	-21.90,	-22.27,	-22.64,	-23.00,	-23.36,	-23.72,	-24.07,	-24.41,	-24.76,	-25.09,	-25.43,	-25.76,	-26.08,	-26.40,	-26.72,	-27.03,	-27.34,	-27.64,	-27.94,	-28.24,	-28.53,	-28.82,	-29.10,	-29.38,	-29.65,	-29.92,	-30.19,	-30.45,	-30.70,	-30.96,	-31.21,	-31.45,	-31.69,	-31.93,	-32.16,	-32.39,	-32.62,	-32.84,	-33.05,	-33.27,	-33.48,	-33.68,	-33.88,	-34.08,	-34.27,	-34.46,	-34.65,	-34.83,	-35.01,	-35.18,	-35.36,	-35.52,	-35.69,	-35.85,	-36.00,	-36.15,	-36.30,	-36.45,	-36.59,	-36.73,	-36.86,	-36.99,	-37.12,	-37.25,	-37.37,	-37.48,	-37.60,	-37.71,	-37.82,	-37.92,	-38.02,	-38.12,	-38.21,	-38.30,	-38.39,	-38.47,	-38.55,	-38.63,	-38.70,	-38.77,	-38.84,	-38.91,	-38.97,	-39.02,	-39.08,	-39.13,	-39.18,	-39.22,	-39.27,	-39.31,	-39.34,	-39.37,	-39.40,	-39.43,	-39.45,	-39.47,	-39.49,	-39.50,	-39.52,	-39.52,	-39.53,	-39.53,	-39.53,	-35.58,	-31.39,	-27.02,	-22.52,	-17.94,	-13.34,	-8.79,	-4.33,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00};
//	float LineUptheta3_LF[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	-0.51,	-1.02,	-1.53,	-2.04,	-2.55,	-3.06,	-3.56,	-4.06,	-4.56,	-5.06,	-5.56,	-6.06,	-6.55,	-7.04,	-7.53,	-8.02,	-8.50,	-8.98,	-9.46,	-9.94,	-10.41,	-10.88,	-11.35,	-11.81,	-12.27,	-12.73,	-13.18,	-13.64,	-14.08,	-14.53,	-14.97,	-15.41,	-15.84,	-16.27,	-16.70,	-17.12,	-17.54,	-17.96,	-18.37,	-18.78,	-19.18,	-19.58,	-19.98,	-20.37,	-20.76,	-21.14,	-21.52,	-21.90,	-22.27,	-22.64,	-23.00,	-23.36,	-23.72,	-24.07,	-24.41,	-24.76,	-25.09,	-25.43,	-25.76,	-26.08,	-26.40,	-26.72,	-27.03,	-27.34,	-27.64,	-27.94,	-28.24,	-28.53,	-28.82,	-29.10,	-29.38,	-29.65,	-29.92,	-30.19,	-30.45,	-30.70,	-30.96,	-31.21,	-31.45,	-31.69,	-31.93,	-32.16,	-32.39,	-32.62,	-32.84,	-33.05,	-33.27,	-33.48,	-33.68,	-33.88,	-34.08,	-34.27,	-34.46,	-34.65,	-34.83,	-35.01,	-35.18,	-35.36,	-35.52,	-35.69,	-35.85,	-36.00,	-36.15,	-36.30,	-36.45,	-36.59,	-36.73,	-36.86,	-36.99,	-37.12,	-37.25,	-37.37,	-37.48,	-37.60,	-37.71,	-37.82,	-37.92,	-38.02,	-38.12,	-38.21,	-38.30,	-38.39,	-38.47,	-38.55,	-38.63,	-38.70,	-38.77,	-38.84,	-38.91,	-38.97,	-39.02,	-39.08,	-39.13,	-39.18,	-39.22,	-39.27,	-39.31,	-39.34,	-39.37,	-39.40,	-39.43,	-39.45,	-39.47,	-39.49,	-39.50,	-39.52,	-39.52,	-39.53,	-39.53,	-39.53,	-35.58,	-31.39,	-27.02,	-22.52,	-17.94,	-13.34,	-8.79,	-4.33,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00};
//	float LineUptheta3_RR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	-0.51,	-1.02,	-1.53,	-2.04,	-2.55,	-3.06,	-3.56,	-4.06,	-4.56,	-5.06,	-5.56,	-6.06,	-6.55,	-7.04,	-7.53,	-8.02,	-8.50,	-8.98,	-9.46,	-9.94,	-10.41,	-10.88,	-11.35,	-11.81,	-12.27,	-12.73,	-13.18,	-13.64,	-14.08,	-14.53,	-14.97,	-15.41,	-15.84,	-16.27,	-16.70,	-17.12,	-17.54,	-17.96,	-18.37,	-18.78,	-19.18,	-19.58,	-19.98,	-20.37,	-20.76,	-21.14,	-21.52,	-21.90,	-22.27,	-22.64,	-23.00,	-23.36,	-23.72,	-24.07,	-24.41,	-24.76,	-25.09,	-25.43,	-25.76,	-26.08,	-26.40,	-26.72,	-27.03,	-27.34,	-27.64,	-27.94,	-28.24,	-28.53,	-28.82,	-29.10,	-29.38,	-29.65,	-29.92,	-30.19,	-30.45,	-30.70,	-30.96,	-31.21,	-31.45,	-31.69,	-31.93,	-32.16,	-32.39,	-32.62,	-32.84,	-33.05,	-33.27,	-33.48,	-33.68,	-33.88,	-34.08,	-34.27,	-34.46,	-34.65,	-34.83,	-35.01,	-35.18,	-35.36,	-35.52,	-35.69,	-35.85,	-36.00,	-36.15,	-36.30,	-36.45,	-36.59,	-36.73,	-36.86,	-36.99,	-37.12,	-37.25,	-37.37,	-37.48,	-37.60,	-37.71,	-37.82,	-37.92,	-38.02,	-38.12,	-38.21,	-38.30,	-38.39,	-38.47,	-38.55,	-38.63,	-38.70,	-38.77,	-38.84,	-38.91,	-38.97,	-39.02,	-39.08,	-39.13,	-39.18,	-39.22,	-39.27,	-39.31,	-39.34,	-39.37,	-39.40,	-39.43,	-39.45,	-39.47,	-39.49,	-39.50,	-39.52,	-39.52,	-39.53,	-39.53,	-39.53,	-35.58,	-31.39,	-27.02,	-22.52,	-17.94,	-13.34,	-8.79,	-4.33,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00};
//	float LineUptheta3_LR[LTOTALSTEPS] = {0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	-0.51,	-1.02,	-1.53,	-2.04,	-2.55,	-3.06,	-3.56,	-4.06,	-4.56,	-5.06,	-5.56,	-6.06,	-6.55,	-7.04,	-7.53,	-8.02,	-8.50,	-8.98,	-9.46,	-9.94,	-10.41,	-10.88,	-11.35,	-11.81,	-12.27,	-12.73,	-13.18,	-13.64,	-14.08,	-14.53,	-14.97,	-15.41,	-15.84,	-16.27,	-16.70,	-17.12,	-17.54,	-17.96,	-18.37,	-18.78,	-19.18,	-19.58,	-19.98,	-20.37,	-20.76,	-21.14,	-21.52,	-21.90,	-22.27,	-22.64,	-23.00,	-23.36,	-23.72,	-24.07,	-24.41,	-24.76,	-25.09,	-25.43,	-25.76,	-26.08,	-26.40,	-26.72,	-27.03,	-27.34,	-27.64,	-27.94,	-28.24,	-28.53,	-28.82,	-29.10,	-29.38,	-29.65,	-29.92,	-30.19,	-30.45,	-30.70,	-30.96,	-31.21,	-31.45,	-31.69,	-31.93,	-32.16,	-32.39,	-32.62,	-32.84,	-33.05,	-33.27,	-33.48,	-33.68,	-33.88,	-34.08,	-34.27,	-34.46,	-34.65,	-34.83,	-35.01,	-35.18,	-35.36,	-35.52,	-35.69,	-35.85,	-36.00,	-36.15,	-36.30,	-36.45,	-36.59,	-36.73,	-36.86,	-36.99,	-37.12,	-37.25,	-37.37,	-37.48,	-37.60,	-37.71,	-37.82,	-37.92,	-38.02,	-38.12,	-38.21,	-38.30,	-38.39,	-38.47,	-38.55,	-38.63,	-38.70,	-38.77,	-38.84,	-38.91,	-38.97,	-39.02,	-39.08,	-39.13,	-39.18,	-39.22,	-39.27,	-39.31,	-39.34,	-39.37,	-39.40,	-39.43,	-39.45,	-39.47,	-39.49,	-39.50,	-39.52,	-39.52,	-39.53,	-39.53,	-39.53,	-35.58,	-31.39,	-27.02,	-22.52,	-17.94,	-13.34,	-8.79,	-4.33,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00,	0.00};	

////向下部分步态数据
//	float LineDowntheta1_RF[LTOTALSTEPS] = {};
//	float LineDowntheta1_LF[LTOTALSTEPS] = {};
//	float LineDowntheta1_RR[LTOTALSTEPS] = {};
//	float LineDowntheta1_LR[LTOTALSTEPS] = {};	
//	
//	float LineDowntheta2_RF[LTOTALSTEPS] = {};
//	float LineDowntheta2_LF[LTOTALSTEPS] = {};
//	float LineDowntheta2_RR[LTOTALSTEPS] = {};
//	float LineDowntheta2_LR[LTOTALSTEPS] = {};
//	
//	float LineDowntheta3_RF[LTOTALSTEPS] = {};
//	float LineDowntheta3_LF[LTOTALSTEPS] = {};
//	float LineDowntheta3_RR[LTOTALSTEPS] = {};
//	float LineDowntheta3_LR[LTOTALSTEPS] = {};

////左转部分步态数据		
//	float TurnLefttheta1_RF[TTOTALSTEPS] = {};
//	float TurnLefttheta1_LF[TTOTALSTEPS] = {};
//	float TurnLefttheta1_RR[TTOTALSTEPS] = {};
//	float TurnLefttheta1_LR[TTOTALSTEPS] = {};	
//	
//	float TurnLefttheta2_RF[TTOTALSTEPS] = {};
//	float TurnLefttheta2_LF[TTOTALSTEPS] = {};
//	float TurnLefttheta2_RR[TTOTALSTEPS] = {};
//	float TurnLefttheta2_LR[TTOTALSTEPS] = {};
//	
//	float TurnLefttheta3_RF[TTOTALSTEPS] = {};
//	float TurnLefttheta3_LF[TTOTALSTEPS] = {};
//	float TurnLefttheta3_RR[TTOTALSTEPS] = {};
//	float TurnLefttheta3_LR[TTOTALSTEPS] = {};
//		
////右转部分步态数据		
//	float TurnRighttheta1_RF[TTOTALSTEPS] = {};
//	float TurnRighttheta1_LF[TTOTALSTEPS] = {};
//	float TurnRighttheta1_RR[TTOTALSTEPS] = {};
//	float TurnRighttheta1_LR[TTOTALSTEPS] = {};	
//	
//	float TurnRighttheta2_RF[TTOTALSTEPS] = {};
//	float TurnRighttheta2_LF[TTOTALSTEPS] = {};
//	float TurnRighttheta2_RR[TTOTALSTEPS] = {};
//	float TurnRighttheta2_LR[TTOTALSTEPS] = {};
//	
//	float TurnRighttheta3_RF[TTOTALSTEPS] = {};
//	float TurnRighttheta3_LF[TTOTALSTEPS] = {};
//	float TurnRighttheta3_RR[TTOTALSTEPS] = {};
//	float TurnRighttheta3_LR[TTOTALSTEPS] = {};

///********************************************************************************************************
//*date：2018-10-10
//*function:RunLIneUp
//*author：NUAA google
////********************************************************************************************************/

//void RunLineUp(void)
//{
//	uint32_t j = 0;
//			for(j = 0; j<(LTOTALSTEPS); j++)	
//		{
//			Angle((int32_t)(LineUptheta3_LF[j]+KMGecko.StartAngle[LF_J3]),LF_J3);
//			Angle((int32_t)(-(LineUptheta1_LF[j]+KMGecko.StartAngle[LF_J1])),LF_J1);//正负关系，注意
//			Angle((int32_t)(LineUptheta2_LF[j]+KMGecko.StartAngle[LF_J2]),LF_J2);
//			
//			Angle((int32_t)(LineUptheta1_RF[j]+KMGecko.StartAngle[RF_J1]),RF_J1);
//			Angle((int32_t)(-(LineUptheta2_RF[j]+KMGecko.StartAngle[RF_J2])),RF_J2);		
//			Angle((int32_t)(-(LineUptheta3_RF[j]+KMGecko.StartAngle[RF_J3])),RF_J3);
//			
//			Angle((int32_t)(LineUptheta1_RR[j]+KMGecko.StartAngle[RR_J1]),RR_J1);
//			Angle((int32_t)(-(LineUptheta2_RR[j]+KMGecko.StartAngle[RR_J2])),RR_J2);
//			Angle((int32_t)(LineUptheta2_RR[j]+KMGecko.StartAngle[RR_J3]),RR_J3);

//			Angle((int32_t)(-LineUptheta1_LR[j]+KMGecko.StartAngle[LR_J1]),LR_J1);
//			Angle((int32_t)(LineUptheta2_LR[j]+KMGecko.StartAngle[LR_J2]),LR_J2);
//			Angle((int32_t)(-(LineUptheta3_LR[j]+KMGecko.StartAngle[LR_J3])),LR_J3);
//			HAL_Delay(15);
//		}
//}

/////********************************************************************************************************
////*date：2018-10-10
////*function:RUN line down
////*author：NUAA google
////********************************************************************************************************/

//void RunLineDown(void)
//{
//	uint32_t j = 0;
//			for(j = 0; j<(LTOTALSTEPS); j++)	
//		{
//			Angle((int32_t)(LineDowntheta3_LF[j]+KMGecko.StartAngle[LF_J3]),LF_J3);
//			Angle((int32_t)(-(LineDowntheta1_LF[j]+KMGecko.StartAngle[LF_J1])),LF_J1);//正负关系，注意
//			Angle((int32_t)(LineDowntheta2_LF[j]+KMGecko.StartAngle[LF_J2]),LF_J2);
//			
//			Angle((int32_t)(LineDowntheta1_RF[j]+KMGecko.StartAngle[RF_J1]),RF_J1);
//			Angle((int32_t)(-(LineDowntheta2_RF[j]+KMGecko.StartAngle[RF_J2])),RF_J2);		
//			Angle((int32_t)(-(LineDowntheta3_RF[j]+KMGecko.StartAngle[RF_J3])),RF_J3);
//			
//			Angle((int32_t)(LineDowntheta1_RR[j]+KMGecko.StartAngle[RR_J1]),RR_J1);
//			Angle((int32_t)(-(LineDowntheta2_RR[j]+KMGecko.StartAngle[RR_J2])),RR_J2);
//			Angle((int32_t)(LineDowntheta2_RR[j]+KMGecko.StartAngle[RR_J3]),RR_J3);

//			Angle((int32_t)(-LineDowntheta1_LR[j]+KMGecko.StartAngle[LR_J1]),LR_J1);
//			Angle((int32_t)(LineDowntheta2_LR[j]+KMGecko.StartAngle[LR_J2]),LR_J2);
//			Angle((int32_t)(-(LineDowntheta3_LR[j]+KMGecko.StartAngle[LR_J3])),LR_J3);
//			HAL_Delay(15);
//		}
//}

/////********************************************************************************************************
////*date：2018-10-10
////*function:Turn Left
////*author：NUAA google
////********************************************************************************************************/
//void TurnLeft(void)
//{
//	uint32_t j = 0;
//			for(j = 0; j<(TTOTALSTEPS); j++)	
//		{
//			Angle((int32_t)(TurnLefttheta3_LF[j]+KMGecko.StartAngle[LF_J3]),LF_J3);
//			Angle((int32_t)(-(TurnLefttheta1_LF[j]+KMGecko.StartAngle[LF_J1])),LF_J1);//正负关系，注意
//			Angle((int32_t)(TurnLefttheta2_LF[j]+KMGecko.StartAngle[LF_J2]),LF_J2);
//			
//			Angle((int32_t)(TurnLefttheta1_RF[j]+KMGecko.StartAngle[RF_J1]),RF_J1);
//			Angle((int32_t)(-(TurnLefttheta2_RF[j]+KMGecko.StartAngle[RF_J2])),RF_J2);		
//			Angle((int32_t)(-(TurnLefttheta3_RF[j]+KMGecko.StartAngle[RF_J3])),RF_J3);
//			
//			Angle((int32_t)(TurnLefttheta1_RR[j]+KMGecko.StartAngle[RR_J1]),RR_J1);
//			Angle((int32_t)(-(TurnLefttheta2_RR[j]+KMGecko.StartAngle[RR_J2])),RR_J2);
//			Angle((int32_t)(TurnLefttheta2_RR[j]+KMGecko.StartAngle[RR_J3]),RR_J3);

//			Angle((int32_t)(-(TurnLefttheta1_LR[j]+KMGecko.StartAngle[LR_J1])),LR_J1);
//			Angle((int32_t)(TurnLefttheta2_LR[j]+KMGecko.StartAngle[LR_J2]),LR_J2);
//			Angle((int32_t)(-(TurnLefttheta3_LR[j]+KMGecko.StartAngle[LR_J3])),LR_J3);
//			HAL_Delay(15);
//		}
//}

