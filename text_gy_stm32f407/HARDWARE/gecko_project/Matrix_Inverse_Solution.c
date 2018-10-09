#include "Matrix_Inverse_solution.h"
#include "math.h"
/*-----------------------------------------------------------------------*
*Input:x0,y0,z0
*output:theta1,theta2,theta3
*------------------------------------------------------------------------*/
KinematicsArm KMGecko;

void Inverse_Kinematic(void)
{
	float a1 = 1.0;//link lengthen
	float a2 = 1.0;//link lengthen
	float a3 = 1.0;//link lengthen
	float a4 = 1.0;//link lengthen
	float a5 = 0.0;//link lengthen
	
	float l1 = 65;//link lengthen
	float l2 = 32;//link lengthen
	float l3 = 32;//link lengthen
	
	float x0 = 65;//x value of end-effect in basic frame
	float y0 = 32;//y value of end-effect in basic frame
	float z0 = -32;//z value of end-effect in basic frame
	
	float theta1 = 0.0;//joint 1
	float theta2 = 0.0;//joint 2
	float theta3 = 0.0;//joint 3
	
	float r =0.0;//Graphical Method variable
	float r1 =0.0;//Graphical Method variable
	float phi1 = 0.0;//Graphical Method variable
	float phi2 = 0.0;//Graphical Method variable
	float phi3 = 0.0;//Graphical Method variable
	float phi4 = 0.0;//Graphical Method variable
	float phi5 = 0.0;//Graphical Method variable
	float phi6 = 0.0;//Graphical Method variable
	
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
/**-----------½Ç¶È×ª»»---------**/
	KMGecko.theta1[0] = (int)((theta1/3.1415)*180);
	KMGecko.theta2[0] = (int)((theta2/3.1415)*180);
	KMGecko.theta3[0] = (int)((theta3/3.1415)*180);
}
