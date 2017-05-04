#ifndef JOINT_SPACE
#define JOINT_SPACE

#include "SystemParameter.h"

class jointSpace
{
public:
	
	float angle[6]; 
	float lastAngle[6];
	float lastAngle2[6];
	float velocity[6];
	//jointSpace(){};
	jointSpace(float joint0, float joint1, float joint2, float joint3, float joint4, float joint5)
	{
		angle[0]=joint0;
		angle[1]=joint1;
		angle[2]=joint2;
		angle[3]=joint3;
		angle[4]=joint4;
		angle[5]=joint5;
		lastAngle[0]=joint0;
		lastAngle[1]=joint1;
		lastAngle[2]=joint2;
		lastAngle[3]=joint3;
		lastAngle[4]=joint4;
		lastAngle[5]=joint5;
		lastAngle2[0]=joint0;
		lastAngle2[1]=joint1;
		lastAngle2[2]=joint2;
		lastAngle2[3]=joint3;
		lastAngle2[4]=joint4;
		lastAngle2[5]=joint5;
	}



	// 輸入欲設定之值的成員函數
	void updataJointAngle(long encoderValue[6], const float ENC2RAD_FACTOR[6]) 
	{
		for (int i=0; i<6 ; i++)
		{
			angle[i] = encoderValue[i] * ENC2RAD_FACTOR[i];
		}
	}

	void updateLastJointAngle()
	{
		for (int i=0; i<6 ; i++)
		{
			lastAngle2[i] = lastAngle[i];
			lastAngle[i] = angle[i];
		}

	}
};


#endif