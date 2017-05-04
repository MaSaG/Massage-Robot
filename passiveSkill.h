#include "systemParameter.h"



class passiveSkill
{
public:

	bool collisionSW; //這個變數用來控制collision detection的開啟和關閉
	bool jointLimitation;

	passiveSkill()
	{
		collisionSW=false;
		jointLimitation=false;
	}

	void collisionDetection(rl::math::Vector delta_v, int &Control_Mode)
	{
		if (collisionSW==true)
		{
			float temp=sqrt(pow(delta_v(0),2)+pow(delta_v(1),2)+pow(delta_v(2),2));
			if (temp > 0.9)    //這個數字是一個threshold
			{
				collisionSW=false;
				Control_Mode=Free_Mode;
			}
		}
	}

	void getRepulsiveTorque(float RepulsiveTorque[6],float jointAngle[6], int ArmIndex)
	{
		if (jointLimitation==true)
		{
			if (ArmIndex == 0)
			{
				double gain=8;  //要啟動的話，這個值應該要調成5~10  建議8
				//logistic function : 1/(1+exp(平滑程度*(現在位置-極限位置-曲線位置)))
				//第一軸 -40~120
				if (jointAngle[0]*180/PI <40) RepulsiveTorque[0]=gain/(1+exp(3*(jointAngle[0]*180/PI-(-40))-2 )); 
				else RepulsiveTorque[0]=gain*(-1+1/(1+exp(3*(jointAngle[0]*180/PI-(120))-2 )));
				//第二軸 -3~90
				if (jointAngle[1]*180/PI <45) RepulsiveTorque[1]=gain/(1+exp(3*(jointAngle[1]*180/PI-(-3))-2 )); 
				else RepulsiveTorque[1]=gain*(-1+1/(1+exp(3*(jointAngle[1]*180/PI-(90))-2 )));
				//第三軸 -90~+90
				if (jointAngle[2]*180/PI <0) RepulsiveTorque[2]=gain/(1+exp(3*(jointAngle[2]*180/PI-(-90))-2 )); 
				else RepulsiveTorque[2]=gain*(-1+1/(1+exp(3*(jointAngle[2]*180/PI-(90))-2 )));
				//第四軸 -95~20
				if (jointAngle[3]*180/PI <-45) RepulsiveTorque[3]=gain/(1+exp(3*(jointAngle[3]*180/PI-(-95))-2 )); 
				else RepulsiveTorque[3]=gain*(-1+1/(1+exp(3*(jointAngle[3]*180/PI-(20))-2 )));
			
				RepulsiveTorque[4]=0;
				RepulsiveTorque[5]=0;
			}

			if (ArmIndex == 1)
			{
				RepulsiveTorque[0]=0;
				RepulsiveTorque[1]=0;
				RepulsiveTorque[2]=0;
				RepulsiveTorque[3]=0;
				RepulsiveTorque[4]=0;
				RepulsiveTorque[5]=0;
			}
		}
		else
		{
			RepulsiveTorque[0]=0;
			RepulsiveTorque[1]=0;
			RepulsiveTorque[2]=0;
			RepulsiveTorque[3]=0;
			RepulsiveTorque[4]=0;
			RepulsiveTorque[5]=0;
		}

	}



};