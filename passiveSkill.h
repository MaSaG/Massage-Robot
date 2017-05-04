#include "systemParameter.h"



class passiveSkill
{
public:

	bool collisionSW; //�o���ܼƥΨӱ���collision detection���}�ҩM����
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
			if (temp > 0.9)    //�o�ӼƦr�O�@��threshold
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
				double gain=8;  //�n�Ұʪ��ܡA�o�ӭ����ӭn�զ�5~10  ��ĳ8
				//logistic function : 1/(1+exp(���Ƶ{��*(�{�b��m-������m-���u��m)))
				//�Ĥ@�b -40~120
				if (jointAngle[0]*180/PI <40) RepulsiveTorque[0]=gain/(1+exp(3*(jointAngle[0]*180/PI-(-40))-2 )); 
				else RepulsiveTorque[0]=gain*(-1+1/(1+exp(3*(jointAngle[0]*180/PI-(120))-2 )));
				//�ĤG�b -3~90
				if (jointAngle[1]*180/PI <45) RepulsiveTorque[1]=gain/(1+exp(3*(jointAngle[1]*180/PI-(-3))-2 )); 
				else RepulsiveTorque[1]=gain*(-1+1/(1+exp(3*(jointAngle[1]*180/PI-(90))-2 )));
				//�ĤT�b -90~+90
				if (jointAngle[2]*180/PI <0) RepulsiveTorque[2]=gain/(1+exp(3*(jointAngle[2]*180/PI-(-90))-2 )); 
				else RepulsiveTorque[2]=gain*(-1+1/(1+exp(3*(jointAngle[2]*180/PI-(90))-2 )));
				//�ĥ|�b -95~20
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