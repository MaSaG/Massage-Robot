#ifndef KINEMATICS
#define KINEMATICS

#include "SystemParameter.h"


class kinematics
{
	public:
		boost::shared_ptr< rl::kin::Kinematics > kin;
		rl::math::Matrix JacobianL;
		rl::math::Matrix JacobianR;

		kinematics()
		{
			
			boost::shared_ptr< rl::kin::Kinematics > kinematics_L(rl::kin::Kinematics::create("massage.xml"));
			kin = kinematics_L;
			
		}

		
		void ForwardKinematics(Eigen::Vector3d &pos,Eigen::Vector3d &ori,rl::math::Transform &T,float jointAngle[6])  //pos從TCP傳過來
		{
			rl::math::Vector q(kin->getDof());
			for ( int i = 0; i < kin->getDof(); i++) 
			{
				q(i) = jointAngle[i];
			}
			kin->setPosition(q);
			kin->updateFrames();
			rl::math::Transform T0 = kin->forwardPosition();
			T = T0;
			pos = T0.translation();
			ori = T0.rotation().eulerAngles(0, 1, 2);
			//ori(0)=ori(0)*180/PI;
			//ori(1)=ori(1)*180/PI;
			//ori(2)=ori(2)*180/PI;

		}

		void updateJacobian(float jointAngle[6],int Card_Index)  
		{
			rl::math::Vector q(kin->getDof());
			for ( int i = 0; i < kin->getDof(); i++) 
			{
				q(i) = jointAngle[i];
			}
			float test[6]={q(0),q(1),q(2),q(3),q(4),q(5)};
			kin->setPosition(q);
			kin->updateFrames();
			kin->updateJacobian();
			if (Card_Index==ARM_L) JacobianL = kin->getJacobian();
			if (Card_Index==ARM_R) JacobianR = kin->getJacobian();
		}

        //void InversKinematics();  
		
};


#endif