#ifndef TCP6D
#define TCP6D

#include "SystemParameter.h"


class tcp6D
{

private:
	// last position and angular position
	Eigen::VectorXf OTG_X_Last; 
	// last velocity and angular velocity
	Eigen::VectorXf OTG_V_Last; 
	Eigen::Vector3d t1;
	Eigen::Matrix3d r1;

public:

	Eigen::Vector3d currentPos;
	Eigen::Vector3d currentOri;  //�H��
	Eigen::Vector3d targetPos;
	Eigen::Vector3d targetOri;  //�H��
	rl::math::Transform currentT;
	rl::math::Transform targetT;
	
	float vmax[6],amax[6];

	rl::math::Vector delta_v; //for collision detection

	
	tcp6D(Eigen::Vector3d pos_, Eigen::Vector3d abc_);  //��xyz abr(����)


	void inputTarget( Eigen::Vector3d pos_, Eigen::Vector3d abc_); //��xyz abr����


	void inputTarget( Eigen::Matrix3d T0 ); //��matrix


	void OTG (rl::math::Vector &delta_x, rl::math::Matrix Jacobian,float angle[6],float lastAngle[6]); //update delta_x


	void updateOTG_Last(); //�bFree Mode���ɭԥ�


};


#endif