#include "tcp6D.h"



tcp6D::tcp6D(Eigen::Vector3d pos_, Eigen::Vector3d abc_)  //丟xyz abr(角度)
	{
		currentPos=pos_;
		currentOri = Eigen::Vector3d(abc_(0)*rl::math::DEG2RAD,abc_(1)*rl::math::DEG2RAD,abc_(2)*rl::math::DEG2RAD);
		
		Eigen::Matrix3d temp;
		temp = Eigen::AngleAxisd( abc_(0)*rl::math::DEG2RAD, Eigen::Vector3d::UnitZ())  
			   *Eigen::AngleAxisd( abc_(1)*rl::math::DEG2RAD, Eigen::Vector3d::UnitY())
			   *Eigen::AngleAxisd( abc_(2)*rl::math::DEG2RAD, Eigen::Vector3d::UnitX());
		currentT.matrix().block(0,0,3,3)=temp;
		currentT.translation()=currentPos;
		
		targetPos=currentPos;
		targetOri=currentOri;
		targetT=currentT;
		
		
		vmax[0]=0.5;
		vmax[1]=0.5;
		vmax[2]=0.5;
		vmax[3]=1;
		vmax[4]=1;
		vmax[5]=1;
		amax[0]=1;
		amax[1]=1;
		amax[2]=1;
		amax[3]=1;
		amax[4]=1;
		amax[5]=1;

		OTG_X_Last.resize(6);
		OTG_V_Last.resize(6);

		OTG_X_Last( 0) = pos_(0);
		OTG_X_Last( 1) = pos_(1);
		OTG_X_Last( 2) = pos_(2);
		OTG_X_Last( 3) = -90*rl::math::DEG2RAD;
		OTG_X_Last( 4) = 0;
		OTG_X_Last( 5) = 180*rl::math::DEG2RAD;

		for( int i = 0; i < 6; i++)
			OTG_V_Last(i) = 0;

		delta_v.resize(6);
	}

void tcp6D::inputTarget( Eigen::Vector3d pos_, Eigen::Vector3d abc_) //丟xyz abr角度
{
	targetPos=pos_;
	targetOri=Eigen::Vector3d(abc_(0)*rl::math::DEG2RAD,abc_(1)*rl::math::DEG2RAD,abc_(2)*rl::math::DEG2RAD);

	Eigen::Matrix3d temp;
	temp = Eigen::AngleAxisd( abc_(0)*rl::math::DEG2RAD, Eigen::Vector3d::UnitZ())  
			*Eigen::AngleAxisd( abc_(1)*rl::math::DEG2RAD, Eigen::Vector3d::UnitY())
			*Eigen::AngleAxisd( abc_(2)*rl::math::DEG2RAD, Eigen::Vector3d::UnitX());
	targetT.matrix().block(0,0,3,3)=temp;

	targetT.translation()=targetPos;
		
}

void tcp6D::inputTarget( Eigen::Matrix3d T0 ) //丟matrix
{
	targetT=T0;

	targetPos = targetT.translation();
	targetOri = targetT.rotation().eulerAngles(0, 1, 2);//.reverse();
	   
}

void tcp6D::OTG (rl::math::Vector &delta_x, rl::math::Matrix Jacobian,float angle[6],float lastAngle[6]) //update delta_x
{

	t1 = Eigen::Vector3d( OTG_X_Last(0), OTG_X_Last(1), OTG_X_Last(2));
	r1 = Eigen::AngleAxisd( OTG_X_Last(3), Eigen::Vector3d::UnitZ())
			*Eigen::AngleAxisd( OTG_X_Last( 4), Eigen::Vector3d::UnitY())
			*Eigen::AngleAxisd( OTG_X_Last( 5), Eigen::Vector3d::UnitX());


	Eigen::VectorXd x(6), v(6);
	x(0) = targetPos(0) - t1(0);
	x(1) = targetPos(1) - t1(1);
	x(2) = targetPos(2) - t1(2);
	x(3) = targetOri(0)*rl::math::DEG2RAD - OTG_X_Last(3); 
	x(4) = targetOri(1)*rl::math::DEG2RAD - OTG_X_Last(4);
	x(5) = targetOri(2)*rl::math::DEG2RAD - OTG_X_Last(5);

	v = x/0.001;

			
	rl::math::Vector TCP_V;
	rl::math::Vector OTG_X(6);
	rl::math::Vector OTG_V(6);			
	rl::math::Vector dq(6);
		for( int i = 0; i < 6; i++)
			dq(i) = (angle[i] - lastAngle[i])/0.001;		
		TCP_V = Jacobian*dq;
	for( int i = 0; i < 6; i++)
	{
			OTG_X(i) = x(i);
			OTG_V(i) = v(i);
			if( fabs(OTG_X(i)) < 0.000001)
				OTG_X(i) = 0;
			double vel_dec  = sqrt( 2*amax[i]*fabs(OTG_X(i)));	// deceleration part of the velocity profile

			if( OTG_V(i) >= 0)
			{
				if( OTG_V(i) > vmax[i])
					OTG_V(i) = vmax[i];
				if( OTG_V(i) > vel_dec)
					OTG_V(i) = vel_dec;
				if( OTG_V(i) > OTG_V_Last(i) + amax[i]*0.001)
					OTG_V(i) = OTG_V_Last(i) + amax[i]*0.001;

			}
			else
			{
				if( OTG_V(i) < -vmax[i])
					OTG_V(i) = -vmax[i];
				if( OTG_V(i) < -vel_dec)
					OTG_V(i) = -vel_dec;
				if( OTG_V(i) < OTG_V_Last(i) - amax[i]*0.001)
					OTG_V(i) = OTG_V_Last(i) - amax[i]*0.001;
			}

			// update
			OTG_X(i) = OTG_X_Last(i) + 0.5*(OTG_V(i)+OTG_V_Last(i))*0.001;
			// record the last cartesian position and velocity
			OTG_X_Last(i) = OTG_X(i);
			OTG_V_Last(i) = OTG_V(i);
		}
		Eigen::Vector3d dx, dth;
		Eigen::Vector3d tcmd( OTG_X(0), OTG_X(1), OTG_X(2));
		// calcuate the target transformation matrix
		Eigen::Matrix3d rcmd;
		rcmd = Eigen::AngleAxisd( OTG_X(3), Eigen::Vector3d::UnitZ())
				*Eigen::AngleAxisd( OTG_X(4), Eigen::Vector3d::UnitY())
				*Eigen::AngleAxisd( OTG_X(5), Eigen::Vector3d::UnitX());

		Eigen::Matrix3d skew = rcmd*currentT.rotation().transpose() - Eigen::Matrix3d::Identity();
		dth = Eigen::Vector3d( 0.5*(skew(2,1)-skew(1,2)), 0.5*(skew(0,2)-skew(2,0)), 0.5*(skew(1,0)-skew(0,1)));
		dx = tcmd - currentT.translation();

		delta_x.resize(6);
		for( int i = 0; i < 3; i++)
		{
			delta_x(i) = dx(i);
			delta_x(i+3) = 0;  //dth沒有放進去，所以本來6*6的J矩陣只有用到左上的4*3
		}

			
		for( int i = 0; i < 6; i++)
			delta_v(i) = OTG_V(i) - TCP_V(i);

}

void tcp6D::updateOTG_Last() //在Free Mode的時候用
{

	OTG_X_Last(0) = currentPos(0);
	OTG_X_Last(1) = currentPos(1);
	OTG_X_Last(2) = currentPos(2);
	OTG_X_Last(3) = currentOri(0);
	OTG_X_Last(4) = currentOri(1);
	OTG_X_Last(5) = currentOri(2);
	for( int i = 0; i < 6; i++)
		OTG_V_Last(i) = 0;
}