#include "control.h"



control::control(tcp6D &tcpLeft, tcp6D &tcpRight, jointSpace &jointspaceLeft, jointSpace &jointspaceRight, kinematics &kinmatic)
{
	tcpL = &tcpLeft;
	tcpR = &tcpRight;
	jointL = &jointspaceLeft;
	jointR = &jointspaceRight;
	kin = &kinmatic;
	timeIndex=0;
	bTeachFlag=false;
	bPlayFlag=false;
	jKey=' ';
	jointPlayDone=false;
}

void control::CartesianControl(rl::math::Vector &torqueL , rl::math::Vector &torqueR)
{
	kin->updateJacobian(jointL->angle,ARM_L); //更新jacobian
	kin->updateJacobian(jointR->angle,ARM_R);

	rl::math::Vector delta_x_L(6),delta_x_R(6);
	tcpL->OTG(delta_x_L, kin->JacobianL, jointL->angle, jointL->lastAngle); //OTG計算delta_x
	tcpR->OTG(delta_x_R, kin->JacobianR, jointR->angle, jointR->lastAngle);

	float k_L[6]={200,200,200,100,100,100}; //cartesian space gain
	float k_R[6]={200,200,200,100,100,100}; //cartesian space gain
	float kx_L[6]={0.7,1.5,6.0,0.7,0,0};  //joint space gain
	float kx_R[6]={1.5,1.5,1.5,1.5,0,0};  //joint space gain

	for( int i = 0; i < 6; i++)//delta_x乘上gain變成force
	{
		torqueL(i) = k_L[i]*delta_x_L(i);
		torqueR(i) = k_R[i]*delta_x_R(i);
	}
	torqueL = kin->JacobianL.transpose()*(torqueL);  //jacobian乘上force變成torque
	torqueR = kin->JacobianR.transpose()*(torqueR);

	for( int i = 0; i < 6; i++)  //torque再乘一個gain
	{
		torqueL(i) = kx_L[i]*torqueL(i);
		torqueR(i) = kx_R[i]*torqueR(i);
	}
}
void control::CartesianControl2(rl::math::Vector &torqueL , rl::math::Vector &torqueR)
{
	kin->updateJacobian(jointL->angle,ARM_L); //更新jacobian
	kin->updateJacobian(jointR->angle,ARM_R);

	rl::math::Vector delta_x_L(6),delta_x_R(6);
	tcpL->OTG(delta_x_L, kin->JacobianL, jointL->angle, jointL->lastAngle); //OTG計算delta_x
	tcpR->OTG(delta_x_R, kin->JacobianR, jointR->angle, jointR->lastAngle);

	float k_L[6]={200,200,200,100,100,100}; //cartesian space gain
	float k_R[6]={200,200,200,100,100,100}; //cartesian space gain
	float kx_L[6]={0.7,1.5,6.0,0.7,0,0};  //joint space gain
	float kx_R[6]={1.5,1.5,1.5,1.5,0,0};  //joint space gain

	for( int i = 0; i < 6; i++)//delta_x乘上gain變成force
	{
		torqueL(i) = k_L[i]*delta_x_L(i);
		torqueR(i) = k_R[i]*delta_x_R(i);
	}

	float massage_sinL=abs(40*sin(double(2*3.14*0.1*timeIndex*0.01))); 
	float massage_sinR=abs(80*sin(double(2*3.14*0.1*timeIndex*0.01)));
	torqueL(0)=torqueL(0)+massage_sinL*0.15;
	torqueL(1)=torqueL(1)+massage_sinL*0.85;
	torqueR(0)=torqueR(0)+massage_sinR*0.15;
	torqueR(1)=torqueR(1)+massage_sinR*0.85;
	float t=torqueL(1);
	torqueL = kin->JacobianL.transpose()*(torqueL);  //jacobian乘上force變成torque
	torqueR = kin->JacobianR.transpose()*(torqueR);

	for( int i = 0; i < 6; i++)  //torque再乘一個gain
	{
		torqueL(i) = kx_L[i]*torqueL(i);
		torqueR(i) = kx_R[i]*torqueR(i);
	}
	timeIndex++;

}
void control::CartesianForceControl(optoforce forceL,optoforce forceR,rl::math::Vector &torqueL , rl::math::Vector &torqueR)
{
	kin->updateJacobian(jointL->angle,ARM_L); //更新jacobian
	kin->updateJacobian(jointR->angle,ARM_R);

	rl::math::Vector delta_x_L(6),delta_x_R(6);
	tcpL->OTG(delta_x_L, kin->JacobianL, jointL->angle, jointL->lastAngle); //OTG計算delta_x
	tcpR->OTG(delta_x_R, kin->JacobianR, jointR->angle, jointR->lastAngle);

	float k_L[6]={200,200,200,100,100,100}; //cartesian space gain
	float k_R[6]={200,200,200,100,100,100}; //cartesian space gain
	float kx_L[6]={0.7,1.5,6.0,0.7,0,0};  //joint space gain
	float kx_R[6]={1.5,1.5,1.5,1.5,0,0};  //joint space gain

	for( int i = 0; i < 6; i++)//delta_x乘上gain變成force
	{
		torqueL(i) = k_L[i]*delta_x_L(i);
		torqueR(i) = k_R[i]*delta_x_R(i);
	}
	
	

	int temp=forceL.Fz;
	if ((torqueL(0) - 0.15*(temp))<0) torqueL(0)=0;
	else  torqueL(0)=torqueL(0) - 0.15*temp;

    

	torqueL = kin->JacobianL.transpose()*(torqueL);  //jacobian乘上force變成torque
	torqueR = kin->JacobianR.transpose()*(torqueR);

	for( int i = 0; i < 6; i++)  //torque再乘一個gain
	{
		torqueL(i) = kx_L[i]*torqueL(i);
		torqueR(i) = kx_R[i]*torqueR(i);
	}


}
void control::JointControl(rl::math::Vector &torqueL ,rl::math::Vector &torqueR) //teach and play
{
	for (int i=0;i<6;i++)
	{
		torqueL(i)=0; //teach的時候要給0
		torqueR(i)=0;
	}

	if( jKey == 't' )
	{
		if( bTeachFlag == false)
		{		
			bTeachFlag = true;
			countIndex = 0;
			
			//清除記憶體資料
			memset (teachTrajL, 0, sizeof(teachTrajL));
			memset (gobackTrajL,0, sizeof(gobackTrajL));
			memset (teachTrajR, 0, sizeof(teachTrajR));
			memset (gobackTrajR,0, sizeof(gobackTrajR));

		
			//teachCounter從1開始累計
			for(int i=0; i<6; i++)
			{
				teachCounterL[i] = 1;
				teachCounterR[i] = 1;
			}
			
			//記錄teach的起始位置
			for(int i=0; i<6; i++)
			{
				teachTrajL[i][1][0] = jointL->angle[i];
				teachTrajR[i][1][0] = jointR->angle[i];
			}
		}
		const float gap = 0.0000001*3.14/180;
		for(int i=0; i<6; i++)
		{
			if( fabs( jointL->angle[i]-jointL->lastAngle[i] ) > gap ) // 各軸位移角度超過gap時做紀錄
			{	
				teachTrajL[i][0][teachCounterL[i]] = countIndex;	// 記錄teach角度變化的時間點
				teachTrajL[i][1][teachCounterL[i]] = jointL->angle[i];		// 記錄teach角度變化的encoder
				teachCounterL[i]++;
			}
			if( fabs( jointR->angle[i]-jointR->lastAngle[i] ) > gap ) // 各軸位移角度超過gap時做紀錄
			{	
				teachTrajR[i][0][teachCounterR[i]] = countIndex;	// 記錄teach角度變化的時間點
				teachTrajR[i][1][teachCounterR[i]] = jointR->angle[i];		// 記錄teach角度變化的encoder
				teachCounterR[i]++;
			}
		}
	}
	if( jKey != 't' && bTeachFlag==true)
	{
		bTeachFlag = false;
		for(int i=0; i<6; i++){
			teachTrajL[i][0][teachCounterL[i]] = 0;	// 在 teach index 插入結束記號
			teachTrajR[i][0][teachCounterR[i]] = 0;	// 在 teach index 插入結束記號
		}
	}
	//============play mode============
	if( jKey == 'p' )
	{
		if( bPlayFlag == false)
		{
			bPlayFlag=true;
			countIndex = 0;

			//teachCounter從1開始執行			
			for(int i=0; i<6; i++)
			{
				playCounterL[i] = 1;
				playCounterR[i] = 1;
			}
			// 計算goback到teach起始點的軌跡

			for(int i=0; i<6; i++){
				gobackGapL[i] = (teachTrajL[i][1][0]-jointL->angle[i])/gobackTrajSize;
				gobackGapR[i] = (teachTrajR[i][1][0]-jointR->angle[i])/gobackTrajSize;
			}

			for(int i=0; i<6; i++){
				for(int j=0; j<gobackTrajSize; j++){
					gobackTrajL[i][gobackTrajSize-1-j] = teachTrajL[i][1][0]-gobackGapL[i]*j;
					gobackTrajR[i][gobackTrajSize-1-j] = teachTrajR[i][1][0]-gobackGapR[i]*j;
				}
			}
		}
		int I_SW=0;		// Integrator switch
		// goback到teach起始點
		static float inputL[6]={0};
		static float inputR[6]={0};
			
 		if( countIndex < gobackTrajSize )
		{
			I_SW=0;
			for(int i=0; i<6; i++)
			{
				inputL[i] = gobackTrajL[i][countIndex];
				inputR[i] = gobackTrajR[i][countIndex];
			}
		}
		// 執行teach記錄的軌跡
		else
		{
			I_SW=1;		
			for(int i=0; i<6; i++)
			{
				if( countIndex >= teachTrajL[i][0][playCounterL[i]]+gobackTrajSize
					&& teachTrajL[i][0][playCounterL[i]] > 0.5 ){	//teach index 結束記號為0，所以要>0.5
					inputL[i] = teachTrajL[i][1][playCounterL[i]];
					playCounterL[i]++;
				}
				if( countIndex >= teachTrajR[i][0][playCounterR[i]]+gobackTrajSize
					&& teachTrajR[i][0][playCounterR[i]] > 0.5 ){	//teach index 結束記號為0，所以要>0.5
					inputR[i] = teachTrajR[i][1][playCounterR[i]];
					playCounterR[i]++;
				}
			}
			if (teachTrajL[0][0][playCounterL[0]]==0 && teachTrajL[1][0][playCounterL[1]]==0 &&
				teachTrajL[2][0][playCounterL[2]]==0 && teachTrajL[3][0][playCounterL[3]]==0 &&
				teachTrajL[4][0][playCounterL[4]]==0 && teachTrajL[5][0][playCounterL[5]]==0 &&
				teachTrajR[0][0][playCounterR[0]]==0 && teachTrajR[1][0][playCounterR[1]]==0 &&
				teachTrajR[2][0][playCounterR[2]]==0 && teachTrajR[3][0][playCounterR[3]]==0 &&
				teachTrajR[4][0][playCounterR[4]]==0 && teachTrajR[5][0][playCounterR[5]]==0 )
			{
				jointPlayDone=true;
			}
			
		}
//-------------------------------------------------------------------------------------------------------
		const int inteGainRatioL = 5;
		const int inteGainRatioR = 10;
		float inteCompL[6]={0};
		float inteCompR[6]={0};

		Integrator( inteCompL[0], 0.05*inteGainRatioL, inputL[0], jointL->lastAngle[0], I_SW );
		Integrator( inteCompL[1], 0.05*inteGainRatioL, inputL[1], jointL->lastAngle[1], I_SW );
		Integrator( inteCompL[2], 0.17/0.354*0.05*inteGainRatioL, inputL[2], jointL->lastAngle[2], I_SW );
		Integrator( inteCompL[3], 0.17/0.354*0.05*inteGainRatioL, inputL[3], jointL->lastAngle[3], I_SW );

		Integrator( inteCompR[0], 0.05*inteGainRatioR, inputR[0], jointR->lastAngle[0], I_SW );
		Integrator( inteCompR[1], 0.05*inteGainRatioR, inputR[1], jointR->lastAngle[1], I_SW );
		Integrator( inteCompR[2], 0.17/0.354*0.05*inteGainRatioR, inputR[2], jointR->lastAngle[2],I_SW );
		Integrator( inteCompR[3], 0.17/0.354*0.05*inteGainRatioR, inputR[3], jointR->lastAngle[3], I_SW );
//-------------------------------------------------------------------------------------------------------
		float samplingTime=0.001;
		float dampCompL[6]={0};
		float dampCompR[6]={0};

		for(int i=0; i<6; i++)
		{
			//float DamperGenerator( float dampGain, float pos, float pos1, float T);	// 微分
			dampCompL[i] = DamperGenerator( 10, jointL->angle[i], jointL->lastAngle[i], samplingTime);
			dampCompR[i] = DamperGenerator( 10, jointR->angle[i], jointR->lastAngle[i], samplingTime);
		}
//--------------------------------------------------------------------------------------------------------
		const float impedanceL = 2000;
		const float impedanceR = 5000;
		float updateTorqL[6]={0};
		float updateTorqR[6]={0};

		//float DigitalModel( float impedance, float posGain, float loadGain, int sw, float input, float pos, float pos1, float pos2, float T, MOTOR_DATA motor);
		updateTorqL[0] = DigitalModel( impedanceL, 1, 0.0001, 1, inputL[0],  jointL->angle[0], jointL->lastAngle[0], jointL->lastAngle2[0], samplingTime, M_DATA[0]);
		updateTorqL[1] = DigitalModel( impedanceL, 1, 0.0001, 1, inputL[1],  jointL->angle[1], jointL->lastAngle[1], jointL->lastAngle2[1], samplingTime, M_DATA[1]);
		updateTorqL[2] = DigitalModel( impedanceL, 1, 0.01,	1, inputL[2],  jointL->angle[2], jointL->lastAngle[2], jointL->lastAngle2[2], samplingTime, M_DATA[2]);
		updateTorqL[3] = DigitalModel( impedanceL, 1, 0.01, 1, inputL[3],  jointL->angle[3], jointL->lastAngle[3], jointL->lastAngle2[3], samplingTime, M_DATA[3]);
		//updateTorqL[4] = DigitalModel( impedanceL, 1, 0.01,	1, inputL[4],  jointL->angle[4], jointL->lastAngle[4], jointL->lastAngle2[4], samplingTime, M_DATA[4]);
		//updateTorqL[5] = DigitalModel( impedanceL, 1, 0.01, 1, inputL[5],  jointL->angle[5], jointL->lastAngle[5], jointL->lastAngle2[5], samplingTime, M_DATA[5]);
		updateTorqR[0] = DigitalModel( impedanceR, 1, 0.0001, 1, inputR[0],  jointR->angle[0], jointR->lastAngle[0], jointR->lastAngle2[0], samplingTime, M_DATA[0]);
		updateTorqR[1] = DigitalModel( impedanceR, 1, 0.0001, 1, inputR[1],  jointR->angle[1], jointR->lastAngle[1], jointR->lastAngle2[1], samplingTime, M_DATA[1]);
		updateTorqR[2] = DigitalModel( impedanceR, 1, 0.01, 1, inputR[2],  jointR->angle[2], jointR->lastAngle[2], jointR->lastAngle2[2], samplingTime, M_DATA[2]);
		updateTorqR[3] = DigitalModel( impedanceR, 1, 0.01, 1, inputR[3],  jointR->angle[3], jointR->lastAngle[3], jointR->lastAngle2[3], samplingTime, M_DATA[3]);		
//-------------------------------------------------------------------------------------------------------------------
		for(int i=0; i<6; i++) 
		{
			torqueL(i) = updateTorqL[i] + inteCompL[i] - dampCompL[i] ;
			torqueR(i) = updateTorqR[i] + inteCompR[i] - dampCompR[i] ;
		}
	}
	if( jKey!='p' && bPlayFlag==true)
	{ 
		jointPlayDone=false;
		bPlayFlag=false;
	}
	countIndex++;
}



void control::Integrator( float &inteComp, float inteGain, float input, float pos, int I_SW)	//積分
{		
	inteComp = (inteGain*(input-pos)+inteComp)*I_SW;
}


float control::DamperGenerator( float diffGain, float pos, float pos1, float T)	// 微分
{
	float diffComp = diffGain*(pos-pos1)/T;
	return diffComp;
}


float control::DigitalModel( float impedance, float posGain, float loadGain, int sw, float input, float pos, float pos1, float pos2, float T, MOTOR_DATA motor)
{
	float torque=0;
	torque = input*impedance-pos*(impedance*posGain*sw+posGain*loadGain*motor.Im/T/T+posGain*loadGain*(motor.Cm*motor.R+motor.Km*motor.Kb)/T/motor.R)+pos1*(2*posGain*loadGain*motor.Im/T/T+posGain*loadGain*(motor.Cm*motor.R+motor.Km*motor.Kb)/T/motor.R)-pos2*posGain*loadGain*motor.Im/T/T;
	return torque;
}