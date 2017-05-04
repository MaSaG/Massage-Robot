#ifndef CONTROL_H
#define CONTROL_H

#include "SystemParameter.h"
#include "jointSpace.h"
#include "tcp6D.h"
#include "kinematics.h"
#include "optoforce.h"
#define gobackTrajSize 4000

class control
{
public:
	tcp6D *tcpL;
	tcp6D *tcpR;
	jointSpace *jointL;
	jointSpace *jointR;
	kinematics *kin;
	int timeIndex;
	char jKey;  //for teach and play
	bool jointPlayDone;


	control(tcp6D &tcpLeft, tcp6D &tcpRight, jointSpace &jointspaceLeft, jointSpace &jointspaceRight, kinematics &kinmatic); // constructor

	void CartesianControl(rl::math::Vector &torqueL , rl::math::Vector &torqueR); //Cartesian Control
	void CartesianControl2(rl::math::Vector &torqueL , rl::math::Vector &torqueR); //Cartesian Control for pressing action
	void CartesianForceControl(optoforce forceL,optoforce forceR,rl::math::Vector &torqueL , rl::math::Vector &torqueR); //Cartesian Control with force feedback 目前是有點失敗的狀態
	void JointControl(rl::math::Vector &torqueL ,rl::math::Vector &torqueR); //Joint Control teach and play

	void CartesianControlTapping();  //cp

private:
	//for teach and play
	float teachTrajL[6][2][3*60*1000/SERVOLOOP_TIME];  //[6軸][時間.位置] 3(min)*60(sec)*1000(ms)/SERVOLOOP_TIM(ms)
	float teachTrajR[6][2][3*60*1000/SERVOLOOP_TIME];
	int teachCounterL[6];
	int teachCounterR[6];
	int playCounterL[6];
	int playCounterR[6];
	int countIndex;
	bool bTeachFlag;
	bool bPlayFlag;
	float gobackGapL[6];
	float gobackGapR[6];
	float gobackTrajL[6][gobackTrajSize];
	float gobackTrajR[6][gobackTrajSize];
	

	void Integrator( float &inteComp, float inteGain, float input, float pos, int I_SW);	//Joint control用到的積分器

	float DamperGenerator( float diffGain, float pos, float pos1, float T);	// Joint control用到的微分器

	float DigitalModel( float impedance, float posGain, float loadGain, int sw, float input, float pos, float pos1, float pos2, float T, MOTOR_DATA motor); //Joint control的model


};

#endif