#include "SystemParameter.h" 

#include "tcp6D.h"
#include "jointSpace.h"


class massageTechniques
{
public:
	Eigen::Vector3d cornerPosition[2][2];
	Eigen::Vector3d massagePosition[2][5];
	Eigen::Vector3d viaPosition[2];
	Eigen::Vector3d virtualPoint[2];
	
	massageTechniques();

	void teachMotion(Eigen::Vector3d &currentPosR,Eigen::Vector3d &currentPosL,char &jKey , int &Control_Mode); //��stroking���ʧ@�A���K�O���̤W�M�̤U���I�M�᤺��
	void Pressing(Eigen::Vector3d &targetPosR,Eigen::Vector3d &targetPosL, int &Control_Mode, int &timeIndex);
	void Rubbing(Eigen::Vector3d &targetPosR,Eigen::Vector3d &targetPosL, int &Control_Mode);
	void Stroking(Eigen::Vector3d &targetPosR,Eigen::Vector3d &targetPosL, int &Control_Mode,char &jKey,bool &jointPlayDone);

	void teachPosition_for_tapping(); // cp
	void play_tapping(); // cp
};