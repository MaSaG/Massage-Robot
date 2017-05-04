#include "TimerHandler.h"

extern communication Com1;
extern jointSpace jointL;
extern jointSpace jointR;
extern kinematics kin;
extern tcp6D tcpL;
extern tcp6D tcpR;
extern dynamics dyn;
extern control cntrl;
extern optoforce forceL;
extern optoforce forceR;
extern passiveSkill pas;
extern int Control_Mode;

extern userInterface UI;

void RTFCNDCL TimerHandler( PVOID context )
{
	Com1.Read_Encoder(ARM_L); //���oencoder value
	Com1.Read_Encoder(ARM_R);
	jointL.updataJointAngle(Com1.encoderValue[ARM_L],ENC2RAD_FACTOR[ARM_L]); //�Nencoder value���⦨joint angle
	jointR.updataJointAngle(Com1.encoderValue[ARM_R],ENC2RAD_FACTOR[ARM_R]);
	kin.ForwardKinematics(tcpL.currentPos,tcpL.currentOri,tcpL.currentT,jointL.angle);  //�p��FK
	kin.ForwardKinematics(tcpR.currentPos,tcpR.currentOri,tcpR.currentT,jointR.angle);

	static rl::math::Vector torqueL(6),torqueR(6);
	static float GcompL[6],GcompR[6],LimitTorqueL[6],LimitTorqueR[6];

	dyn.GravityComp(GcompL , jointL.angle);  //���o���O���v��torque
	dyn.GravityComp(GcompR , jointR.angle);
	
	pas.collisionDetection(tcpL.delta_v,Control_Mode);

	pas.getRepulsiveTorque(LimitTorqueL,jointL.angle,ARM_L);
	pas.getRepulsiveTorque(LimitTorqueR,jointR.angle,ARM_R);

	switch (Control_Mode)
	{

		case Cartesian_Control:    //Cartesian impedance control
			cntrl.CartesianControl(torqueL,torqueR);
			
			for(int i=0; i<6; i++)
			{
				Com1.outputTorque[ARM_L][i] = GcompL[i]+LimitTorqueL[i]+torqueL(i);
				Com1.outputTorque[ARM_R][i] = GcompR[i]+LimitTorqueR[i]+torqueR(i);
			}
			break;

		case Joint_Control:      //Joint impedance control
			cntrl.JointControl(torqueL,torqueR);
			for(int i=0; i<6; i++)
			{
				if(i==1)
				{
				Com1.outputTorque[ARM_L][i] = GcompL[i];
				Com1.outputTorque[ARM_R][i] = GcompR[i];
				
				}
				else
				{
				Com1.outputTorque[ARM_L][i] = GcompL[i]+torqueL(i);
				Com1.outputTorque[ARM_R][i] = GcompR[i]+torqueR(i);
				}
				
			}
			tcpL.updateOTG_Last(); //�]��joint control�S���Ψ�OTG�A�b���^Cartesian Control���ɭԷ|�]���S���W�@����Ʀ�error�A�ҥH�ݭn�B�~����s�W�@�����
			tcpR.updateOTG_Last();
			break;

		case Cartesian_Control2:
			cntrl.CartesianControl2(torqueL,torqueR);
			for(int i=0; i<6; i++)
			{
				Com1.outputTorque[ARM_L][i] = GcompL[i]+torqueL(i);
				Com1.outputTorque[ARM_R][i] = GcompR[i]+torqueR(i);
			}
			break;

		case CartesianForce_Control:
			cntrl.CartesianForceControl(forceL,forceR,torqueL,torqueR);
			for(int i=0; i<6; i++)
			{
				Com1.outputTorque[ARM_L][i] = GcompL[i]+torqueL(i);
				Com1.outputTorque[ARM_R][i] = GcompR[i]+torqueR(i);
			}
			break;

		case Free_Mode:      //���O���v�Ҧ�
		default:
			for(int i=0; i<6; i++)
			{
				Com1.outputTorque[ARM_L][i] = GcompL[i]+LimitTorqueL[i];
				Com1.outputTorque[ARM_R][i] = GcompR[i]+LimitTorqueR[i];
			}
			tcpL.updateOTG_Last();
			tcpR.updateOTG_Last();
			break;

	}

	//�u�n�b�o�̥Τ@�ӷs��flag�A�o��flag�u��TimerHandler2�̭��Q���ܡA�M��j���X�ܦ����O���v�A�N��a�o��flag�Ȱ������H���ʧ@

	Com1.Output_Voltage(ARM_L);  //�e�Xtorque command
	Com1.Output_Voltage(ARM_R);
	jointL.updateLastJointAngle();  //��slast angle
	jointR.updateLastJointAngle(); 

}

void RTFCNDCL TimerHandler2( PVOID context ) //�o��timer�w�]�O50ms���_�@��
{
	forceL.getForce();
	forceR.getForce();
	UI.dispaly(tcpL,jointL,forceL); //�}�t�@��console�h��ܪ��A
	UI.recordData(tcpL.delta_v);
	//����Ʊ�[�W�b����ɭԳ������ʧ@�Ȱ������O�A�Ʊ�qIOŪ��sensor����L��button�ӼȰ��ʧ@
	//�٦��������\�൥��
}