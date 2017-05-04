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
	Com1.Read_Encoder(ARM_L); //取得encoder value
	Com1.Read_Encoder(ARM_R);
	jointL.updataJointAngle(Com1.encoderValue[ARM_L],ENC2RAD_FACTOR[ARM_L]); //將encoder value換算成joint angle
	jointR.updataJointAngle(Com1.encoderValue[ARM_R],ENC2RAD_FACTOR[ARM_R]);
	kin.ForwardKinematics(tcpL.currentPos,tcpL.currentOri,tcpL.currentT,jointL.angle);  //計算FK
	kin.ForwardKinematics(tcpR.currentPos,tcpR.currentOri,tcpR.currentT,jointR.angle);

	static rl::math::Vector torqueL(6),torqueR(6);
	static float GcompL[6],GcompR[6],LimitTorqueL[6],LimitTorqueR[6];

	dyn.GravityComp(GcompL , jointL.angle);  //取得重力補償的torque
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
			tcpL.updateOTG_Last(); //因為joint control沒有用到OTG，在切回Cartesian Control的時候會因為沒有上一筆資料而error，所以需要額外的更新上一筆資料
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

		case Free_Mode:      //重力補償模式
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

	//只要在這裡用一個新的flag，這個flag只有TimerHandler2裡面被改變，然後強制輸出變成重力補償，就能靠這個flag暫停機器人的動作

	Com1.Output_Voltage(ARM_L);  //送出torque command
	Com1.Output_Voltage(ARM_R);
	jointL.updateLastJointAngle();  //更新last angle
	jointR.updateLastJointAngle(); 

}

void RTFCNDCL TimerHandler2( PVOID context ) //這個timer預設是50ms中斷一次
{
	forceL.getForce();
	forceR.getForce();
	UI.dispaly(tcpL,jointL,forceL); //開另一個console去顯示狀態
	UI.recordData(tcpL.delta_v);
	//之後希望加上在任何時候都能讓動作暫停的指令，希望從IO讀取sensor或鍵盤或button來暫停動作
	//還有紀錄的功能等等
}