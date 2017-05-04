#include "TimerHandler.h"




//===全域class區===
communication Com1;  
jointSpace jointL(0,0,0,0,0,0);
jointSpace jointR(0,0,0,0,0,0);
kinematics kin;
tcp6D tcpL(Eigen::Vector3d(0.65,0,0),Eigen::Vector3d(-180,0,90));
tcp6D tcpR(Eigen::Vector3d(0.65,0,0),Eigen::Vector3d(-180,0,90));
dynamics dyn;
control cntrl(tcpL,tcpR,jointL,jointR,kin);
activeSkill* skill = new activeSkill;
optoforce forceL(0x27);
optoforce forceR(0x37);
userInterface UI;
passiveSkill pas;
//===class區結束===

int Control_Mode=Free_Mode; // 預設是重力補償模式


int main ()
{
	forceL.setFilter(0x04);//////////////////////////////////
	Com1.Initial_IMPCard( ARM_L );    //ARM_L==0
	Com1.Initial_IMPCard( ARM_R );  //ARM_R==1

	//================開啟控制迴路Timer==============
    LARGE_INTEGER  liPeriod;   // timer period
    HANDLE         hTimer;     // timer handle
    liPeriod.QuadPart = SERVOLOOP_TIME * 10000;  // 10000 = 1ms
	// Create a periodic timer
	if (! (hTimer = RtCreateTimer(
									NULL,            // security
									0,               // stack size - 0 uses default
									TimerHandler,    // timer handler
									NULL,            // NULL context (argument to handler)
									RT_PRIORITY_MAX, // priority
									CLOCK_FASTEST) 
		  )
	   )      // RTX HAL timer
	{
		ExitProcess(1);
	}
	RtSetTimerRelative( hTimer, &liPeriod, &liPeriod);
	//==============開啟顯示畫面的Timer=============
	LARGE_INTEGER	liPeriod2;
	HANDLE         hTimer2;
	liPeriod2.QuadPart = 20 * 10000;  // 10000 = 1ms
	if (! (hTimer2 = RtCreateTimer(
									NULL,            // security
									0,               // stack size - 0 uses default
									TimerHandler2,    // timer handler
									NULL,            // NULL context (argument to handler)
									50, // priority
									CLOCK_FASTEST) 
		  )
	   )      // RTX HAL timer
	{
		// TO DO:  exception code here
		// RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
		ExitProcess(1);
	}
	RtSetTimerRelative( hTimer2, &liPeriod2, &liPeriod2);


	//================主要流程開始================

	char cKey=' ';
	
	while (1)
	{
		UI.menu();
		std::cout<<"Command:";
		std::cin >> cKey;
		switch (cKey)
		{
			case '0':
				std::cout<<"Free mode"<<std::endl;
				Control_Mode=Free_Mode;
				break;
			case '1':
				std::cout<<"Cartesian Control"<<std::endl;
				tcpL.targetPos=tcpL.currentPos;
				tcpR.targetPos=tcpR.currentPos;
				Control_Mode=Cartesian_Control;
				break;	

			case '2': // cp
				std::cout<<"Joint Control"<<std::endl;
				std::cout<<"press [t] to teach"<<std::endl;
				std::cout<<"press [p] to play"<<std::endl;
				std::cout<<"press [ESC] to exit"<<std::endl;
				Control_Mode = Joint_Control;
				
				while(true)
				{
					if (_kbhit())
					{
						cntrl.jKey = _getch();
						printf("%c\n",cntrl.jKey);

						if (cntrl.jKey == ESC_KEY)
						{
							break;
						}
		
					}
				}
				break;
			case '3':
				tcpL.targetPos=Eigen::Vector3d(0,0.5,0);
				tcpR.targetPos=Eigen::Vector3d(0,0.5,0);
				break;
			case '4':
				std::cout<<"Cartesian Force Control"<<std::endl;
				forceL.setFilter(0x00);
				forceR.setFilter(0x00);
				tcpL.targetPos=tcpL.currentPos;
				tcpR.targetPos=tcpR.currentPos;
				Control_Mode=CartesianForce_Control;
				break;
			case '5':
				forceL.zero();
				break;

			case '7':
				tcpL.targetPos(0)=tcpL.targetPos(0)+0.01;
				break;
			case 's':
				if (UI.recordSW==0)
				{
					UI.recordSW=1;
					std::cout<<"data recording"<<std::endl;
				}
				else
				{
					UI.recordSW=0;
					std::cout<<"stop data recording"<<std::endl;
				}
				break;
			case 'z':
				skill->massage.teachMotion(tcpR.currentPos,tcpL.currentPos,cntrl.jKey , Control_Mode);
				break;
			case 'x':
				skill->massage.Pressing(tcpR.targetPos,tcpL.targetPos, Control_Mode, cntrl.timeIndex);
				break;
			case 'c':
				skill->massage.Rubbing(tcpR.targetPos,tcpL.targetPos, Control_Mode);
				break;
			case 'v':
				skill->massage.Stroking(tcpR.targetPos,tcpL.targetPos, Control_Mode,cntrl.jKey,cntrl.jointPlayDone);
				break;
			case 'd':
				if (pas.collisionSW==false)
				{
					std::cout<<"collision detection ON"<<std::endl;
					pas.collisionSW=true;
				}
				else
				{
					std::cout<<"collision detection OFF"<<std::endl;
					pas.collisionSW=false;
				}
				break;
			case'f':
				if (pas.jointLimitation==false)
				{
					std::cout<<"Joint Limitation ON"<<std::endl;
					pas.jointLimitation=true;
				}
				else
				{
					std::cout<<"Joint Limitation OFF"<<std::endl;
					pas.jointLimitation=false;
				}
				break;
				

		}
		if (cKey =='q')break; //結束程式
	}


	//================主要流程結束================

	//================以下為關閉的流程================

	Com1.Close_IMPCard( ARM_L );
	Com1.Close_IMPCard( ARM_R );
	delete skill;
	RtDeleteTimer( hTimer );
	RtDeleteTimer( hTimer2 );
	printf("Close System\n");

	ExitProcess(0);
}