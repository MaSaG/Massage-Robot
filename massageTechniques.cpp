#include "massageTechniques.h"
#include <windows.h> //Sleep


massageTechniques::massageTechniques()
{
	for (int i=0;i<2;i++) 
	{
		cornerPosition[0][i]=Eigen::Vector3d::Zero();
		cornerPosition[1][i]=Eigen::Vector3d::Zero();
	}
	for (int i=0;i<5;i++) 
	{
		massagePosition[0][i]=Eigen::Vector3d::Zero();
		massagePosition[1][i]=Eigen::Vector3d::Zero();
	}
	for (int i=0;i<2;i++) viaPosition[i]=Eigen::Vector3d::Zero();

	virtualPoint[0]=Eigen::Vector3d(0.02,0.08,0);
	virtualPoint[1]=Eigen::Vector3d(0.02,0.08,0);
}

void massageTechniques::teachMotion(Eigen::Vector3d &currentPosR,Eigen::Vector3d &currentPosL, char &jKey, int &Control_Mode)
{
	Control_Mode=Free_Mode;  
	char tKey=' ';
	std::cout<<"press enter to save first point"<<std::endl;
	while(1)  //先記錄第一個點(背部上方
	{
		if (_kbhit())
		{ 
			tKey =_getch();
			if (tKey == 13)
			{
				cornerPosition[0][0]=currentPosL;
				cornerPosition[1][0]=currentPosR;
				break;
			}
		}
	}

	Control_Mode=Joint_Control;  //切到joint teach mode
	jKey='t';
	std::cout<<"teach trajectory then press enter"<<std::endl;
	while(1)  //teach joint mode軌跡
	{
		if (_kbhit())
		{ 
			tKey =_getch();
			if (tKey == 13)
			{
				jKey=' ';  //結束teach
				cornerPosition[0][1]=currentPosL;//紀錄結束的點
				cornerPosition[1][1]=currentPosR;
				break;
			}
		}
	}
	Control_Mode=Free_Mode;
	Sleep(200);
	std::cout<<"press enter to save via point"<<std::endl;
	while(1)  
	{
		if (_kbhit())
		{ 
			tKey =_getch();
			if (tKey == 13)
			{
				viaPosition[0]=currentPosL;//via point
				viaPosition[1]=currentPosR;
				break;
			}
		}
	}

	/*********/
	
	Eigen::Vector3d temp1=cornerPosition[0][1]-cornerPosition[0][0];
	Eigen::Vector3d temp2=cornerPosition[1][1]-cornerPosition[1][0];
	float dis1=sqrt(pow(temp1(0),2)+pow(temp1(1),2)+pow(temp1(2),2));
	float dis2=sqrt(pow(temp2(0),2)+pow(temp2(1),2)+pow(temp2(2),2));
	int interpolate=3;
	
	for (int i=0;i<=interpolate+1;i++)
	{
		massagePosition[0][i]=cornerPosition[0][0]+temp1/(interpolate+2)*i;//+virtualpoint;
		massagePosition[1][i]=cornerPosition[1][0]+temp2/(interpolate+2)*i;//+virtualpoint;
		//cout<<i<<":"<<position[0][i]<<endl;
	}
	/*********/
}

void massageTechniques::Pressing(Eigen::Vector3d &targetPosR ,Eigen::Vector3d &targetPosL, int &Control_Mode,int &timeIndex)
{
	
	Control_Mode=Cartesian_Control;

	for (int i=0;i<=3;i++)
	{
		targetPosL=massagePosition[0][i]; //移到目標點
		targetPosR=massagePosition[1][i];
		Sleep(1200);
		targetPosL=massagePosition[0][i]+virtualPoint[0];
		targetPosR=massagePosition[1][i]+virtualPoint[1];
		Sleep(1000);
		timeIndex=0;
		Control_Mode=Cartesian_Control2;
		
		while (timeIndex<3000){} //執行一段時間按摩

		Control_Mode=Cartesian_Control;

		targetPosL= viaPosition[0];//移到預備位置
		targetPosR= viaPosition[1];
		Sleep(1500);
		
	}

}

void massageTechniques::Rubbing(Eigen::Vector3d &targetPosR,Eigen::Vector3d &targetPosL, int &Control_Mode)
{
	Control_Mode=Cartesian_Control;
	
	double r = 0.03;
	
	for (int i=0;i<=3;i++)
	{
		targetPosL=massagePosition[0][i]; //移到目標點
		targetPosR=massagePosition[1][i];
		Sleep(1200);
		targetPosL=massagePosition[0][i]+virtualPoint[0];
		targetPosR=massagePosition[1][i]+virtualPoint[1];
		Sleep(1000);
		double x, y, z;
		for( int j = 0; j < 16; j++)
		{
			double th = 45.0*j;
			x = r*cos(th);
			z = r*sin(th);
			y = 0.8*(-x+0.03);  // maximum while x=-0.03, y = 0.3*0.06=0.18
			targetPosL=Eigen::Vector3d( massagePosition[0][i](0)+virtualPoint[0](0) + x, massagePosition[0][i](1)+virtualPoint[0](1) + y , massagePosition[0][i](2)+virtualPoint[0](2) + z);
			targetPosR=Eigen::Vector3d( massagePosition[1][i](0)+virtualPoint[1](0) + x, massagePosition[1][i](1)+virtualPoint[1](1) + y , massagePosition[1][i](2)+virtualPoint[1](2) + z);
			Sleep(300);
		}

		
	targetPosL=viaPosition[0]; //移到目標點
	targetPosR=viaPosition[1];
	Sleep(1500);
		
	}


}

void massageTechniques::Stroking(Eigen::Vector3d &targetPosR,Eigen::Vector3d &targetPosL, int &Control_Mode,char &jKey,bool &jointPlayDone)
{
	  
	targetPosL=cornerPosition[0][0]; //移到目標點
	targetPosR=cornerPosition[1][0];
	Control_Mode=Cartesian_Control;
	Sleep(1500);
	Control_Mode=Joint_Control;  //切到joint  mode
	jKey='p';  //play
	while (1)
	{
		if (jointPlayDone==true) 
		{   
			jointPlayDone=false;
			break;
		}
	}
	jKey=' ';
	Sleep(50);
	targetPosL=viaPosition[0]; //移到目標點
	targetPosR=viaPosition[1];
	Control_Mode=Cartesian_Control;  //切到Cartesian mode
	
	Sleep(1500);


}