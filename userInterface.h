#ifndef USER_INTERFACE
#define USER_INTERFACE


#include "tcp6D.h"
#include "jointSpace.h"
#include "dynamics.h"
#include "kinematics.h"
#include "SystemParameter.h"
#include "optoforce.h"
#include "multi_console\ConsoleLogger.h"
#include <iostream>


class userInterface
{
public:
	bool recordSW;


	userInterface()
	{
		initial_console=0;
		recordSW=0;
		recordInitial=0;
	}
	

	
	
	void dispaly(tcp6D& _tcpL,jointSpace& _jointL, optoforce& _forceL)
	{
		if(initial_console==0)
		{
			
			another_console.Create("This is the first console");
			initial_console=1;
		}
		another_console.cls();
		float a=_tcpL.targetPos(0);
		another_console.printf("Current Pos :\t%6.2f\t%6.2f\t%6.2f\n",_tcpL.currentPos(0),_tcpL.currentPos(1),_tcpL.currentPos(2));
		another_console.printf("Current Ori :\t%6.1f\t%6.1f\t%6.1f\n",_tcpL.currentOri(0)*180/PI,_tcpL.currentOri(1)*180/PI,_tcpL.currentOri(2)*180/PI);
		another_console.printf("Target Pos : \t%6.2f\t%6.2f\t%6.2f\n",_tcpL.targetPos(0),_tcpL.targetPos(1),_tcpL.targetPos(2));
		another_console.printf("Target Ori : \t%6.1f\t%6.1f\t%6.1f\n",_tcpL.targetOri(0)*180/PI,_tcpL.targetOri(1)*180/PI,_tcpL.targetOri(2)*180/PI);
		another_console.printf("Current Joint Angle : %.1f  %.1f  %.1f  %.1f  %.1f  %.1f\n",_jointL.angle[0]*RAD2DEG,_jointL.angle[1]*RAD2DEG,_jointL.angle[2]*RAD2DEG,_jointL.angle[3]*RAD2DEG,_jointL.angle[4]*RAD2DEG,_jointL.angle[5]*RAD2DEG);
		another_console.printf("Left F/T Sensor  Fx:%d  Fy:%d  Fz:%d  Tx:%d  Ty:%d  Tz:%d\n"  ,_forceL.Fx,_forceL.Fy,_forceL.Fz,_forceL.Tx,_forceL.Ty,_forceL.Tz);
	}


	void recordData(rl::math::Vector delta_v)  
	{
		if (recordSW==1)
		{
			if (recordInitial==0)
			{
				
				pFile = fopen( "deltaV.txt","w" );
				//pFile2 = fopen( "ForceR.txt","w");
				recordInitial=1;
			}
			fprintf(pFile,"%f, %f, %f, \n",delta_v(0),delta_v(1),delta_v(2));
			//fprintf(pFile2,"%d, %d, %d, %d, %d, %d, \n",int(forceR.Fx),int(forceR.Fy),int(forceR.Fz),int(forceR.Tx),int(forceR.Ty),int(forceR.Tz));

		}
		else
		{
			if (recordInitial==1)
			{
				fclose(pFile);
				//fclose(pFile2);
				recordInitial=0;
			}
		}
	}

	void menu()
	{
		std::cout<<"[0] Free Mode"<<std::endl;
		std::cout<<"[1] Cartesian Control Mode"<<std::endl;
		std::cout<<"[2] Joint Control Mode"<<std::endl;
		std::cout<<"[z] Teach massage trajectory"<<std::endl;
		std::cout<<"[x] Play pressing"<<std::endl;
		std::cout<<"[c] Play rubbing"<<std::endl;
		std::cout<<"[v] Play stroking"<<std::endl;
		std::cout<<"[s] Record data ([s] again to stop recording) "<<std::endl;
		std::cout<<"[d] Collision Detection"<<std::endl;
		std::cout<<"[f] Joint Angle Limitation"<<std::endl;
		std::cout<<"[q] Quit "<<std::endl;

	}

	void keyboardFunction();

private:
	
	bool recordInitial;
	FILE *pFile;
	FILE *pFile2;
	bool initial_console;
	CConsoleLoggerEx another_console;

};



#endif