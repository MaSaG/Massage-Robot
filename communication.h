#include "SystemParameter.h"

/*
舊的massage robot因為用兩張IMP卡分別控制兩手，
所以用Card_Index來區分data要送(讀)到哪一張卡
*/
#include "IMCDriver.h"
#include "IMCDefine.h"
#include <stdio.h>



class communication
{
	public:
		//基本變數
		long encoderValue[2][6];
		float outputTorque[2][6];



		void Initial_IMPCard( int Card_Index );
		void Initial_Encoder( int Card_Index );
		void Initial_DAC( int Card_Index );
		void Read_Encoder( int Card_Index );
		void Output_Voltage( int Card_Index );
		void Close_IMPCard( int Card_Index );
		void Output_LM_Voltage( float outpLMVoltage);
		void Read_LM_Encoder( long* LMcount);
};




