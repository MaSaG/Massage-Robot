#include "SystemParameter.h"

/*
�ª�massage robot�]���Ψ�iIMP�d���O������A
�ҥH��Card_Index�ӰϤ�data�n�e(Ū)����@�i�d
*/
#include "IMCDriver.h"
#include "IMCDefine.h"
#include <stdio.h>



class communication
{
	public:
		//���ܼ�
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




