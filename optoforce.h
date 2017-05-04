#include <iostream>
#include "AbstractForceSensor.h"

#ifndef force_sensor
#define force_sensor

class optoforce : public AbstractForceSensor
{
public:
    optoforce(unsigned long a )
	{
		pAddr = new AmsAddr;
		Rxadd0 = 0x11002;//need to check the adderess in TwinCAT
		Txadd0 = 0x12002;//need to check the adderess in TwinCAT
		port = 300;
		dwData=0;
		Data=0;
		len=0x02;  //©T©wªº
		offset=a;   

		int nErr = 0;
		AdsPortOpen();
		nErr = AdsGetLocalAddress(pAddr);
		if(nErr)
		{
			std::cerr << "Error: AdsGetLocalAddress: " << nErr << std::endl;
		}
		pAddr->port = port;


		zero();
	}
	void zero()
	{
		add1=offset;
		dwData = 0x0000;  //unzero
		AdsSyncWriteReq(pAddr,Rxadd0,add1,len,&dwData);
		Sleep(10);
		dwData = 0xFF00;  //zero
		AdsSyncWriteReq(pAddr,Rxadd0,add1,len,&dwData);
	}
	void getForce()
	{
		add1= offset;
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Fx=dwData;

		add1=offset+0x02; 
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Fy=dwData;

		add1= offset+0x04; 
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Fz=dwData;

		add1= offset+0x06; 
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Tx=dwData;

		add1= offset+0x08; 
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Ty=dwData;

		add1= offset+0x0A; 
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&dwData);
		Tz=dwData;

	}
	void setFilter(DWORD filter)
	{
		//000 = No filter 
		//001 = 500 Hz 
		//010 = 150 Hz 
		//011 = 50 Hz 
		//100 = 15 Hz 
		//101 = 5 Hz 
		//110 = 1.5 Hz
		add1=offset;
		dwData = 0xFF00 | filter;  //zero
		AdsSyncWriteReq(pAddr,Rxadd0,add1,len,&dwData);
	}
	unsigned long getError()
	{
		add1= offset+0x0C;
		Data=0x0;
		AdsSyncReadReq(pAddr,Txadd0,add1,len,&Data);
		return Data;
	}
};

#endif