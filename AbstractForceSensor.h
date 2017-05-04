 #pragma once

#include <windows.h>
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsDef.h"
#include "C:\Twincat\AdsApi\TcAdsDll\Include\TcAdsApi.h"

class AbstractForceSensor
{
	public:
	virtual void zero() = 0; 
	virtual void getForce() = 0; 
	virtual void setFilter(DWORD) = 0; 
	virtual unsigned long getError() = 0;
	short Fx,Fy,Fz,Tx,Ty,Tz;
	
    protected:
    AmsAddr* pAddr;
    unsigned long Rxadd0;
    unsigned long Txadd0;
    unsigned short port;
    unsigned long add1,len,offset;
    DWORD dwData;
	WORD Data;



};