#ifndef TIMER_HANDLER_H
#define TIMER_HANDLER_H


#include "SystemParameter.h"
#include "communication.h"
#include "jointSpace.h"
#include "optoforce.h"
#include "tcp6D.h"
#include "kinematics.h"
#include "dynamics.h"
#include "userInterface.h"
#include "control.h"
#include "activeSkill.h"
#include "passiveSkill.h"
#include <rtapi.h> //RTFCNDCL、RtCreateTimer




// function prototype for periodic timer function
void RTFCNDCL TimerHandler( void * nContext );
void RTFCNDCL TimerHandler2( void * nContext );
// Interrupt handler prototype
void RTFCNDCL InterruptHandler( void * nContext );  //不會用  沒用到


#endif