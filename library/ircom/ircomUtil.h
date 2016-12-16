#ifndef _IRCOMUTIL
#define _IRCOMUTIL 

#include "ircom.h"

void sendWord( char* word );

void startSpeed();

void moveToSensor( int base_speed, IrcomMessage msg );

#endif
