#include "stdio.h"
#include "string.h"
#include "math.h"

#include "ircomSend.h"
#include "ircom.h"

#include "btcom/btcom.h"

#include "motor_led/advance_one_timer/e_motors.h" 

#include "custom_util/motor_control.h"

// float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};


void sendWord( char* word )
{	
	int i;
	int length = strlen( word );

	for( i = 0; i < length; i++ )
	{	
		ircomSend( (int) i );
		while (ircomSendDone() == 0);
	}
}


