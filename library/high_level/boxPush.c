
#include "btcom/btcom.h"

#include "custom_util/motor_control.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "high_level/boxPush.h"

#include "uart/e_uart_char.h"

#include "ircom/e_ad_conv.h"
 

/*
* This file is used to push the box.
*
* It uses a braitenberg aggression approach to perform this behaviour.
*
*/


/*
* Push the box infinitely.
*/
void startBoxPush()
{
	while(1)
	{
		pushBox();
	}
}


/*
* Perform one adjustment iteration of pushing the box.
*/
void pushBox()
{		
		int base_speed = 500;
		int max_speed = 800;
		
		// Get front proximity values
		int front_right_prox = e_get_calibrated_prox( 0 );
		int front_left_prox = e_get_calibrated_prox( 7 );
		
		int left_speed = base_speed;
		int right_speed = base_speed;
		
		// This will create the aggression behaviour.
		left_speed += front_right_prox;
		right_speed += front_left_prox;
		
		if( left_speed > max_speed )
		{
			left_speed = max_speed;
		}	
		if( right_speed > max_speed )
		{
			right_speed = max_speed;
		}
			
		left_speed = normalise_speed( left_speed );
		right_speed = normalise_speed( right_speed );
		
		set_wheel_speeds( left_speed, right_speed );

}
