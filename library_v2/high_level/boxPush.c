
#include "btcom/btcom.h"

#include "custom_util/motor_control.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "high_level/boxPush.h"

#include "uart/e_uart_char.h"

#include "ircom/e_ad_conv.h"
 


void startBoxPush()
{
	while(1)
	{
		pushBox();
	}
}

void pushBox()
{		
		int base_speed = 500;
		int max_speed = 800;
		
		int front_right_prox = e_get_calibrated_prox( 0 );
		int front_left_prox = e_get_calibrated_prox( 7 );
		
		int left_speed = base_speed;
		int right_speed = base_speed;
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
			
		int total = front_right_prox + front_left_prox;
		
		left_speed = normalise_speed( left_speed );
		right_speed = normalise_speed( right_speed );
		
		set_wheel_speeds( left_speed, right_speed );

}
