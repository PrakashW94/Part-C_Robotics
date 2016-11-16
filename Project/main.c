#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "a_d/e_prox.h"
#include "util/motor_control.h"

void wait(int steps);

int main(void)
{
	e_init_motors();
	e_init_prox();

	moveToPoint(500, 500);
	moveToPoint(-500, -500);
	
	e_set_speed_left( 0 );
	e_set_speed_right( 0 );

	/*int value;
	while(1) 
	{ 
		long i;
 		value = e_get_prox(0);
 		if(value > 1000)
 			LED0 = 1;
 		else
			LED0 = 0;

		wait(100000);		
	}*/
	while(1) {}
}

void wait(int steps)
{
	int i = 0;

	for(i=0; i<steps; i++)
	{
 		asm("NOP");
	}
}

