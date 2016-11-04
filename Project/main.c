#include "../library/motor_led/e_epuck_ports.h"
#include "../library/motor_led/e_init_port.h"
#include "../library/motor_led/e_motors.h"

int main(void)
{
	e_init_motors();
	
	e_set_speed_left(500);
	e_set_speed_right(500);	
	
	while(1) 
	{
		if (e_get_steps_left() > 1000)
		{
			e_set_speed_left(0);
			e_set_speed_right(0);
		}

		Wait(15000);
	}
}

void Wait(int steps)
{
	int i = 0;

	for(i=0; i<steps; i++)
	{
 		asm("NOP");
	}
}