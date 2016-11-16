#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "util/motor_control.h"

void wait(int steps);

int main(void)
{
	e_init_motors();

	moveToPoint(-500, -500);	

	e_set_speed_left( 0 );
	e_set_speed_right( 0 );

	while(1) { }
}

void wait(int steps)
{
	int i = 0;

	for(i=0; i<steps; i++)
	{
 		asm("NOP");
	}
}

