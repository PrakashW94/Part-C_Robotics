#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "a_d/e_prox.h"
#include "custom_util/motor_control.h"

void wait(int steps);

int main(void)
{
	e_init_motors();
	e_init_prox();

	setGoal(0, 1000);
	moveToPoint(0, 500);
	moveToPoint(500, 500);
	moveToPoint(500, 0);
	moveToPoint(0, 0);
	moveToGoal();

	e_set_speed_left( 0 );
	e_set_speed_right( 0 );

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

