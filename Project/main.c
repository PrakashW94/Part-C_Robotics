#include "../library/motor_led/e_epuck_ports.h"
#include "../library/motor_led/e_init_port.h"
#include "../library/motor_led/e_motors.h"

int main(void)
{
	e_init_motors();
	e_set_speed_left(500); //go forward on half speed
	e_set_speed_right(-500); //go backward on half speed
	while(1) {}
}