#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "custom_util/utility.h"

#include "phase2.h"

int status = 0;

void controlAgenda()
{
	reportValue("Current Status", status);
	switch(status)
	{
		case 0: 
		{
			e_activate_agenda(moveAndSense, 1000);
			e_activate_agenda(e_blink_led, 1000);
			break;
		}
		case 1:
		{
			e_pause_agenda(moveAndSense);
			rotateAtWall();
			break;
		}
		case 2:
		{
			e_restart_agenda(moveAndSense);
			break;
		}
	}
}
