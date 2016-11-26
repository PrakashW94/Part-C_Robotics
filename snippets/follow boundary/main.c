#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"
#include "avoid_objects.h"
#include "constants.c"

#include "pathfinding.h"

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int main()
{
	e_init_port();
	e_init_prox();
	e_init_motors();
	e_init_uart1();
	
	int selector = getSelector();
	if (selector > 0)
	{
		pathfinder();
		/*
		updateLeft(s);
		updateRight(s);
		int frontwide[] = {6, 7, 0, 1};
		int frontProx = getProx(frontwide, 4);
		while(1)
		{	
			reportValue("front", frontProx);
			if (frontProx > 400)
			{
				reportValue("Entering Main", frontProx);
				avoidBoundary();
			}
			updateLeft(s);
			updateRight(s);
			frontProx = getProx(frontwide, 4);
			wait(delayTimer);
		}
		*/
	}
	else
	{
		e_set_led(8,1);
		while(1);
	}
}
