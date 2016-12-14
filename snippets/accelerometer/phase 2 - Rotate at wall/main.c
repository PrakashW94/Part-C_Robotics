#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "a_d/e_accelerometer.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"
#include "constants.c"

#include "phase2.h"
#include "accel.h"

#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

int getSelector() 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

//main code to emulate behaivour before turning
int main()
{
	e_init_port();
	e_init_prox();
	e_init_motors();
	e_init_uart1();
	e_init_acc();
	
	int selector = getSelector();
	if (selector > 0)
	{
		e_set_speed_left(1000);
		e_set_speed_right(1000);
		accel();
		/*while(1)
		{
			int frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
			reportValue("frontProx", frontProx);
			if (frontProx > 600)
			{
				rotateAtWall();
			}
			wait(delayTimer);
		}*/
	}	
	else
	{
		e_set_led(0,1);
		while(1);
	}
}
