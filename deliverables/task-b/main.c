#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "uart/e_uart_char.h"
#include "a_d/e_prox.h"
#include "custom_util/motor_control.h"
#include "motor_led/e_led.h"
#include "a_d/e_prox.h"
#include <string.h>
#include "custom_util/constants.c"
#include "utility/utility.h"

#include <math.h>

#define threshold 300

void wait(int steps);
int getSelector();

int updateDetector()
{
	int d = 0;

	// FRONT Detected
	if (e_get_prox(0) > threshold
		&& e_get_prox(7) > threshold)
	{
		d = d + 1;
	}

	// RIGHT SIDE Detected
	if (e_get_prox(2) > threshold)
	{
		d = d + 2;
	}

	// LEFT SIDE Detected
	if (e_get_prox(5) > threshold)
	{
		d = d + 4;
	}

	// Don't do this too often
	//wait(1000000);

	return d;
}

void moveSensory()
{
	int j;
	int detector;
	
	while (1)
	{
		detector = updateDetector();
		switch(detector)
		{
			case 0: 
			{
				e_led_clear();
				//e_set_speed_left(j);
				//e_set_speed_right(j);
				//wait(2000000);
				break;
			}
			case 1: 
			{
				LED7 = 1;
				rotateAntiClockwiseDegrees(10);
				//wait(2000000);
				break;
			}
			case 2: 
			{
				LED7 = 1;
				LED0 = 1;
				rotateAntiClockwiseDegrees(10);
				//wait(2000000);
				break;
			}
			case 4: 
			{
				LED0 = 1; 
				LED1 = 1;
				rotateClockwiseDegrees(10);
				//wait(2000000);				
				break;
			}
			case 8: 
			{
				LED1 = 1; 
				rotateClockwiseDegrees(10);
				//wait(2000000);
				break;
			}
			default: 
			{
				break;
			}
		}

		wait(1000000);
	}
}

int main(void)
{
	e_init_motors();
	e_init_prox();
	e_init_uart1();

	int selector = getSelector();
	if (selector == 1)
	{		
		setGoal(0, 2200);
		moveToGoal();		
		
		e_set_speed_left( 0 );
		e_set_speed_right( 0 );		
	}
	else if(selector == 2 ){
		
		setGoal(0, 1500);
		moveToPoint(0, 200);
		move(400, 0);
		move(-400, 0);
		moveToGoal();
	
		e_set_speed_left( 0 );
		e_set_speed_right( 0 );
	}
	else if (selector == 3)
	{
		while(1) 
		{ 
			int val = updateDetector();
			reportValue("Detector", val);
			wait(10000000);
		}
	}

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

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}