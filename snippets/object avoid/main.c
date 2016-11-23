#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "motor_led/e_led.h"
#include "a_d/e_prox.h"
#include "uart/e_uart_char.h"
#include <string.h>

#include <math.h>

#define threshold 300
int getSelector();
void wait(int steps);

//function to report a value via uart1 comms given a piece of str (desc) and a value
void reportValue(char* title, int value)
{
	char uartbuffer[100];
	int length;
	sprintf(uartbuffer, "(%s - %d)\r\n", title, value);
	length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}

int updateDetector()
{
	int d = 0;
	if(e_get_prox(6) > threshold)
	{
		d = d + 1*1;
	}
	
	if(e_get_prox(7) > threshold)
	{
		d = d + 1*2;
	}
	
	if(e_get_prox(0) > threshold)
	{
		d = d + 1*4;
	}
	
	if(e_get_prox(1) > threshold)
	{
		d = d + 1*8;
	}
	if (d)
		return d;
	else
		return 0;
}

void moveSensory()
{
	int j;
	int detector;
	
	j = 200;
	e_set_speed_left(j);
	e_set_speed_right(j);
	while (1)
	{
		detector = updateDetector();
		switch(detector)
		{
			case 0: 
			{
				e_led_clear();
				e_set_speed_left(j);
				e_set_speed_right(j);
				//wait(2000000);
				break;
			}
			case 1: 
			{
				LED7 = 1;
				e_set_speed_left(-3*j);
				e_set_speed_right(j);
				//wait(2000000);
				break;
			}
			case 2: 
			{
				LED7 = 1;
				LED0 = 1;
				e_set_speed_left(-5*j);
				e_set_speed_right(j);
				//wait(2000000);
				break;
			}
			case 4: 
			{
				LED0 = 1; 
				LED1 = 1;
				e_set_speed_left(-j);
				e_set_speed_right(5*j);
				//wait(2000000);				
				break;
			}
			case 8: 
			{
				LED1 = 1; 
				e_set_speed_left(-j);
				e_set_speed_right(3*j);
				//wait(2000000);
				break;
			}
			default: 
			{
				break;
			}
		}
		reportValue("Detector", detector);
		wait(1000000);
	}
}

int main(void)
{
	e_init_motors();
	e_init_prox();
	e_init_uart1();
	
	int selector = getSelector();
	if (selector > 0)
	{
		moveSensory();		
	}
	while(1){}
}

void wait(int steps)
{
	int i = 0;

	for(i=0; i<steps; i++)
	{
 		asm("NOP");
	}
}

//This function gets the position that the selector is in. 
int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}
