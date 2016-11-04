#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "uart/e_uart_char.h"
#include "a_d/e_prox.h"
#include "motor_led/e_motors.h"
#include "p30f6014A.h"

#include "stdio.h"
#include "math.h"

//char buffer that will hold the message that will be transferred
char uartbuffer[100];

//This function gets the position that the selector is in. 
int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void echosel()
{
	sprintf(uartbuffer,"The number of the selector is: %d ", getselector());	
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
}

void testprox()
{
	long i;
	int value;
	value = e_get_prox(0);
	while(1)
	{
		value = e_get_prox(0);		
		sprintf(uartbuffer, "%d, ", value);
		e_send_uart1_char(uartbuffer, strlen(uartbuffer));
		for(i=0; i<100000; i++) { asm("nop"); }	
	}
}

void move()
{
	long i, j;
	j=0;
	int value0, value1, prox0;
	e_set_speed_left(j); 
	e_set_speed_right(j); 
	while(1)
	{
		value0 = e_get_prox(0);
		value1 = e_get_prox(7);
		prox0 = floor((value0+value1)/2);
		
		sprintf(uartbuffer, "%d-%d (%d), ", value0, value1, prox0);
		e_send_uart1_char(uartbuffer, strlen(uartbuffer));
		
		switch(prox0)
		{
			case 1 ... 500:
			{
				e_led_clear();
				j = 500;
				e_set_speed_left(j);
				e_set_speed_right(j);
				break;
			}
			case 501 ... 2000:
			{
				e_led_clear();
				LED0 = 1;
				j = 150;
				e_set_speed_left(j); 
				e_set_speed_right(j);
				break;
			}
			case 2001 ... 4000:
			{
				e_led_clear();
				e_set_led(8, 2);
				for(i=0; i<500000; i++) { asm("nop"); }
				j = 500;
				e_set_speed_left(j); 
				e_set_speed_right(-j);
				break;
			}
		}
		
		/*if ((value0 > 500) || (value1 > 500))
		{
			LED0 = 1;			
			j=0;
			e_set_speed_left(j);
			e_set_speed_right(j);
		}
		else
		{
			LED0 = 0;
			j=500;
			e_set_speed_left(j);
			e_set_speed_right(j);
		}*/
		for(i=0; i<25000; i++) { asm("nop"); }
	}
}

int main() 
{
	e_init_port();
	e_init_uart1();
	e_init_prox();
	e_init_motors();
	
	int selector;
	selector=getselector();
	
	switch(selector)
	{
		case 0: 
		{
			//by keeping this blank you can easily switch the selector to 0 
			//and you’ll always be able to idle your epuck.
			LED0 = 1;
			break;
		}
		case 1: 
		{
			LED1 = 1;
			echosel();
			break;
		}
		case 2: 
		{
			LED2 = 1;
			echosel();
			break;
		}
		case 3: 
		{
			LED3 = 1;
			echosel();
			break;
		}
		case 4:
		{
			LED4 = 1;
			echosel();
			break;
		}
		case 5: 
		{
			LED5 = 1;
			echosel();
			break;
		}
		case 6: 
		{
			LED6 = 1;
			echosel();
			break;
		}
		case 7: 
		{
			LED7 = 1;
			echosel();
			break;
		}
		case 8:
		{
			testprox();
			break;
		}
		case 9:
		{
			move();
			break;
		}
		default:
		{
			echosel();
			break;
		}
	}
	while(1);

}
