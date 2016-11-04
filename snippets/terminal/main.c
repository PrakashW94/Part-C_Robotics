#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "uart/e_uart_char.h"
#include "a_d/e_prox.h"
#include "p30f6014A.h"

#include "stdio.h"

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

int main() 
{
	e_init_port();
	e_init_uart1();
	e_init_prox();
	
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
			//LED8 = 1;
			testprox();
		}
		default:
		{
			echosel();
			break;
		}
	}
	while(1);

}
