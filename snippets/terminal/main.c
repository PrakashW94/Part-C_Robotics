#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "uart/e_uart_char.h"

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

int main() 
{
	e_init_port();
	e_init_uart1();
	
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
		default:
		{
			echosel();
			break;
		}
	}
	while(1);

}
