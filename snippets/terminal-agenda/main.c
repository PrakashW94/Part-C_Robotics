#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "uart/e_uart_char.h"

#include "stdio.h"

//char buffer that will hold the message that will be transferred
char uartbuffer[100];

//This function gets the position that the selector is in. 
int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

//function to report a value via uart1 comms given a piece of str (desc) and a value
void reportValue(char* title, int value)
{
	sprintf(uartbuffer, "(%s - %d), ", title, value);
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
}

int main() 
{
	e_init_port();
	e_init_uart1();
	
	int selector;
	selector=getSelector();
	
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
			reportValue("Selector:", selector);
			break;
		}
		case 2: 
		{
			LED2 = 1;
			reportValue("Selector:", selector);
			break;
		}
		case 3: 
		{
			LED3 = 1;
			reportValue("Selector:", selector);
			break;
		}
		case 4: 
		{
			LED4 = 1;
			reportValue("Selector:", selector);
			break;
		}
		case 5: 
		{
			LED5 = 1;
			reportValue("Selector:", selector);
			break;
		}
		case 6: 
		{
			LED6 = 1;
			reportValue("Selector:", selector);
			break;
		}
		case 7: 
		{
			LED7 = 1;
			reportValue("Selector:", selector);
			break;
		}
		default:
		{
			reportValue("Selector:", selector);
			break;
		}
	}
	while(1);
}
