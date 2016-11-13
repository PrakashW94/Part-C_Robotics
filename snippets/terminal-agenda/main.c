#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "uart/e_uart_char.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

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

void agendaStuff()
{
	e_activate_agenda(flow_led, 5000); //blink with 500ms
	e_set_speed(500,0);
	e_start_agendas_processing();
	while(1){}
}

//function to get a value for proximity based on (IR7 + IR0)/2 and report it via uart1 comms
int updateProx(int report)
{
	int value0, value1, prox;
	
	value0 = e_get_prox(0);
	value1 = e_get_prox(7);
	prox = floor((value0+value1)/2);
	
	if (report > 0)
	{
		reportValue("prox", prox);
	}
	
	return prox;
}

int main() 
{
	e_init_port();
	e_init_uart1();
	e_init_motors();
	
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
			agendaStuff();
			break;
		}
		case 2: 
		{
			LED2 = 1;
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
