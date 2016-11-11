#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "util/motor_control.h"

#include "uart/e_uart_char.h"
#include "stdio.h"
#include <string.h>

void wait(int steps);

char uartbuffer[100];

int main(void)
{
	e_init_port();
	
	int selector;
	selector = getSelector();
	
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
			//Jacks original prog
			e_init_motors();

			moveToPoint(-500, -500);	

			e_set_speed_left( 0 );
			e_set_speed_right( 0 );
			break;
		}
		case 2:
		{
			//waits for user to enter a word (via terminal) and repeats the word back to user
			e_init_uart1();
			
			char car;
			char word[100];
			int i, len;

			while(1)
			{
				e_send_uart1_char("Give a character:\r\n", 19);
				
				// do nothing while the text is not sent and while nothing is comming from the user
				while(e_uart1_sending() || !e_ischar_uart1()) {}	
				
				//wait for full word to enter buffer
				wait(10000); 
				
				len = e_ischar_uart1();
				e_send_uart1_char("You have written: ", 18);
				for (i = 0; i < len; i++)
				{
					e_getchar_uart1(&car);	// read the character entered.
					e_send_uart1_char(&car, 1);		//... and resend him to uart.
				}				
				e_send_uart1_char("\r\n\r\n",4);
			}
		}
		default:
		{
			
		}
	}
	
	while(1) { }
}

void wait(int steps)
{
	int i = 0;

	for(i=0; i<steps; i++)
	{
 		asm("NOP");
	}
}