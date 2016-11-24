#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>

#include "uart/e_uart_char.h"

//char buffer that will hold the message that will be transferred
char uartbuffer[100];

void wait(long num) {
	long i;
	for(i=0;i<num;i++);
}

int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}


//function to report a value via uart1 comms given a piece of str (desc) and a value
void reportValue(char* title, int value)
{
	sprintf(uartbuffer, "(%s - %d), ", title, value);
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
}
