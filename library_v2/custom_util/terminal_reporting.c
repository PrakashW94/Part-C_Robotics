#include "uart/e_uart_char.h"

void reportValue(char* title, int value)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "(%s - %d)\r\n", title, value);
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
	//while(e_uart1_sending()){}
}