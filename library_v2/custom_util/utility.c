#include <string.h>
#include <stdio.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/e_led.h>

#include "uart/e_uart_char.h"

void wait(long num) {
	long i;
	for(i=0;i<num;i++);
}

int getselector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void reportValue(char* title, int value)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "(%s - %d)\r\n", title, value);
	int length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}

void reportXY(int x, int y, int a, int mDist)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "%d, %d, %d, %d\r\n", x, y, a, mDist);
	int length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}

//function to turn given leds on/off given an array leds (0-7) and the number of leds in the array
void setLeds(int leds[], int noOfLeds)
{
	int i;
	for (i = 0; i < noOfLeds; i++)
	{
		e_set_led(leds[i], 2);
	}
}
