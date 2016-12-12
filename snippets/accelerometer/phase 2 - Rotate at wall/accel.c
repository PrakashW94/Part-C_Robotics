#include "p30F6014A.h"
#include <math.h>
#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "motor_led/e_led.h"
#include "a_d/e_accelerometer.h"

void reportAccel(int x, int hit)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "%d, %d\r\n", x, hit);
	int length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}

void accel()
{
	int x, y, z;
	int lastx = 1900;
	while(1)
	{
		e_get_acc(&x, &y, &z);
		int diff = lastx - x;
		if (diff > 500)
		{
			LED0 = 1;
			reportAccel(x, 1);
		}
		else
		{
			LED0 = 0;
			reportAccel(x, 0);
		}
		lastx = x;
		long i;
		for(i=0; i<500000; i++) { asm("nop"); }
	}
}
