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
	int xCurrent[10] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
	int xCurrentSmooth[10] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
	int xAcc = 0;
	int xAccSmooth = 0;
	int	xAccPrev, xAccSum;
	while(1)
	{
		//calculate moving average value
		e_get_acc(&x, &y, &z);
		int j;
		for (j = 0; j < 9; j++)
		{ //move array items to make space for new value
			xCurrent[j] = xCurrent[j+1];
			xCurrentSmooth[j] = xCurrent[j+1];
		}
		xCurrent[9] = x;
		xAccSum = 0;
		for (j = 0; j < 10; j++)
		{//sum values and calculate factor 10 moving average
			xAccSum += xCurrent[j];
		}
		xAcc = floor(xAccSum/10);
		
		xCurrentSmooth[9] = xAcc;
		xAccSum = 0;
		for (j = 0; j < 10; j++)
		{//sum factor 10 values and calculate factor 100 (effective) moving average
			xAccSum += xCurrentSmooth[j];
		}
		xAccPrev = xAccSmooth;
		xAccSmooth = xAccSum/10;
		
		int diff = xAccPrev - xAccSmooth;
		if (diff > 100)
		{
			LED0 = 1;
			reportAccel(xAccSmooth, 1);
		}
		else
		{
			LED0 = 0;
			reportAccel(xAccSmooth, 0);
		}
		long i;
		for(i=0; i<10000; i++) { asm("nop"); }
	}
}
