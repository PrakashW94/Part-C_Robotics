#include "p30f6014A.h"
#include "stdio.h"
#include "string.h"

#include "uart/e_uart_char.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "motor_led/advance_one_timer/e_agenda.h"

#include "imageCapture.h"
#include "findRed.h"

int getSelector();

int main(void)
{
	e_init_port();
	e_init_uart1();
	e_init_motors();
	
	if (getSelector() == 1)
	{
		findRed();
	}
	while(1) {}
}

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}