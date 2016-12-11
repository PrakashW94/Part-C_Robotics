#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"
#include "constants.c"

#include "phase2.h"
#include "agendacontrol.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

//main code to emulate behaiviour before turning
int main()
{
	e_init_port();
	e_init_prox();
	e_init_motors();
	e_init_uart1();
	
	int selector = getSelector();
	if (selector > 0)
	{
		e_set_speed_left(300);
		e_set_speed_right(300);
		
		e_start_agendas_processing();
		
		controlAgenda();
		while(1){}
	}	
	else
	{
		e_set_led(0,1);
		while(1);
	}
}
