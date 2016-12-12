#include "p30f6014A.h"
#include "stdio.h"
#include "string.h"

#include "uart/e_uart_char.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "motor_led/advance_one_timer/e_agenda.h"

#include "a_d/advance_ad_scan/e_ad_conv.h"
#include "a_d/advance_ad_scan/e_prox.h"

#include "runwallfollow.h"

#include "findRed.h"
#include "findGreen.h"
#include "avoidBox.h"

int getSelector();

int main(void)
{
	e_init_uart1();
	e_init_port();
	e_init_ad_scan(ALL_ADC);
	e_init_motors();
	e_start_agendas_processing();	
	e_calibrate_ir();

	if (getSelector() == 1)
	{
		initRed();
		while (1) {findRed(); }
	}
	else if (getSelector() == 2)
	{
		initGreen();
		while (1) { findGreen(); }
	}
	else if (getSelector() == 3)
	{
		initGreen();

		//e_set_speed_left(500);
		//e_set_speed_right(500);
		//e_start_agendas_processing();

		while (1)
		{
			int val = findGreen();
			reportValue("val", val);
			if (val)
			{						
				avoidBox();
			}
		}
	}
	else if (getSelector() == 4)
	{
		run_wallfollow();
	}
	while(1) {}
}

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}