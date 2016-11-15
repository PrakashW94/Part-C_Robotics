#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"

#include "util/constants.c"
#include "util/motor_control.h"

#include "motor_led/e_motors.h"

#include "uart/e_uart_char.h"

#include "stdio.h"
#include "math.h"

#define SCALE 2	

#define LEFT 0
#define RIGHT 1
#define BOTH 2

//char buffer that will hold the message that will be transferred
char uartbuffer[100];

void reportValue(char* title, int value)
{
	sprintf(uartbuffer, "(%s - %d)", title, value);
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
}

/*
* Uses the two front proximity sensors and the motors corresponding to the opposite side of each sensot
* to create the aggressive braitenberg behaviour.
* 
* As the sensory value for a sensor on one side increases, the motor for the opposite side of the robot
* will increase in speed.
*/
void initAggression()
{
  	int value_left;
	int value_right;
	int value_avg;
	
	while(1)
	{
		e_set_speed_left( 100 );
		e_set_speed_right( 100 );
	}

  /*	while(1)
  	{
  		long i;
			
  		value_left = e_get_prox(0);
  		value_right = e_get_prox(7);

		reportValue( "Prox value left", value_left );
		reportValue( "Prox value right", value_right );

		if( value_left > 100 || value_right > 100 )
		{
  			LED0 = 1;
			set_speed( LEFT, value_left * SCALE );
			set_speed( RIGHT, value_right * SCALE );
  		}
		else
		{
  			LED0 = 0;
			
			set_speed( BOTH, 100 );
		}

  		for(i=0; i<100000; i++) { asm("nop"); }
  	}*/
}