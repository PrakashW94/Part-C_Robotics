#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"

#include "custom_util/constants.c"
#include "custom_util/motor_control.h"
#include "custom_util/math.c"

#include "motor_led/e_motors.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "a_d/advance_ad_scan/e_ad_conv.h"
#include "a_d/advance_ad_scan/e_prox.h"

#include "uart/e_uart_char.h"

#include "stdio.h"
#include "string.h"
#include "p30f6014a.h"

#define PROXSCALING_FOLLOW 20
#define PROXSCALING_SHOCK 4
#define BASICSPEED 200

//char buffer that will hold the message that will be transferred
char uartbuffer[100];

int i, s, m;
long potential[2];
int speed[2];
long ProxSensOffBuf[8];
int ui_lin = 0;

int factor_array[2][8] =
	{{-10,-30,15,0,0,-15,30,10},
	{10,30,-15,0,0,15,-30,-10}};

int matrix_prox[2][8] =
	{{8,4,2,0,0,-4,-8,-16},
	{-16,-8,-4,0,0,2,4,8}};

const SCALE = 0.5;


void aggression_v2()
{	
//	int max_proximity = -1;

	// For each side of the robot 
	for( m = 0; m < 2; m++)
	{	
		potential[m] = 0;
		// For each proximity sensor
		for( s = 0; s < 8 ; s++ )
		{	
			// Multiple proximity value by its given weight on that side and add to total
			int prox_value = e_get_calibrated_prox( s );
	
		//	if( max_proximity ==  -1 )
		//		max_proximity = prox_value;
			
		//	max_proximity = fmax( max_proximity, prox_value );

			potential[m] += matrix_prox[m][s] * prox_value;
		}
	//	sprintf(uartbuffer, "%d\r\n", max_proximity);
	//	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
	//	float scale_raise = ( ( max_proximity > 1000 ? 1000 : max_proximity ) / 1000 ) * 50;
	//	float scale =  scale_raise + 1;
		 
	//	speed[m] = ( potential[m] / PROXSCALING_SHOCK + BASICSPEED ) * scale ;
		speed[m] = ( potential[m] / PROXSCALING_SHOCK + BASICSPEED );
	
	}	
		
	if((speed[1] < 50 && speed[1] > -50)
		&& (speed[0] < 50 && speed[0] > -50)) {
		speed[1] = speed[1] * 20;
		speed[0] = speed[0] * 20;
	}

	if (speed[1] > 1000)
		speed[1] = 1000;
	else if (speed[1] < -1000 )
		speed[1] = -1000;

	if (speed[0] > 1000)
		speed[0] = 1000;
	else if (speed[0] < -1000 )
		speed[0] = -1000;		
		
	
	e_set_speed_right( speed[0] );
	e_set_speed_left( speed[1] );
}

void aggression()
{
	LED1 = ( LED1 == 1 ? 0 : 1 );
	int value_left;
	int value_right;

  	value_left = e_get_calibrated_prox(0);
  	value_right = e_get_calibrated_prox(7);

//	reportValue( "Prox value left", value_left );
//	reportValue( "Prox value right", value_right );

	if( value_left > 100 || value_right > 100 )
	{
 		LED0 = 1;
		set_speed( LEFT, value_left * 2 );
		set_speed( RIGHT, value_right * 2 );
 	}
	else
	{
  		LED0 = 0;
		set_speed( BOTH, 100 );
	}
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
	// Initialise components 
	e_init_port();
	e_init_motors();
	
	e_init_ad_scan(ALL_ADC);
	e_calibrate_ir();
	
	// Register agendas
  	e_activate_agenda( aggression_v2, 650 );

	// Start processing
	e_start_agendas_processing();
	
	while(1);
}