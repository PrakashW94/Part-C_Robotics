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

/*
* The lower this value, the more potential force is
* generated from the objects around
*/
#define PROXSCALING_CURIOUS 2

/*
* Increase this value to increase the base speed of the robot.
* This speed is constant. 
*/
#define BASICSPEED 400

/*
* Change this value to increase the sensitivity of the
* decelleration when near an object.
*/
#define DECELLERATE_PER_SCALE 200

void curiousity()
{	
	
	// Iterator Variables
	int i, s, m;
	
	/* Potential Force per side of the vehicle.
	* 0 = Potential Force to apply to left wheel.
	* 1 = Potential Force to apply to right wheel.
	*/
	long potential[2];
	
	/* Speed per side of the vehicle.
	* 0 = Speed to apply to left wheel
	* 1 = Speed to apply to right wheel
	*/
	int speed[2];
	
	// Match LED to proximity sensor
	int led_array[8] = { 9, 1, 2, 3, 5, 6, 7, 0};

	/*
	* A 2-D array to scale the potential forces of each sensor.
	* 0 = Potential force on left wheel
	* 1 = Potential force on right wheel
	*
	* Positive value represents an attract force.
	* Negative value represents a repelling force.
	*/
	int matrix_prox[2][8] =
		{{0,0,0,0,0,-8,-4,-8},
		{-8,-4,-8,0,0,0,0,0}};

	int max_proximity = -1;

	// For each side of the robot 
	for( m = 0; m < 2; m++)
	{	
		potential[m] = 0;

		// For each proximity sensor
		for( s = 0; s < 8 ; s++ )
		{	
			int prox_value = e_get_calibrated_prox( s );
			
			// Turn led on/off if the sensor is triggered/not triggered
			int led_on = ( prox_value > 50 ) ? 1 : 0;
			e_set_led( led_array[s], led_on );
			
			int potential_modifier = matrix_prox[m][s];

			// If modifier is 0, we do not care about that sensor.
			if( potential_modifier != 0 )
			{
				if( max_proximity ==  -1 )
					max_proximity = prox_value;
				else
					max_proximity = fmax( max_proximity, prox_value );
			}
		
			int prox_potential = prox_value * potential_modifier;
			potential[m] += prox_potential;
		}

		speed[m] = ( potential[m] / PROXSCALING_CURIOUS ) + BASICSPEED;
	}	

	float scale_raise = ( ( max_proximity > 1000 ? 1000 : max_proximity ) / DECELLERATE_PER_SCALE );
	scale_raise = scale_raise + 1.0; 
	
	// Scale up each wheel speed
	for( m = 0; m < 2 ; m++ )
	{
		speed[m] = speed[m] / scale_raise;
		
		// TODO:: if speed is between 0 and 50, or 0 and -50, then change to 50/-50		

		if( speed[m] > 1000 )
			speed[m] = 1000;
		else if( speed[m] < - 1000 )
		{
			speed[m] = -1000;
		}
	}	

	e_set_speed_right( speed[0] );
	e_set_speed_left( speed[1] );
}

/*
* Uses all proximity sensors except the two back.
*
* As the sensory values increase at the front, the robot should slow down.
*
* The robot should then be repelled by the object once close enough and speed up as it moves away.
*/
void initCuriousity()
{
	// Initialise components 
	e_init_port();
	e_init_motors();
	e_init_uart1();
	
	e_init_ad_scan(ALL_ADC);
	e_calibrate_ir();
	
	// Register agendas
  	e_activate_agenda( curiousity, 650 );

	// Start processing
	e_start_agendas_processing();
	
	while(1);
}