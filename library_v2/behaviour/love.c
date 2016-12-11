#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"

#include "custom_util/utility.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "ircom/e_ad_conv.h"

#include "uart/e_uart_char.h"

#include "stdio.h"
#include "string.h"
#include "p30f6014a.h"


void love()
{		
	// Iterator Variables
	int m , s;

	// The base speed of the robot. This is the speed of the wheels when there is no sensory input.
	int base_speed = 700;

	// The sensory value when the robot is too close. This will cause the wheel speeds to the opposite sign ( Positive <-> Negative ).
	int too_close = 4000;
	
	// The maximum speed of the wheels
	int max_speed = 1000;
	
	// The minimum speed of the wheels
	// In the case of the epuck, this value is the fastest speed the wheels can go in reverse.
	int min_speed = 0;
	

	/*
	* Sensor ids to use in order of front to back of the robot.
	* First array is left sensors.
	* Second array is right sensors.
	*/
	int sensors[2][3] = {
							{7,6,5},
							{0,1,2}
						};		
	
	/*
	* The weight of each sensor ( how much the sensor affects the wheel speed )
	* Each sensor values' position corresponds to the position in the 'sensors' array definition.
	* The 'sensors' and 'weights' array must be the same size.
	*/
	int weights[2][3] = {
							{3,2,1},
							{3,2,1}
						};	
			
	// Starting sensor intensity.			
	int sensor_intensity[2] = {0,0};


	// For each side
	for( m=0; m < (sizeof(sensors) / sizeof(sensors[0])); m++)
	{	

		// For each sensor on that side
		for( s=0; s < (sizeof(sensors[m]) / sizeof(sensors[m][0]) ); s++ )
		{
			int prox_value = e_get_calibrated_prox( sensors[m][s] );
			int weight = weights[m][s];
			sensor_intensity[m] += (prox_value*weight);
		}
		sensor_intensity[m] = base_speed - sensor_intensity[m];
	}
	
	int left_sensor = sensor_intensity[0];
	int right_sensor = sensor_intensity[1];

	// If sensors are too close, then invert so that the robot can move back slightly and continue love from expected distance.
	int left_speed =  right_sensor < too_close ? left_sensor : -left_sensor;
	int right_speed = left_sensor < too_close ? right_sensor : -right_sensor;
	
    if( left_speed > max_speed ) 
		left_speed = max_speed;
    if( left_speed < min_speed )
		left_speed = min_speed;
    if( right_speed > max_speed )
		right_speed = max_speed;
    if( right_speed < min_speed ) 
		right_speed = min_speed;

	// Perform love.
	if( left_speed == min_speed &&
		right_speed == min_speed )
	{
		BODY_LED = 1;
	}
	else
	{
		BODY_LED = 0;
	}

	e_set_speed_left( left_speed );
	e_set_speed_right( right_speed );
}

/*
* Uses all proximity sensors except the two back.
*
* As the sensory values increase, the robot is `attracted` to the object
* causing the increase. The robot also slows down as it gets closer.
*
* After getting close enough, the robot will stop and perform a signal of `love`.
*/
void initLove()
{
	// Initialise components 
	e_init_port();
	e_init_motors();
	e_init_uart1();
	
	e_init_ad_scan();
	e_calibrate_ir();
	
	// Register agendas
  	e_activate_agenda( love, 650 );

	// Start processing
	e_start_agendas_processing();
	
	while(1);
}
