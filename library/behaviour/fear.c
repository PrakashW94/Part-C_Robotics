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


void fear()
{
	// Iterator Variables
	int m , s;

	// The base speed of the robot. This is the speed of the wheels when there is no sensory input.
	int base_speed = 300;

	// The sensory value when the robot is too close. This will cause the wheel speeds to the opposite sign ( Positive <-> Negative ).
	int too_close = 2000;
	
	// The maximum speed of the wheels
	int max_speed = 1000;
	
	// The minimum speed of the wheels
	// In the case of the epuck, this value is the fastest speed the wheels can go in reverse.
	int min_speed = -1000;
	

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
							{3,3,1},
							{3,3,1}
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
		
			//	sprintf(uartbuffer, "Wheel %d, Prox %d: %d \r\n", m, sensors[m][s], prox_value);
			//	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
		
			int weight = weights[m][s];
			sensor_intensity[m] += (prox_value * weight);
		}
		sensor_intensity[m] += base_speed;
	}
	
	int left_sensor = sensor_intensity[0];
	int right_sensor = sensor_intensity[1];
	

	// If robot is too close, we should invert the sign of the values to allow robot to move back and continue the behaviour.
	int left_speed =  right_sensor < too_close ? left_sensor : -left_sensor;
	int right_speed = left_sensor < too_close ? right_sensor : -right_sensor;
	
	// Normalise the speed.
    if( left_speed > max_speed ) 
		left_speed = max_speed;
    if( left_speed < min_speed )
		left_speed = min_speed;
    if( right_speed > max_speed )
		right_speed = max_speed;
    if( right_speed < min_speed ) 
		right_speed = min_speed;

	e_set_speed_left( left_speed );
	e_set_speed_right( right_speed );
}


/*
* Uses the two front proximity sensors and the motors corresponding to the opposite side of each sensor
* to create the fear braitenberg behaviour.
* 
* As the sensory value for a sensor on one side increases, the motor for the side of the robot
* will increase in speed.
*/
void initFear()
{
	// Initialise components 
	e_init_port();
	e_init_motors();
	e_init_uart1();
	
	e_init_ad_scan();
	e_calibrate_ir();
	
	// Register agendas
  	e_activate_agenda( fear, 650 );

	// Start processing
	e_start_agendas_processing();
	
	while(1);
}
