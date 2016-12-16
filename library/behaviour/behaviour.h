/*
* Shared variables of behaviours should go here.
*
*
*/
#ifndef _BEHAVIOUR
#define _BEHAVIOUR

// Iterator Variables
extern int i, s, m;

/* 
* Potential Force per side of the vehicle.
* 0 = Potential Force to apply to left wheel.
* 1 = Potential Force to apply to right wheel.
*/
extern long potential[2];


/* 
* Speed per side of the vehicle.
* 0 = Speed to apply to left wheel
* 1 = Speed to apply to right wheel
*/
extern int speed[2];


/* 
* A 2-D array to scale the potential forces of each sensor.
* 0 = Potential force on left wheel
* 1 = Potential force on right wheel
*
* Positive value represents an attract force.
* Negative value represents a repelling force.
*/
extern int matrix_prox[2][8];


// Match LED to proximity sensor
int led_array[8] = { 9, 1, 2, 3, 5, 6, 7, 0};



#endif

