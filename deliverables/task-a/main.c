#include "behaviour/aggression.h"
#include "behaviour/curious.h"
#include "behaviour/fear.h"
#include "behaviour/love.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_motors.h"
#include "a_d/e_prox.h"

#include "custom_util/motor_control.h"
#include "custom_util/constants.c"

int getSelector() 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int main()
{	
	int selector = getSelector();
	
	e_init_port();
	e_init_motors();
	
	// Selectors go from 0-14
	switch( selector ){
		case 0:
			initFear();
			break;
		case 1:
			initCuriousity();
			break;
		case 2:
			initAggression();
			break;
		case 3:
			initLove();
			break;
		default:
			// Indicated an unregistered selector.
			BODY_LED = 1;
			while(1);
			break;
	} 
	

	return 0;	
}