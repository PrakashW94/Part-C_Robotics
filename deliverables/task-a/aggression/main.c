#include "behaviour/aggression.h"
#include "motor_led/e_epuck_ports.h"


int getSelector() 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int main()
{
	e_init_port();
	e_init_uart1();	
	e_init_prox();
	
	int selector = getSelector();

	switch( selector ){
		case 0:
			initAggression();
		default:
			BODY_LED = 1;
			while(1);
			break;
	} 

	return 0;	
}