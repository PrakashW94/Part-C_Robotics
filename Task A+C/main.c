#include "behaviour/aggression.h"
#include "behaviour/curious.h"
#include "behaviour/fear.h"
#include "behaviour/love.h"

#include "btcom/btcom.h"

#include "uart/e_uart_char.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "ircom/ircom.h"
#include "ircom/ircomUtil.h" 

#include "ircom/emitter.h"
#include "ircom/receiver.h"

#include "high_level/global.h"
#include "high_level/packet.h"
#include "high_level/init.h"
#include "high_level/traverse.h"
#include "high_level/findGreen.h"
#include "high_level/positionAroundObject.h"

#include "custom_util/bluetooth_util.h"
#include "custom_util/constants.c"
#include "custom_util/utility.h"
#include "custom_util/motor_control.h"

int getSelector() 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int main()
{	
	int selector = getSelector();
	
	e_init_port();
	e_init_motors();
	e_init_ad_scan();
	e_init_uart1();

	// Selectors go from 0-14
	switch( selector ){
		
		// Fear Braitenberg Behaviour
		case 0:
			initFear();
			break;

		// Curiousity Braitenberg Behaviour
		case 1:
			initCuriousity();
			break;

		// Aggression Braitenberg Behaviour
		case 2:
			initAggression();
			break;

		// Love Braitenberg Behaviour
		case 3:
			initLove();
			break;

		/*
		* High level Behaviour - Two-robot box find and push.
		*
		* This selector is for the following demo:
		*		This robot as the LEADER. 
		*		Starting with a LEFT -> RIGHT traverse.
		*/
		case 4:
			btcomSendString( "Starting high level behaviour (Master) \r\n" );
			LED4 = 1;
			initHighLevelMaster( 1,  RIGHT );
			while(1);
			break;

		/*
		* High level Behaviour - Two-robot box find and push.
		*
		* This selector is for the following demo:
		*		This robot as the LEADER. 
		*		Starting with a RIGHT -> LEFT traverse.
		*/
		case 5:	
			btcomSendString( "Starting high level behaviour (R -> L) (Master) \r\n" );
			LED4 = 1;
			initHighLevelMaster( 1, LEFT );
			while(1);
			break;

		/*
		* High level Behaviour - Two-robot box find and push.
		*
		* This selector is for the following demo:
		*		This robot as the FOLLOWER.
		*/
		case 6:
			btcomSendString( "Starting high level behaviour (Not master) \r\n" );
			initHighLevelFollower();
			while(1);			
			break;
		
		default:
			// Indicated an unregistered selector.
			BODY_LED = 1;
			while(1);
			break;
	} 
	

	return 0;	
}
