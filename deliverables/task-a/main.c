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
	

	//wait(1000000);

	int counter = 0;

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
		// IR Test (Sender)
		case 4:
			btcomSendString( "Starting IR send test. \r\n" );
			e_calibrate_ir(); 
			// initialize ircom and start reading
    		ircomStart();
    		ircomEnableContinuousListening();
    		ircomListen();
			
			LED1 = 1;
			while(1)
			{
				btcomSendString( "Initialised. \r\n" );
				// takes ~15knops for a 32window, avoid putting messages too close...
	    		int j;
				for( j = 0; j < 500; j++)
				{
					asm("nop");
				}
				btcomSendString("Looped. \r\n");			
	
				// Send bit 1.
				btcomSendString("Sending IR message... \r\n");
				
				int command = 1;
				int payload = 55;
				int packet = toBinFromInts( command, payload );
				
				btcomSendInt( packet );
			
				ircomSend( packet );
				while (ircomSendDone() == 0);
				btcomSendString("Sent IR Message. \r\n");
			}	
		
			break;
		// IR Test (Receiver)
		case 5:
			btcomSendString( "Starting IR receive test. \r\n" );
			e_calibrate_ir();
			// initialize ircom and start reading
    		ircomStart();
    		ircomEnableContinuousListening();
    		ircomListen();		
			
			while(1)	
			{	
				IrcomMessage imsg;
				Packet packet;
				int error;
				
				// Wait to receive
				btcomSendString("Waiting to receive IR Message... \r\n");
				do
				{
					ircomPopMessage( &imsg );
					error = imsg.error;
				}
				while( error != 0 );
					
				//while( ircomReceiveDone() == 0 );	
				btcomSendString("Received IR Message. \r\n");
				
				int val = imsg.value;
				
				toPacket( &packet, val );

				btcomSendString( "---- MESSAGE ----- \r\n" );
				btcomSendInt( val );
				btcomSendInt( packet.command );
				btcomSendInt( packet.payload );
	

				e_led_clear();
				e_set_led( counter % 8, 1 );
				counter++;
	 	 	}  	
			
			break;
		// High level Behaviour (MASTER)
		case 6:
			btcomSendString( "Starting high level behaviour (Master) \r\n" );
			LED4 = 1;
			initHighLevelMaster( 1,  RIGHT );
			while(1);
			break;
		// High level Behaviour
		case 7:
			btcomSendString( "Starting high level behaviour (Not master) \r\n" );
			initHighLevelFollower();
			while(1);			
			break;
		// Traverse test (Left)
		case 8:	
			btcomSendString( "Starting high level behaviour (R -> L) (Master) \r\n" );
			LED4 = 1;
			initHighLevelMaster( 1, LEFT );
			while(1);
			break;
		// Traverse test (Right)
		case 9:
			btcomSendString( "Starting Traverse Test. Right Robot" );
			LED1 = 1;

			initGlobal( RIGHT );
			e_start_agendas_processing();
			e_calibrate_ir(); 
    		ircomStart();
    		ircomEnableContinuousListening();
    		ircomListen();
				
			e_activate_agenda( emit, 3000 );
			e_activate_agenda( receive, 500 );
			
			setPacketToEmit( CMD_SET_STATE, STATE_TEST_SIDE_FOLLOW );
			
			while( 1 );
			break;	
		// Traverse behaviour test
		case 10:
			btcomSendString( "Starting traverse behaviour test." );

			initGlobal( RIGHT );

			e_start_agendas_processing();
			//e_calibrate_ir(); 

    		ircomStart();
    		ircomEnableContinuousListening();
    		ircomListen();
			initTraverse();
			
			while( global.phase < PHASE_SEARCH_COMPLETE );
			btcomSendString( "Starting box follow." );
			initBoxFollow( 0 );

			while( 1 );
			break;
		// Box follow test
		case 11:
			btcomSendString( "Starting box follow test (Right -> Left)." );
			
			initGlobal(LEFT);
	
			e_start_agendas_processing();

			LED1 = 1;
			wait(1000000);
			LED1 = 0;
			
			btcomSendString( "Starting box follow." );
			initBoxFollow( 0 );
				
			BODY_LED = 1;
			
			while( 1 );
			break;
		case 12:
			btcomSendString( "Starting box follow test (Left -> Right)." );
			initGlobal(RIGHT);
			
			e_start_agendas_processing();
			
			LED1 = 1;			
			wait(1000000);
			LED1 = 0;

			btcomSendString( "Starting box follow." );
			initBoxFollow( 0 );
			
			while( 1 );
			break;
		case 13:
			// Start agenda processing
			e_start_agendas_processing();
			initGlobal(LEFT);
			global.isMaster = 0;
			LED1 = 1;
			moveToObject();
			initBoxFollow( 1 );
			while(1)
			break;
		default:
			// Indicated an unregistered selector.
			BODY_LED = 1;
			while(1);
			break;
	} 
	

	return 0;	
}
