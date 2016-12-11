#include "bluetooth/e_bluetooth.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"

void printEpuck( char* address, char* number )
{
	char uartbuffer[100];
	sprintf(uartbuffer, "e-puck - %s - %s \n\r", address, number);
	e_send_uart1_char(uartbuffer, strlen(uartbuffer));
}


/**
* Print all epuck available connections
**/
void printNearbyEpucks()
{
	int i;
	int epucks_found = e_bt_find_epuck();
	
	reportValue("Epucks found", epucks_found);
	for( i = 0; i < 10; i++ )
	{
		char* address = e_bt_present_epuck[i].address;
		char* number = e_bt_present_epuck[i].number;
		printEpuck( address,number );
	}
}

/**
* Create a connection to an e-puck, using an address and pin.
*/
int connect_to_epuck( char* address, int* pin )
{
/*	char error;

	// Read current pin	
	e_bt_read_local_pin_number( local_bt_PIN );
	
	// Write epuck to connect to pin
	e_bt_write_local_pin_number( *pin );
	
	// Attempt to establish comms
	error = e_bt_establish_SPP_link( *address );
	
	// Write back old pin
	e_bt_write_local_pin_number( local_bt_PIN );

	return error;*/
}


int send_to_epuck( char* address, int* pin )
{	
	
	// Check if connection established
	// if not, establish

	// then send message
}