/*
* This file is used to handle communication between 
* the robots in a protocol.
*/

#define SET 'S'
#define GEt 'G'

#define POSITION 'P'


/*
* Command List
*
* P - "P,#,#" Broadcast position X and Y (in steps)
* O - "O,#" Broadcast orientation from start ( in steps )
*/
void processMessage( char* message )
{
	
}