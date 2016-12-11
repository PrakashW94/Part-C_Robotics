#include "btcom/btcom.h"

#include "high_level/global.h"

#include "ircom.h"
#include "ircomSend.h"

void emit()
{	
	//btcomSendString("Sending IR message... \r\n");

	ircomSend( global.messageToEmit );

	//btcomSendString("Sending...");

	while ( ircomSendDone() == 0 );

	//btcomSendString("Sent IR Message. \r\n");	
}
