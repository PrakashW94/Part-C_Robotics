#ifndef _EMITTER
#define _EMITTER

/*
* Emit a message via IR.
*/ 
void emit();

/*
* Emit some master ACKs.
*/
void emitMasterAcks();

/*
* Emit a follow message via IR.
*/ 
void emitFollow();


/*
* Emits position.
*/
void emitPos();

#endif
