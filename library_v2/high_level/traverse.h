/**
* Header file for traverse functionality
**/


/**
* Init the search behaviour.
* This behaviour is made up of a series of sub-behaviours, that when 
* combined, make up this higher level behaviour.
*/ 
void initTraverse();

// Perform traverse 
void traverse();

/**
* Finish search behaviour.
*/
void endTraverse();


/**
* Sub-Behaviours.
*
* Each of these behaviours make up the entirety of the traverse behaviour.
*
* Each behaviour has a TRIGGER and a EXECUTINO
*
**/


/**
* The cruise behaviour.
*
* Cruise is to simply move at the normal speed in a straight line.
*/ 
void cruise();


/**
* The "avoid wall" behaviour.
*
* This behaviour is triggered on some triggerAvoidWall() true.
*
* Flow:
* 	When triggered:
*		Pause other behaviours.
*		Avoid wall in some way
*		Resume behaviours.
*/
int triggerAvoidWall();
void avoidWall();
int stillTurning();

/**
* The "approach wall" behaviour.
*
* We want to approach the wall slightly so that we can become
* perpendicular to it.
*
* This will therefore, allow us to have a more accurate "avoid wall" turn.
*/
int approachingWall();
void approachWall();

/**
* The "search" behaviour.
*
* This behaviour is the process of looking for a given "thing".
* ( In our case, this is the green box )
*
* Flow:
*	When triggered:
*		Set an indicator to say we have found the object.
*		Terminate all other behaviours.
*/
int foundObject();


