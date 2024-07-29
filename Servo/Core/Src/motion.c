
#include "motion.h"
#include <stdlib.h>

void motionInit(MOTION *motion, uint8_t timeStep) {

	motion->prevInput 			= 0;
	motion->prevVelocity 		= 0;
	motion->inMotion 			= 0;
	motion->motionThreshold 	= MOTION_THRESHOLD;
	motion->lpfConstant			= MOTION_LPF;
	motion->timeStep			= timeStep;

}


void motionUpdate(MOTION *motion, int16_t input) {

	/*   Calculate unit corrected values   */
	float timeStep 				= motion->timeStep 		/ 1000.0f;
	float lpfC	 				= motion->lpfConstant 	/ 1000.0f;

	/*   Calculate velocity   */
	motion->velocity 	  		= ( ( 2.0f * ( input - motion->prevInput ) ) +
			  	  	  	  	  	    ( 2.0f * lpfC - timeStep ) * motion->velocity ) /
			  	  	  	  	  	    ( 2.0f * lpfC + timeStep );

	/*   Calculate acceleration   */
	motion->acceleration 	  	= ( ( 2.0f * ( motion->velocity  - motion->prevVelocity ) ) +
  	  	  	    					( 2.0f * lpfC - timeStep ) * motion->acceleration ) /
  	  	  	    					( 2.0f * lpfC + timeStep );

	/*   Update previous variables for use in next cycle    */
	motion->prevInput			= input;
	motion->prevVelocity 		= motion->velocity;


    /*   Update inMotion variable   */
	motion->inMotion = ( ( abs( motion->velocity ) >= motion->motionThreshold ) || ( abs( motion->acceleration ) >= motion->motionThreshold ) );
	/*   1 if velocity or acceleration is above threshold, only 0 if stationary and not changing velocity.   */

}
