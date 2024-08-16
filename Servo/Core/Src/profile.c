
#include "profile.h"
#include "stm32f3xx_hal.h"
#include <math.h>
#include <stdlib.h>


void profileInit(PROFILE *profile, uint8_t timeStep) {

	/*   Initialise motion profile limits   */
	profile->maxPosition 				= PROFILE_MAX_POSITION;
	profile->minPosition				= PROFILE_MIN_POSITION;
	profile->maxAcceleration 			= PROFILE_MAX_ACCELERATION;
	profile->maxVelocity 	   			= PROFILE_MAX_VELOCITY;

	/*   Initialise motion profile state   */
	profile->trajectoryStatus 			= PROFILE_COMPLETE;
	profile->trajectoryGoalPosition 	= 0;
	profile->trajectoryVelocity 		= 0;

	/*   Set following threshold    */
	profile->followingThreshold 		= PROFILE_FOLLOWING_THRESHOLD;

	/*   Set motion profile time-step   */
	profile->timeStep					= timeStep;             						// mS

}


/*   Motion profile calculation    */
uint16_t profileUpdate(PROFILE *profile, int16_t goalPosition) {

	/*   Motion profile calculation phase duration calculation   */
	if (
		( goalPosition 				!= 		profile->trajectoryGoalPosition )
	||  ( profile->maxAcceleration 	!= 		profile->trajectoryMaxAcceleration )
	||  ( profile->maxVelocity 		!= 		profile->trajectoryMaxVelocity )
	||  ( profile->trajectorySplit  == PROFILE_SPLIT_2 )
	   )

	{

		/*   Reset split status if goal has changed or if in SERVO_PROFILE_SPLIT_2 state   */
		profile->trajectorySplit = ( goalPosition == profile->trajectoryGoalPosition ) * (profile->trajectorySplit != PROFILE_SPLIT_2 );

		/*   Motion profile parameter calculation   */
		profile->trajectoryStartPosition 			= 	profile->trajectorySetPoint;
		profile->trajectoryGoalPosition				=   goalPosition;
		profile->trajectoryMaxAcceleration			=   profile->maxAcceleration;
		profile->trajectoryMaxVelocity				=	profile->maxVelocity;

		profile->trajectoryDistance 				= 	profile->trajectoryGoalPosition - profile->trajectoryStartPosition;
		profile->trajectoryDirection 				= ( profile->trajectoryDistance >= 0 ) ? 1 : -1;
		profile->trajectoryStartVelocity			=   profile->trajectoryVelocity;

		/*   Trajectory time reset   */
		profile->trajectoryTime						= 0;

		/*   Motion profile acceleration phase parameter calculation   */
		profile->trajectoryAccelerationTime 		= (float) abs( profile->trajectoryDirection * profile->maxVelocity - profile->trajectoryStartVelocity )  / (float) profile->maxAcceleration;
		profile->trajectoryAccelerationDistance 	= ( profile->trajectoryDirection * profile->trajectoryStartVelocity * profile->trajectoryAccelerationTime ) + 0.5f * profile->maxAcceleration * profile->trajectoryAccelerationTime * profile->trajectoryAccelerationTime;

		/*   Motion profile deceleration phase parameter calculation   */
		profile->trajectoryDecelerationTime			= (float) profile->maxVelocity  / (float) profile->maxAcceleration;
		profile->trajectoryDecelerationDistance		= 0.5f * profile->maxAcceleration * profile->trajectoryDecelerationTime * profile->trajectoryDecelerationTime;

		/*   Motion profile constant-velocity phase parameter calculation   */
		profile->trajectoryVelocityDistance 		= abs(profile->trajectoryDistance ) - ( profile->trajectoryAccelerationDistance + profile->trajectoryDecelerationDistance );
		profile->trajectoryVelocityTime 			= profile->trajectoryVelocityDistance / profile->maxVelocity;

		/*   If distance too short to reach max velocity and start velocity is 0   */
		if ( profile->trajectoryVelocityDistance < 0 && profile->trajectoryStartVelocity == 0 ) {		// check if calculated veloity is actually zero by the end and there sint some roudning error like 0.0000001

			profile->trajectoryAccelerationTime 	= sqrt( (float) abs( profile->trajectoryDistance ) / (float) profile->maxAcceleration );
			profile->trajectoryAccelerationDistance = 0.5f * profile->maxAcceleration * profile->trajectoryAccelerationTime * profile->trajectoryAccelerationTime;

			profile->trajectoryDecelerationTime		= profile->trajectoryAccelerationTime;

			profile->trajectoryVelocityTime			= 0;
			profile->trajectoryVelocityDistance		= 0;

		}

		/*   If distance too short to reach max velocity and start velocity is not 0   */
		if ( profile->trajectoryVelocityDistance < 0 && profile->trajectoryStartVelocity != 0 ) {

			profile->trajectoryAccelerationTime		= 0;
			profile->trajectoryAccelerationDistance = 0;

			profile->trajectoryVelocityTime			= 0;
			profile->trajectoryVelocityDistance		= 0;

			/*   Decelerate to zero   */
			profile->trajectoryDecelerationTime		= (float) abs(profile->trajectoryStartVelocity) / (float) profile->maxAcceleration;

			/*   Set direction to that of the deceleration of SERVO_PROFILE_SPLIT_1 and not full move   */
			profile->trajectoryDirection 			= ( profile->trajectoryStartVelocity >= 0 ) ? 1 : -1;

			/*   Update profile split status to deceleration phase   */
			profile->trajectorySplit 				= PROFILE_SPLIT_1;

		}

		/*   Motion profile status update   */
		profile->trajectoryStatus 					= PROFILE_ONGOING;

	}


	/*   Convert time reference to seconds   */
	float time = profile->trajectoryTime * 0.001f;


	/*   Acceleration phase   */
	if (time <= profile->trajectoryAccelerationTime ){

		profile->trajectorySetPoint =
		profile->trajectoryStartPosition
	 +  profile->trajectoryDirection
	 *( profile->trajectoryStartVelocity * time
	 *  profile->trajectoryDirection + 0.5f
	 *  profile->maxAcceleration * time * time );

		profile->trajectoryVelocity =
		profile->trajectoryDirection
	 *( profile->trajectoryStartVelocity
	 *  profile->trajectoryDirection
	 +  profile->maxAcceleration * time );

	}

	/*   Constant velocity phase   */
	else if (time <= ( profile->trajectoryAccelerationTime + profile->trajectoryVelocityTime ) ) {

		float t = time - profile->trajectoryAccelerationTime;

		profile->trajectorySetPoint =
		profile->trajectoryStartPosition
	+   profile->trajectoryDirection
	* ( profile->trajectoryAccelerationDistance
	+   profile->maxVelocity * t);

		profile->trajectoryVelocity =
		profile->trajectoryDirection
	 *  profile->maxVelocity;

	}

	/*   Deceleration phase   */
	else if (time <= ( profile->trajectoryAccelerationTime + profile->trajectoryVelocityTime + profile->trajectoryDecelerationTime ) ) {

		float t = time - profile->trajectoryAccelerationTime - profile->trajectoryVelocityTime;

		  profile->trajectorySetPoint =
		  profile->trajectoryStartPosition
	+     profile->trajectoryDirection
	*   ( profile->trajectoryAccelerationDistance
	+     profile->trajectoryVelocityDistance
	+ ( ( profile->maxAcceleration
	*     profile->trajectoryAccelerationTime
	+     profile->trajectoryStartVelocity
	*     profile->trajectoryDirection ) * t )
	-   ( profile->maxAcceleration * 0.5f * t * t ) );

		  profile->trajectoryVelocity =
		  profile->trajectoryDirection
	* ( ( profile->maxAcceleration
    * 	  profile->trajectoryAccelerationTime
	+     profile->trajectoryStartVelocity * profile->trajectoryDirection)
	-     profile->maxAcceleration * t );

	}

	/*   Hold phase   */
	else {

		/*   Update status and velocity when complete   */
		profile->trajectoryVelocity 				= 0;
		profile->trajectoryStatus 					= PROFILE_COMPLETE;

		/*   Update trajectory split status to SERVO_PROFILE_SPLIT_2   */
		if( profile->trajectorySplit == PROFILE_SPLIT_1 ) profile->trajectorySplit = PROFILE_SPLIT_2;

	}

	/*   Increment trajectory time   */
	profile->trajectoryTime += profile->timeStep * !!profile->trajectoryStatus;				// Increment by time step (ms) if trajectory is not yet complete.


	/*    Return within limits   */

	/*   If below minimum limit   */
	if ( profile->trajectorySetPoint < profile->minPosition ) {

		return profile->minPosition;

	}

	/*   If above maximum limit   */
	else if ( profile->trajectorySetPoint > profile->maxPosition){

		return profile->maxPosition;

	}

	/*   If within limits   */
	return profile->trajectorySetPoint;


}


/*   Motion profile following check   */
void profileFollowing(PROFILE *profile, uint16_t realAngle) {

	profile->trajectoryFollowing = ( abs( profile->trajectorySetPoint - realAngle ) <= profile->followingThreshold );

}


/*   Reset trajectory calculation from..  */
void profileResetFrom(PROFILE *profile, int16_t velocity, uint16_t angle){						// Sets start velocity and position to avoid jolt when enabling torque

	profile->trajectoryVelocity = velocity;
	profile->trajectorySetPoint = angle;

}
