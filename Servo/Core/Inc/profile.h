
#ifndef INC_PROFILE_H_
#define INC_PROFILE_H_

#include "stm32f3xx_hal.h"
#include <math.h>
#include <stdlib.h>

/*   Motion profile status   */
#define	PROFILE_ONGOING   	1
#define	PROFILE_COMPLETE  	0


/*   Motion profile split status   */
#define PROFILE_SPLIT_2		2
#define PROFILE_SPLIT_1		1
#define	PROFILE_RESET 		0


/*   Profile following threshold deviation   */
#define PROFILE_FOLLOWING_THRESHOLD	300


/*   Operating limits   */
#define PROFILE_MAX_ACCELERATION  	10000
#define PROFILE_MAX_VELOCITY  		6000
#define PROFILE_MAX_POSITION		16000					// Absolute position limits to avoid encoder loop-back
#define PROFILE_MIN_POSITION		383


/*   Motion profile settings and status   */
typedef struct {

	/*   Motion profile acceleration phase parameters   */
	float 		trajectoryAccelerationTime;
	float 		trajectoryAccelerationDistance;

	/*   Motion profile constant-velocity phase parameters   */
	float 		trajectoryVelocityTime;
	float 		trajectoryVelocityDistance;

	/*   Motion profile deceleration phase parameters   */
	float 		trajectoryDecelerationTime;
	float 		trajectoryDecelerationDistance;

	/*   Motion profile parameters   */
	int16_t		trajectoryDistance;
	int8_t 		trajectoryDirection;						// 1 if positive -1 if negative
	int16_t		trajectoryVelocity;
	uint16_t    trajectoryGoalPosition;
	int32_t 	trajectoryStartPosition;
	int32_t 	trajectorySetPoint;
	int16_t     trajectoryStartVelocity;
	uint16_t    trajectoryMaxAcceleration;
	uint16_t    trajectoryMaxVelocity;

	/*   Motion profile limits   */
	uint16_t  	maxAcceleration;							// Maximum acceleration
	uint16_t  	maxVelocity;								// Maximum velocity
	uint16_t 	maxPosition;					    		// 0-2^14
	uint16_t 	minPosition;					    		// 0-2^14

	/*   Motion profile status   */
	uint16_t	trajectoryTime;
	uint8_t  	trajectoryStatus;
	uint8_t		trajectorySplit;							// Curve has been split for ease of calculation
	uint8_t  	trajectoryFollowing;						// Motor following profile trajectory
	uint16_t 	followingThreshold;							// Threshold deviation to be considered following profile

	/*   Motion profile timestep   */
	uint16_t 	timeStep;             						// mS

} PROFILE;


/*   Motion profile initialsation    */
void profileInit(PROFILE *profile, uint8_t timeStep);


/*   Motion profile calculation   */
uint16_t profileUpdate(PROFILE *profile, int16_t goalPosition);		// Returns set-point


/*   Motion profile following check   */
void profileFollowing(PROFILE *profile, uint16_t realAngle);


#endif /* INC_PROFILE_H_ */
