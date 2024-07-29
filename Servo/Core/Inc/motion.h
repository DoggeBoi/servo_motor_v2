
#ifndef SRC_MOTION_H_
#define SRC_MOTION_H_


#include "stm32f3xx_hal.h"


/*   Motion threshold velocity   */
#define MOTION_THRESHOLD 	100								// ~2.2 degree/s


/*   Base motion loop filter constant  */
#define MOTION_LPF				100							// 0.1


typedef struct {


	/*   Motion status  */
	int16_t		prevInput;
	int16_t   	velocity;
	int16_t	  	prevVelocity;
	int16_t   	acceleration;
	uint8_t  	inMotion;									// Currently moving
	uint16_t 	motionThreshold;							// Threshold velocity to be considered moving

	/*   Motion low-pass filter constant   */
	uint16_t 	lpfConstant;

	/*   Motion time-step   */
	uint16_t 	timeStep;             				// mS

} MOTION;


/*   Motion value initialisation   */
void motionInit(MOTION *motion, uint8_t timeStep);


/*   Motion calculate and update   */
void motionUpdate(MOTION *motion, int16_t input);


#endif /* SRC_MOTION_H_ */
