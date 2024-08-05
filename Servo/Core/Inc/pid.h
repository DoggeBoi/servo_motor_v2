

#ifndef INC_PID_H_
#define INC_PID_H_


#include "stm32f3xx_hal.h"


/*   Base PID loop filter constant  */
#define PID_LPF				100						// 0.1


/*   Base PID gains   */
#define PID_KP				1500					// 1.5
#define PID_KD				1500					// 0.015
#define PID_KI				125						// 0.125


/*   PID output limits   */
#define PID_MAX				511
#define PID_MIN		   	   -512


/*   PID controller internal/external variables   */
typedef struct {

	/*   PID gains   */
	int16_t  	Kp;
	int16_t  	Ki;
	int16_t  	Kd;

	/*   PID values   */
	float 		integrator;
	float 		differentaitor;
	float 		proportinal;

	/*   Integrator enable term   */
	uint8_t 	integratorEnable;

	/*   PID values external   */
	int16_t		extIntegrator;
	int16_t		extDifferentaitor;
	int16_t		extProprotinal;

	/*   PID  state   */
	uint16_t 	setPoint;
	int16_t  	Error;
	uint16_t 	Input;
	int16_t	 	prevError;

	/*   PID low-pass filter constant   */
	uint16_t 	lpfConstant;

	/*   PID output clamp filter limits   */
	int16_t 	outputMax;
	int16_t 	outputMin;

	/*   PID integrator anti whind-up dynamic clamping terms   */
	float integratorLimitMax;
	float integratorLimitMin;

	/*   PID timestep   */
	uint16_t 	timeStep;             				// mS

	/*   PID output   */
	int16_t 	output;

	/*   PID limited output   */
	int16_t 	outputLimited;

} PID_CTRL;


/*   PID value initialisation   */
void pidInit(PID_CTRL *PID, uint8_t timeStep);


/*   PID calculate and update   */
void pidUpdate(PID_CTRL *PID, int16_t input, int16_t setpoint);


/*   Enable PID integrator   */
void pidIntegratorEnable(PID_CTRL *PID);


/*   Disable PID integrator   */
void pidIntegratorDisable(PID_CTRL *PID);


#endif /* INC_PID_H_ */
