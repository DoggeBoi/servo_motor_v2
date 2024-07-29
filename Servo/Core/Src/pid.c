
#include "pid.h"
#include "stm32f3xx_hal.h"


/*   PID value initialisation   */
void pidInit(PID_CTRL *PID, uint8_t timeStep) {

	/*   Initialise PID sample period value   */
	PID->timeStep   		= 	timeStep;

	/*   Initialise PID output limits   */
	PID->outputMax      	=  	PID_MAX;
	PID->outputMin      	=  	PID_MIN;

	/*   Initialise PID gains   */
	PID->Kp 			  	= 	PID_KP;
	PID->Kd 			  	= 	PID_KD;
	PID->Ki 			  	= 	PID_KI;

	/*   Integrator enable   */
	PID->integratorEnable	=	1;

	/*   Initialise low pass filter parameter   */
	PID->lpfConstant 	  	= 	PID_LPF;

	/*   Initialise previous values    */
	PID->prevInput 	  		= 	0;
	PID->prevError			= 	0;

	/*   Initialise PID terms    */
	PID->differentaitor 	= 	0;
	PID->integrator	  		= 	0;

}


/*   PID calculate and update   */
void pidUpdate(PID_CTRL *PID, int16_t input, int16_t setpoint) {


	/*   Calculate unit corrected values   */
	float timeStep 	= PID->timeStep 	/ 1000.0f;
	float lpfC	 	= PID->lpfConstant 	/ 1000.0f;
	float Kp		= PID->Kp 			/ 1000.0f;
	float Ki		= PID->Ki 			/ 1000.0f;
	float Kd		= PID->Kd 			/ 100000.0f;


	/*   Update PID input   */
	PID->Input 				= 	input;
	PID->setPoint			= 	setpoint;


	/*   Calculate error   */
	PID->Error      		= 	PID->setPoint - PID->Input;


	/*   Calculate PID proportional term   */
	PID->proportinal 		= 	Kp * PID->Error;


	/*   Calculate PID integrator term   */				// Disable change if torque is off./////////////////////////
	PID->integrator 		= 	( PID->integrator + 0.5f * Ki * timeStep *
								( PID->Error + PID->prevError ) ) * PID->integratorEnable;


	/*   Calculate PID dynamic anti whind-up limits   */
	if ( PID->outputMax > PID->proportinal )  				PID->integratorLimitMax 	= ( PID->outputMax - PID->proportinal );
	else 													PID->integratorLimitMax 	= 0.0f;

	if ( PID->outputMin < PID->proportinal )  				PID->integratorLimitMin 	= ( PID->outputMin - PID->proportinal );
	else 													PID->integratorLimitMin 	= 0.0f;


    /*   Apply calculated clamping limits   */
	if 		( PID->integrator > PID->integratorLimitMax )   PID->integrator 			= PID->integratorLimitMax;
	else if ( PID->integrator < PID->integratorLimitMin ) 	PID->integrator 			= PID->integratorLimitMin;


	/*   Calculate PID differentaitor term   */
	PID->differentaitor 	= 	( ( 2.0f * Kd  * ( PID->Error - PID->prevError ) ) +
								  ( 2.0f * lpfC - timeStep )  *   PID->differentaitor ) /
								  ( 2.0f * lpfC + timeStep );

	/*   Calculate PID output   */
	PID->output = (int16_t) ( PID->proportinal + PID->integrator + PID->differentaitor);


	/*   Apply clamping limiter to PID output   */
	if 		(PID->output < PID->outputMin) 	PID->outputLimited = PID->outputMin;

	else if (PID->output > PID->outputMax) 	PID->outputLimited = PID->outputMax;

	else 									PID->outputLimited = PID->output;


	/*   Update previous variables for use in next PID cycle    */
	PID->prevInput 				= PID->Input;
	PID->prevError				= PID->Error;

    /*   Update external PID values   */
	PID->extProprotinal			= PID->proportinal;
	PID->extDifferentaitor		= PID->differentaitor;
	PID->extIntegrator			= PID->integrator;

}


/*   Enable PID integrator   */
void pidIntegratorEnable(PID_CTRL *PID) {

	PID->integratorEnable = 1;

}


/*   Disable PID integrator   */
void pidIntegratorDisable(PID_CTRL *PID) {

	PID->integratorEnable = 0;

}

