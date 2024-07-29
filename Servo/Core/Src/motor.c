
#include "motor.h"
#include <stdlib.h>
#include <stdio.h>


/*   Locked anti-phase driver for 2x IR2104 in full bridge configuration   */
/*   Duty cycle limited to avoid MOSFET high side on continuously and fully discharge  bootstrap capacitor   */


/*   Motor initialisation   */
uint8_t motorInit(MOTOR *motor, TIM_HandleTypeDef *timerHandle, uint8_t timerNormalChannel, uint8_t timerInvertedChannel, GPIO_TypeDef *shutdownPort, uint16_t shutdownPin) {

	/*   Initialise status variable   */
	uint8_t status 				= 0;

	/*   Assign PWM timer handle and channels   */
	motor->timerHandle 			= timerHandle;

	motor->timerNormalChannel 	= timerNormalChannel;
	motor->timerInvertedChannel = timerInvertedChannel;

	/*   Assign shutdown pin   */
	motor->shutdownPort 		= shutdownPort;
	motor->shutdownPin 			= shutdownPin;

	/*   Drive parameters   */
	motor->dutyCycle 			= 0;							// 0-1024
	motor->enabled 				= 0;							// 0 = disabled

	/*   Friction compensation value   */
	motor->frictionCompensation = MOTOR_STATIC_FRICTION;

	/*   Start PWM timers   */
	status += HAL_TIM_PWM_Start(motor->timerHandle, motor->timerNormalChannel);
	status += HAL_TIM_PWM_Start(motor->timerHandle, motor->timerInvertedChannel);

	/*   Ensure driver is disabled upon startup   */
	HAL_GPIO_WritePin		(motor->shutdownPort, motor->shutdownPin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE	(motor->timerHandle, motor->timerNormalChannel, 	512);
	__HAL_TIM_SET_COMPARE	(motor->timerHandle, motor->timerInvertedChannel,  512);

	return status;

}


/*   Update PWM   */
void motorUpdate(MOTOR *motor) {

	/*   Enable/disable shutdown  */
	if 	( motor->enabled == 0 ) HAL_GPIO_WritePin(motor->shutdownPort, motor->shutdownPin, GPIO_PIN_RESET);
	else 						HAL_GPIO_WritePin(motor->shutdownPort, motor->shutdownPin, GPIO_PIN_SET);

	/*   Set PWM duty cycle value   */
	__HAL_TIM_SET_COMPARE(motor->timerHandle, motor->timerNormalChannel, motor->dutyCycle);
	__HAL_TIM_SET_COMPARE(motor->timerHandle, motor->timerInvertedChannel, motor->dutyCycle);

}


/*   PWM decomposition into magnitude and direction   */
void motorPwmDecomp(MOTOR *motor, int16_t input, uint8_t enable, uint8_t direction) {

	/*   Set driver enable/disable based on input parameter   */
	motor->enabled = enable;

	/*   Correct input based on direction parameter   */
	int16_t correctedInput = ( ( direction == 0 ) ? input : ( -1 * input -1 ) );				// If inverted, invert input subtract 1 to map -512 -- 511 to 511 -- -512

	/*   Add friction compensation value if input is below */
	if ( abs( correctedInput ) < motor->frictionCompensation ) {

		if ( correctedInput >= 0 ) 	correctedInput =  motor->frictionCompensation;

		else						correctedInput = -1 *  motor->frictionCompensation;

	}

	/*   Shift range  ( locked anti phase drive )   */
	uint16_t shiftedInput = correctedInput + 512;												// Shifts range by 512 to map -512 -- 511 to 0 -- 1023

	/*   Clamp limit duty cycle   */
	if ( shiftedInput > MOTOR_DUTY_MAX ) shiftedInput = MOTOR_DUTY_MAX;
	if ( shiftedInput < MOTOR_DUTY_MIN ) shiftedInput = MOTOR_DUTY_MIN;

	motor->dutyCycle = shiftedInput;

}
