
#ifndef IR2104_INC_IR2104_H
#define IR2104_INC_IR2104_H


#include "stm32f3xx_hal.h"


/*   Operating limits   */
//#define MOTOR_DUTY_MAX 			972 	// ~ 95% of 1023 To avoid discharging either bootstrap capacitor
//#define MOTOR_DUTY_MIN			51		// ~ 5% of 1023

#define MOTOR_DUTY_MAX 			972 	// ~ 95% of 1023 To avoid discharging either bootstrap capacitor
#define MOTOR_DUTY_MIN			51		// ~ 5% of 1023


/*   Static friction compensation value   */
#define MOTOR_STATIC_FRICTION		25


typedef struct {

	/*   PWM timer handle and channels   */
	TIM_HandleTypeDef  *timerHandle;
	uint8_t				timerNormalChannel;
	uint8_t				timerInvertedChannel;

	/*   Shutdown pin   */
	GPIO_TypeDef 	   *shutdownPort;
	uint16_t 		   	shutdownPin;

	/*   Drive parameters   */
	uint16_t 			dutyCycle;							// 0-1022, 511 50% is 0 torque
	uint8_t 			enabled;							// 1 if shutdown is off

	/*   Friction compensation value   */
	uint8_t 			frictionCompensation;				// Adds value to PWM to avoid startup fiction

} MOTOR;


/*   Motor initialisation   */
uint8_t motorInit(MOTOR *motor, TIM_HandleTypeDef *timerHandle, uint8_t timerNormalChannel, uint8_t timerInvertedChannel, GPIO_TypeDef *shutdownPort, uint16_t shutdownPin);


/*   PWM decomposition into magnitude and direction   */
void motorPwmDecomp(MOTOR *motor, int16_t input, uint8_t enable, uint8_t direction);


/*   Update PWM and H-brdige switches   */
void motorUpdate(MOTOR *motor);


#endif /* IR2104_INC_IR2104_H */
