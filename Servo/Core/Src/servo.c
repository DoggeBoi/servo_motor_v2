
#include "stm32f3xx_hal.h"
#include "main.h"

#include "AS5048A.h"
#include "analogue.h"
#include "motor.h"
#include "can.h"
#include "servo.h"
#include "pid.h"
#include "profile.h"
#include "LSM6DSO.h"
#include "motion.h"
#include "error.h"

#include <math.h>
#include <stdlib.h>


/*   Servo value initialisation   */
void servoCoreInit(SERVO_CONTROL *servo) {

	/*   Initialise ID and version   */
	servo->id 								= ID;
	servo->masterId							= MASTER_ID;
	servo->firmwareVer 						= FIRMWARE_VERSION;


	/*   Disable torque   */
	servo->torqueEnable 					= SERVO_TORQUE_DISABLE;


	/*   Initialise motion variables   */
	servo->motionDirection				 	= SERVO_DIRECTION_CW;


	/*   Initialise  operating limits   */
	servo->maxMotorTemp 					= SERVO_MAX_MOTOR_TEMP;
	servo->maxIntTemp 						= SERVO_MAX_INTERNAL_TEMP;
	servo->maxVoltage 						= SERVO_MAX_VOLTAGE;
	servo->minVoltage 						= SERVO_MIN_VOLTAGE;
	servo->maxCurrent 						= SERVO_MAX_CURRENT;

	servo->timeStep							= SERVO_UPDATE_INTERVAL;


	servo->commsConfrimation				= 0;
	servo->startupConfrimation 				= 0;

	pidInit		(&servo->PID, 	  servo->timeStep);
	motionInit	(&servo->motion,  servo->timeStep);
	profileInit	(&servo->profile, servo->timeStep);
	errorInit	(&servo->error);

}


/*   Encoder initialisation   */
void servoEncoderInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += AS5048A_Init(&servo->encoder, spiHandle, csPort, csPin, servo->motionDirection);

	/*   Check status   */
	if ( status != 0 ) {

		servo->error.errorEncoder		= SERVO_ERROR;

	}

}


/*   CAN-bus initialisation    */
void servoCanInit(SERVO_CONTROL *servo, CAN_HandleTypeDef *canHandle) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += CAN_Init(&servo->can, canHandle, servo->id, servo->masterId);

	/*   Check status   */
	if ( status != 0 ) {

			servo->error.errorCAN			= SERVO_ERROR;

		}

}


/*   Motor H-Bridge initialisation   */
void servoMotorInit(SERVO_CONTROL *servo, TIM_HandleTypeDef *timHandle, uint8_t timerNormalChannel, uint8_t timerInvertedChannel, GPIO_TypeDef *shutdownPort, uint16_t shutdownPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	motorInit(&servo->motor, timHandle, timerNormalChannel, timerInvertedChannel, shutdownPort, shutdownPin);


	/*   Check status   */
	if ( status != 0 ) {

			servo->error.errorPWM			= SERVO_ERROR;

		}

}


/*   ADC sensors initialisations   */
void servoAnalougeInit(SERVO_CONTROL *servo, ADC_HandleTypeDef *adcHandle0, ADC_HandleTypeDef *adcHandle1, ADC_HandleTypeDef *adcHandle2, ADC_HandleTypeDef *adcHandle3, OPAMP_HandleTypeDef *hopamp) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise sensor ADCs   */
	status += adcSensorInit(&servo->intTemp, 			rawToIntTemp, 	adcHandle0);

	status += adcSensorInit(&servo->motorCurrent,	 	rawToCurrent, 	adcHandle1);

	status += adcSensorInit(&servo->batteryVoltage, 	rawToVoltage, 	adcHandle2);

	status += adcSensorInit(&servo->motorTemp, 			rawToMotorTemp, adcHandle3);

	/*   Start current sensing opamp   */
	status += HAL_OPAMP_Start(hopamp);

	/*   Check status   */
	if ( status != 0 ) {

			servo->error.errorADC			= SERVO_ERROR;

		}

}


/*   IMU initialisations   */
void servoImuInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	status += LSM6DSO_Init(&servo->imu, spiHandle, csPort, csPin, servo->timeStep);

	/*   Check status   */
	if ( status != 0 ) {

			servo->error.errorIMU			= SERVO_ERROR;

		}

}


/*   Sensor Data check    */
void sensorsCheck(SERVO_CONTROL *servo) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Check each sensor value   */
	status += adcSensorRangeCheck(&servo->intTemp, 			&servo->error.overTempMCU, SERVO_MAX_INTERNAL_TEMP, 0);

	status += adcSensorRangeCheck(&servo->motorTemp, 		&servo->error.overTempMCU, SERVO_MAX_MOTOR_TEMP, 0);

	status += adcSensorRangeCheck(&servo->batteryVoltage, 	&servo->error.overTempMCU, SERVO_MAX_VOLTAGE, SERVO_MIN_VOLTAGE);

	status += adcSensorRangeCheck(&servo->motorCurrent, 	&servo->error.overTempMCU, SERVO_MAX_CURRENT, -SERVO_MAX_CURRENT);

	/*   If range error has occurred call error handler   */
	if ( status != 0 ) {

		commandShutdown(servo, NULL, NULL, NULL);

	}

}


/*   Motor power control functions   */
void torqueDisable(SERVO_CONTROL *servo) {

	servo->torqueEnable = SERVO_TORQUE_DISABLE;
	pidIntegratorDisable(&servo->PID);
}


void torqueEnable(SERVO_CONTROL *servo, uint8_t direction) {

	servo->motionDirection = direction;

	AS5048A_ReadAngle(&servo->encoder);
	servo->goalPosition = servo->encoder.angle;
	servo->torqueEnable = SERVO_TORQUE_ENABLE;
	pidIntegratorEnable(&servo->PID);
}


void servoConnectionCheck(SERVO_CONTROL *servo) {

	/*   If conncetion to IMU, encoder or CAN-bus cannot be established   */
	if ( ( LSM6DSO_ConnectionCheck(&servo->imu) != 0 ) || ( AS5048A_ConnectionCheck(&servo->encoder) != 0 ) || ( servo->commsConfrimation == 0 )) {

		commandShutdown(servo, NULL, NULL, NULL);

	}

	/*   Reset confirmation check   */
	servo->commsConfrimation = 0;

}


