
#ifndef STM32F303RE_SERVO_DRIVER_INC_MOTOR_H
#define STM32F303RE_SERVO_DRIVER_INC_MOTOR_H


#include "stm32f3xx_hal.h"
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
#include "cmsis_os.h"

/*   Default values   -------------------------------------   */

/*   Device information   */
#define ID							1						// 8 bit Device ID
#define MASTER_ID					0
#define FIRMWARE_VERSION			1


/*   Motion direction   */
#define SERVO_DIRECTION_CCW			0
#define SERVO_DIRECTION_CW	    	1


/*   Torque enable   */
#define SERVO_TORQUE_ENABLE  		1
#define SERVO_TORQUE_DISABLE 		0


/*   Operating limits   */
#define SERVO_MAX_VOLTAGE			16000					// 16 V
#define SERVO_MIN_VOLTAGE			8000					// 8 V
#define SERVO_MAX_MOTOR_TEMP		8000					// 80 °C
#define SERVO_MAX_INTERNAL_TEMP		8000					// 80 °C
#define SERVO_MAX_CURRENT			3000					// 3 A


/*   Base PID loop sample interval and filter constant  */
#define SERVO_UPDATE_INTERVAL		4						// 4 ms


/*   Connection check interval   */
#define SERVO_CONNECTION_INTERVAL	500


/*   Error value   */
#define SERVO_ERROR 1
#define SERVO_OK	0


/*   Servo core strcut   */
typedef struct {

	/*   Version and ID   */
	uint8_t  	id;										// 8 bit CAN motor identifier
	uint8_t		masterId;							    // 8 bit CAN master identifier
	uint8_t  	firmwareVer;							// Firmware version


	/*   Position and drive status   */
	uint16_t 	goalPosition;							// 0-2^14
	uint8_t  	torqueEnable;
	uint8_t  	motionDirection;						// Motion direction 0 is CW 1 is CCW


	/*   Operating limits   */
	uint16_t 	maxMotorTemp;							// Centi°C
	uint16_t 	maxIntTemp;								// Centi°C
	uint16_t 	maxVoltage;								// mV
	uint16_t 	minVoltage;								// mV
	uint16_t 	maxCurrent;								// mA

	/* Main loop update period   */
	uint8_t		timeStep;


	/*   Error status   */
	ERRORR  	error;


	/*   PID parameters   */
	PID_CTRL  	PID;


	/*   Rotary encoder   */
	AS5048A 	encoder;


	/*   ADC Sensors   */
	ADC 		intTemp;
	ADC 		motorTemp;
	ADC 		batteryVoltage;
	ADC 		motorCurrent;


	/*   H-bridge motor drive   */
	MOTOR 		motor;


	/*   Motion profile parameters   */
	PROFILE 	profile;


	/*   CAN bus   */
	CANBUS 		can;


	/*   Inertial measurement unit   */
	LSM6DSO 	imu;


	/*   Motion status  */
    MOTION 		motion;


    /*   Communication confirmation variables   */
    uint8_t		commsConfrimation;
    uint8_t		startupConfrimation;


	/*   Operation function array   */
	void* 		OperationFunctions	[8];


	/*   List if externally accessible variables   */
	void*    externalVariableList[47];


} SERVO_CONTROL;


/*   Operation function pointer   */
typedef void (*OperationFucntionPointer)(SERVO_CONTROL*, uint8_t*, uint8_t, uint8_t);


/*   Servo external variables initialisation   */
void servoExtVariablesInit(SERVO_CONTROL *servo);


/*   Core servo value initialisation   */
void servoCoreInit(SERVO_CONTROL *servo);


/*   Peripheral and external device initialisation ---------------------------------------   */

/*   Initialise can bus */
void servoCanInit(SERVO_CONTROL *servo, CAN_HandleTypeDef *canHandle);


/*   Encoder initialisation   */
void servoEncoderInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);


/*   Motor H-Bridge initialisation   */
void servoMotorInit(SERVO_CONTROL *servo, TIM_HandleTypeDef *timerHandle, uint8_t timerNormalChannel, uint8_t timerInvertedChannel, GPIO_TypeDef *shutdownPort, uint16_t shutdownPin);


/*   ADC sensors initialisation   */
void servoAnalougeInit(SERVO_CONTROL *servo, ADC_HandleTypeDef *adcHandle0, ADC_HandleTypeDef *adcHandle1, ADC_HandleTypeDef *adcHandle2, ADC_HandleTypeDef *adcHandle3, OPAMP_HandleTypeDef *hopamp);


/*   IMU initialisation   */
void servoImuInit(SERVO_CONTROL *servo, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);


/*   Peripheral and external device connection check ---------------------------------------   */

void servoConnectionCheck(SERVO_CONTROL *servo);


/*   Motor power control functions   */
void torqueDisable(SERVO_CONTROL *servo);

void torqueEnable(SERVO_CONTROL *servo, uint8_t direction);


/*   Sensor Data check    */
void sensorsCheck(SERVO_CONTROL *servo);


#endif /* STM32F303RE_SERVO_DRIVER_INC_MOTOR_H */
