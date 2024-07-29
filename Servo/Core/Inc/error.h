
#ifndef INC_ERROR_H_
#define INC_ERROR_H_


#include "stm32f3xx_hal.h"


/*   Error status   */
typedef struct {

	uint8_t 	rangeVoltage;						// Voltage not within range of maxVoltage and minVoltage
	uint8_t 	overTempMCU;						// Internal MCU temperature exceeding maxTempInt
	uint8_t 	overTempMotor;						// Motor temperature exceeding maxTempMotor
	uint8_t 	overCurrent;						// Current exceeding maxCurrent

	uint8_t 	errorEncoder;						// Error with encoder communication
	uint8_t 	errorPWM;							// Error with motor drive
	uint8_t 	errorADC;							// Error with ADC calibration
	uint8_t 	errorIMU;							// Error with IMU communication
	uint8_t 	errorCAN;							// Error with CAN bus communication

} ERRORR;		// Can't use ERROR. Declared as different kind of symbol


/*   Error value initialisation   */
void errorInit(ERRORR *error);

/*   Combine error flags into byte   */
uint8_t errorCombine(ERRORR *error);


#endif /* INC_ERROR_H_ */
