
#include "error.h"
#include "stm32f3xx_hal.h"


/*   Error value initialisation   */
void errorInit(ERRORR *error) {

	/*   Initialise hardware error status   */
	error->rangeVoltage		= 0;
	error->overTempMCU		= 0;
	error->overTempMotor	= 0;
	error->overCurrent		= 0;

	error->errorEncoder		= 0;
	error->errorPWM			= 0;
	error->errorADC			= 0;
	error->errorIMU			= 0;

}

/*   Combine error flags into byte   */
uint8_t errorCombine(ERRORR *error) {

	uint8_t combinedFlags = 0;

	/*   Initialise list of error flags   */
	uint8_t errorList[] =
	{

		error->rangeVoltage,
		error->overTempMCU,
		error->overTempMotor,
		error->overCurrent,
		error->errorEncoder,
		error->errorPWM,
		error->errorADC,
		error->errorIMU

	};


	/*   Assign each flag to bit of byte   */
		for ( uint8_t i = 0; sizeof( errorList ) / sizeof( uint8_t ); i++ ) {

			if (errorList[i] == 1) {

					combinedFlags |= (1 << i); // Set the i-th bit to 1

				} else {

					combinedFlags &= ~(1 << i); // Set the i-th bit to 0

			    }

		}

		return combinedFlags;

}
