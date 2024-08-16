
#include "current.h"
#include "stm32f3xx_hal.h"
#include <stdlib.h>

/*   Current controller initialisation   */
void currentInit(CURRENT_CTRL *cCtrl) {

	cCtrl->maxCurrent = MAX_CURRENT;

}


/*   Calculate limited dutyCycle value   */
int16_t currentLimit(CURRENT_CTRL *cCtrl, int16_t current, int16_t temperature, int16_t dutyCycle) {

	/*   Change to mA   */
	float correctedCurrent = current / 10.0f;

	/*   Correct current cutoff   */
	float correctedLimit = cCtrl->maxCurrent - 40;

	/*   Calculate limiter value from 0 - 1   */
	float limitionMultiplier = ( 1.0f - 0.0002f * ( correctedCurrent - correctedLimit ) * ( correctedCurrent - correctedLimit ) );

	/*   If current is above threshold   */
	if ( correctedCurrent > correctedLimit ) {

		return ( dutyCycle * limitionMultiplier );

	}

	return dutyCycle;

}
