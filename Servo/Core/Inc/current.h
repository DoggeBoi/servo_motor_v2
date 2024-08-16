
#ifndef INC_CURRENT_H_
#define INC_CURRENT_H_

#include "stm32f3xx_hal.h"

#define MAX_CURRENT 400;   		// mA



typedef struct {

	uint16_t maxCurrent;

} CURRENT_CTRL;


/*   Current controller initialisation   */
void currentInit(CURRENT_CTRL *cCtrl);


/*   Calculate limited dutyCycle value   */
int16_t currentLimit(CURRENT_CTRL *cCtrl, int16_t current, int16_t temperature, int16_t dutyCycle);		// Current in units of 1/10000 of AMP


#endif /* INC_CURRENT_H_ */
