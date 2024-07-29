
#ifndef INC_EXTVAR_H_
#define INC_EXTVAR_H_

#include "servo.h"


/*   External accessible variable   */
typedef struct {

	void* 		variable;			// Pointer to variable
	uint8_t		writePrivilege;		// 1 if variable can be changed externally
	uint8_t		byteSize;			// Variable size in byte

} EXT_VAR;


/*   Servo external variables initialisation   */
void servoExtVariablesInit(SERVO_CONTROL *servo);


#endif /* INC_EXTVAR_H_ */
