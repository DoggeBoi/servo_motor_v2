
#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_


#include "stm32f3xx_hal.h"
#include "servo.h"
#include "extvar.h"

/*   Highest command ID   */
#define HIGHEST_COMMAND_ID			7

/*   Data output packet ID's   */
#define DATA_STARTUP_READY 			0
#define DATA_SHUTDOWN_CONF			1
#define DATA_TORQUE_CONF			2
#define DATA_CONNECTION_CHECK		3
#define DATA_SINGLE_READ			4
#define DATA_STANDARD_READ_BASE		5

/*   Command input packet ID's   */
#define COMMAND_STARTUP_ENGAGE		0
#define COMMAND_TORQUE_ON			1
#define COMMAND_TORQUE_OFF			2
#define COMMAND_SHUTDOWN			3
#define COMMAND_CONNECTION_CHECK	4
#define COMMAND_SINGLE_READ			5
#define COMMAND_SINGLE_WRITE		6
#define COMMAND_STANDARD_WRITE		7


void servoCommandAssignInit(SERVO_CONTROL *servo);

void configJoin(SERVO_CONTROL *servo, uint8_t *TxBuf, uint8_t *addressList, uint8_t listLenght);

void processCanMessages(SERVO_CONTROL *servo, uint32_t RxFifo);


/*   Input command functions   */
void commandStartup 		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandStartup 		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandShutdown		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandTorqueEnable	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandTorqueDisable	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandConnectionAKN	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandReadSingle		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandWriteSingle		(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);

void commandWriteStandard	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority);


/*   Output data functions   */
void dataStartupReady		(SERVO_CONTROL *servo);

void dataShutdownAKN		(SERVO_CONTROL *servo);

void dataTorqueAKN			(SERVO_CONTROL *servo);

void dataConnectionREQ		(SERVO_CONTROL *servo);

void dataReadSingle			(SERVO_CONTROL *servo, uint8_t *TxBuf, uint8_t dataLenght, uint8_t priority);

void dataStandardRead		(SERVO_CONTROL *servo, uint8_t type);		// Variable data ID based on contents


#endif /* INC_COMMANDS_H_ */
