
#include "commands.h"
#include "stm32f3xx_hal.h"
#include "servo.h"
#include "extvar.h"
#include "can.h"
#include "cmsis_os.h"

void servoCommandAssignInit(SERVO_CONTROL *servo) {

	/*   Assign command function pointers   */
	servo->OperationFunctions[0] 	= (void*)commandStartup;
	servo->OperationFunctions[1] 	= (void*)commandShutdown;
	servo->OperationFunctions[2] 	= (void*)commandTorqueEnable;
	servo->OperationFunctions[3] 	= (void*)commandTorqueDisable;
	servo->OperationFunctions[4]	= (void*)commandConnectionAKN;
	servo->OperationFunctions[5]	= (void*)commandReadSingle;
	servo->OperationFunctions[6]	= (void*)commandWriteSingle;
	servo->OperationFunctions[7]	= (void*)commandWriteStandard;

}


void commandStartup(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	servo->startupConfrimation = 1;

}


void commandShutdown(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Deactivate can interrupts to disable can communication   */
	HAL_CAN_DeactivateNotification(&servo->can.canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&servo->can.canHandle, CAN_IT_RX_FIFO1_MSG_PENDING);

	/*   Send response   */
	dataShutdownAKN(servo);

	/*   Permanently suspend all tasks   */
	vTaskSuspendAll();							// Disables entire device

}


void commandTorqueEnable(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Servo  command    */
	torqueEnable(servo, RxBuf[0] );		// First byte is direction

	/*   Send response   */
	dataTorqueAKN(servo);

}


void commandTorqueDisable(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Servo  command    */
	torqueDisable(servo);

	/*   Send response   */
	dataTorqueAKN(servo);

}


void commandConnectionAKN(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Change variables to confirm communication received   */
	servo->commsConfrimation = 1;

}


void commandReadSingle(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Initialise address variable   */
	uint8_t adress = RxBuf[0];


	/*   If address is inside range   */
	if ( adress <= 46 ) {

		/*   If variable is 16-bit number   */
		if ( ((EXT_VAR *)(servo->externalVariableList[adress]))->byteSize == 2 ) {

			/*   Initialise transmission buffer   */
			uint8_t TxBuf[3];

			/*   Set read address byte   */
			TxBuf[0] = adress;

			uint16_t readData =  *(uint16_t*)((EXT_VAR *)(servo->externalVariableList[adress]))->variable;

			/*   Set data bytes in array   */
			TxBuf[1] = ( readData >> 8 ) & 0xFF;
			TxBuf[2] =   readData        & 0xFF;

			/*   Send response frame   */
			dataReadSingle(servo, TxBuf, 3, priority);

		}

		/*   If variable is 8-bit number   */
		else {

			/*   Initialise transmission buffer   */
			uint8_t TxBuf[2];

			/*   Set read address byte   */
			TxBuf[0] = adress;

			uint8_t readData =  *(uint8_t*)((EXT_VAR *)(servo->externalVariableList[adress]))->variable;

			/*   Set data bytes in array   */
			TxBuf[1] = readData;

			/*   Send response frame   */
			dataReadSingle(servo, TxBuf, 2, priority);

		}

	}

}


void commandWriteSingle(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	/*   Initialise address variable   */
	uint8_t adress = RxBuf[0];

	/*   If address is inside range and has write privilege   */
	if ( ( adress <= 46 )  && ( ((EXT_VAR *)(servo->externalVariableList[adress]))->writePrivilege == 1 ) ) {

			/*   If variable is 16-bit number   */
			if ( ((EXT_VAR *)(servo->externalVariableList[adress]))->byteSize == 2 ) {

				*(uint16_t*)((EXT_VAR *)(servo->externalVariableList[adress]))->variable = ( ( RxBuf[1] << 8 ) | RxBuf[2] );

			}

			/*   If variable is 8-bit number   */
			else {

				*(uint8_t*)((EXT_VAR *)(servo->externalVariableList[adress]))->variable = RxBuf[2];

			}

	}

	/*   Send back read message   */
	commandReadSingle(servo, RxBuf, cmdId, priority);		// Value wont have changed if above if statement  in false;

}


void commandWriteStandard	(SERVO_CONTROL *servo, uint8_t *RxBuf, uint8_t cmdId, uint8_t priority) {

	servo->goalPosition 			= RxBuf[0];
	servo->profile.maxAcceleration 	= RxBuf[1];
	servo->profile.maxVelocity 		= RxBuf[2];

}


/*   Output data functions   */
void dataStartupReady		(SERVO_CONTROL *servo) {

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[3];

	/*   Set read torqueEnable byte   */
	TxBuf[0] = servo->id;				// Variable 18 is torqueEnable

	TxBuf[1] = servo->firmwareVer;		// Variable 18 is motionDirection

	TxBuf[2] = errorCombine( &servo->error );

	/*   Send data   */
	CAN_SendDataFrame(&servo->can, TxBuf, 3, PRIORITY_CRITICAL, DATA_SHUTDOWN_CONF);

}


void dataShutdownAKN		(SERVO_CONTROL *servo) {

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[1];

	TxBuf[0] = errorCombine( &servo->error );

	/*   Send data   */
	CAN_SendDataFrame(&servo->can, TxBuf, 1, PRIORITY_CRITICAL, DATA_SHUTDOWN_CONF);

}


void dataTorqueAKN			(SERVO_CONTROL *servo) {

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[2];

	/*   Set read torqueEnable byte   */
	TxBuf[0] = servo->torqueEnable;

	TxBuf[1] = servo->motionDirection;

	/*   Send response frame   */
	CAN_SendDataFrame(&servo->can, TxBuf, 3, PRIORITY_HIGH, DATA_TORQUE_CONF);

}


void dataConnectionREQ		(SERVO_CONTROL *servo) {

	/*   Send data   */
	CAN_SendDataFrame(&servo->can, NULL, 0, PRIORITY_CRITICAL, DATA_CONNECTION_CHECK);

}


void dataReadSingle			(SERVO_CONTROL *servo, uint8_t *TxBuf, uint8_t dataLenght, uint8_t priority) {

	/*   Send data   */
	CAN_SendDataFrame(&servo->can, TxBuf, dataLenght, priority, DATA_SINGLE_READ);

}

//////////////////////////////////
void dataStandardRead		(SERVO_CONTROL *servo, uint8_t type) {



}


void configJoin(SERVO_CONTROL *servo, uint8_t *TxBuf, uint8_t *addressList, uint8_t listLenght) {

	uint8_t i = 0;

	while ( i < listLenght ) {
		if ( ((EXT_VAR *)(servo->externalVariableList[i]))->byteSize == 2 ) {

			TxBuf[i] 	= ( ( *(uint16_t*)((EXT_VAR *)(servo->externalVariableList[i]))->variable ) >> 8 ) & 0xFF;
			TxBuf[i+1] 	=   ( *(uint16_t*)((EXT_VAR *)(servo->externalVariableList[i]))->variable ) 	  & 0xFF;


			i += 2;

		}
		else {

			TxBuf[i] 	=  *(uint8_t*)((EXT_VAR *)(servo->externalVariableList[i]))->variable;

			i += 1;

		}

	}

}


/*   Higher level CAN-bus functions   */
void processCanMessages(SERVO_CONTROL *servo, uint32_t RxFifo) {

	/*   Initialise message data variables   */
	uint8_t 			RxBuf[8];
	uint8_t 			operationId;
	uint8_t				senderId;
	uint8_t				priority;

	/*   While new messages are available   */
	while (  HAL_CAN_GetRxFifoFillLevel(servo->can.canHandle, RxFifo) > 0 ) {

		/*   Get message   */
		CAN_GetFrame(&servo->can, RxFifo, RxBuf, &senderId, &operationId, &priority);

		/*   Check if operation id is within range and operation is defined    */
		if ( operationId <= HIGHEST_COMMAND_ID ) {

			/*   Call operation function   */
			OperationFucntionPointer function = ( OperationFucntionPointer ) servo->OperationFunctions[operationId];

			function(servo, RxBuf, operationId, priority);

		}

	}

}


void readStandard(SERVO_CONTROL *servo, uint8_t priority, uint8_t dataPackId) {

	/*   Initialise address lists   */
	uint8_t read1[] = {3, 27, 28, 31};
	uint8_t read2[] = {6, 7, 8, 9};
	uint8_t read3[] = {12, 13, 29, 45, 46};
	uint8_t read4[] = {32, 34, 36 ,38};

	/*   Initialise select list parameters   */
	uint8_t *selectedList;
	uint8_t  selectedListLenght;

	/*   Select list based on input parameter   */
	switch (dataPackId) {

		case 0:
			selectedList	 	= read1;
			selectedListLenght 	= 4;
			break;

		case 1:
			selectedList	 	= read2;
			selectedListLenght 	= 4;
			break;

		case 2:
			selectedList	 	= read3;
			selectedListLenght 	= 5;
			break;

		case 3:
			selectedList	 	= read4;
			selectedListLenght 	= 4;
			break;

		default:
			return;

	}

	/*   Initialise transmission buffer   */
	uint8_t TxBuf[8];

	/*   Read back written values   */
	configJoin(servo, TxBuf, selectedList, selectedListLenght);

	/*   Send back response frame to confirm change   */
	CAN_SendDataFrame(&servo->can, TxBuf, ( sizeof( TxBuf ) / sizeof( uint8_t ) ), priority, 4 + dataPackId);   // Sets data/operation identifier based on dataPackId

}
