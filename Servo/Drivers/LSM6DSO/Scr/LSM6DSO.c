
#include "LSM6DSO.h"
#include <math.h>

uint8_t LSM6DSO_Init(LSM6DSO *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t sampleTime) {

	/*   Store parameters in struct   */
	imu->spiHandle 					= 	spiHandle;
	imu->csPort						=	csPort;
	imu->csPin						=	csPin;

	/*   Set base parameters   */
	imu->extCFilterCoefficient 		= LSM6DSO_CFILTER_COEFFICIENT;
	imu->extIIRFilterCoefficient	= LSM6DSO_IIRFILTER_COEFFICIENT;

	/*   Sample time   */
	imu->extSampleTime 				= sampleTime;

	/*   Initialise previous calculation terms   */
	imu->intAngleX 					= 0;
	imu->intAngleY 					= 0;


	/*   Chip select default inactive high   */
	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_SET);

	/*	IMU startup   */
	HAL_Delay(20);

	/*   Check connection   */
	if ( LSM6DSO_ConnectionCheck(imu) >= 1 ) return LSM6DSO_SPI_FAIL;

	/*   Activate accelerometer   */
	LSM6DSO_WriteRegister(imu, LSM6DSO_CTRL1_XL, 0x71);		// Accelerometer 833Hz, +-2g, secondary LPF filtering

	/*   Activate gyroscope   */
	LSM6DSO_WriteRegister(imu, LSM6DSO_CTRL2_G, 0x74);		// Gyroscope 833Hz +-500 dps

	/*   If communication is successful   */
	return LSM6DSO_SPI_SUCCESS;
}


void LSM6DSO_TransmitReceive(LSM6DSO *imu, uint8_t *TxBuf, uint8_t *RxBuf, uint8_t lenght) {

	/*   Transmit and pulse chip select   */
	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(imu->spiHandle, TxBuf, RxBuf, lenght, LSM6DSO_MAX_DELAY);

	HAL_GPIO_WritePin(imu->csPort, imu->csPin, GPIO_PIN_SET);

}


void LSM6DSO_ReadRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t *data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[2] = {0};
	uint8_t RxBuf[2] = {0};

	/*   Set read bit and register address   */
	TxBuf[0]  = ( regAddr | ( 1 << 7 ) );			// R/W bit 1

	/*   Transmit and receive SPI package   */
	LSM6DSO_TransmitReceive(imu, TxBuf, RxBuf, 2);

	/*   Save read data to data variable   */
	*data = RxBuf[1];

}


void LSM6DSO_WriteRegister(LSM6DSO *imu, uint8_t regAddr, uint8_t data) {

	/*   Initialise SPI buffers   */
	uint8_t TxBuf[2] = {0};
	uint8_t RxBuf[2] = {0};

	/*   Set write bit and register address   */
	TxBuf[0]  = ( regAddr | ( 0 << 7 ) );			// R/W bit 0

	/*   Set write data   */
	TxBuf[1]  = data;

	/*   Transmit and receive SPI package   */
	LSM6DSO_TransmitReceive(imu, TxBuf, RxBuf, 2);

}


void LSM6DSO_ReadAccelerometer(LSM6DSO *imu) {

	/*   Initialise raw data registers   */
	int16_t registerX;
	int16_t registerY;
	int16_t registerZ;

	/*   Calculate internal filter value from external    */
	float alpha				= imu->extIIRFilterCoefficient / 255.0f;					// Convert Filter-coefficient to alpha

	/*   Conversion factor calculation ( m/s^2 )   */
	float convertionFactor  = 9.81f * LA_So_2g;		// For +-2g

	/*   Read accelerometer data   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_L_A, ( (uint8_t *)&registerX ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_H_A, ( (uint8_t *)&registerX + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_A, ( (uint8_t *)&registerY ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_A, ( (uint8_t *)&registerY + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTZ_L_A, ( (uint8_t *)&registerZ ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTZ_H_A, ( (uint8_t *)&registerZ + 1 ) );

	/*   Update values in struct   */
	imu->acclerationX = accelerationLimiter( ( 1.0f - alpha ) * registerX * convertionFactor + alpha * imu->acclerationX );
	imu->acclerationY = accelerationLimiter( ( 1.0f - alpha ) * registerY * convertionFactor + alpha * imu->acclerationY );
	imu->acclerationZ = accelerationLimiter( ( 1.0f - alpha ) * registerZ * convertionFactor + alpha * imu->acclerationZ );

}


void LSM6DSO_ReadGyroscope(LSM6DSO *imu) {

	/*   Initialise raw data registers   */
	int16_t registerX;
	int16_t registerY;
	int16_t registerZ;

	/*   Conversion factor calculation ( radians/s )   */
	float convertionFactor  = G_So_500dps * ( 3.141592f / 180.0f );			// For +-500dps

	/*   Read gyroscope data   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_L_G, ( (uint8_t *)&registerX ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTX_H_G, ( (uint8_t *)&registerX + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_L_G, ( (uint8_t *)&registerY ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTY_H_G, ( (uint8_t *)&registerY + 1 ) );

	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTZ_L_G, ( (uint8_t *)&registerZ ) );
	LSM6DSO_ReadRegister(imu, LSM6DSO_OUTZ_H_G, ( (uint8_t *)&registerZ + 1 ) );

	/*   Update values in struct   */
	imu->angularVelocityX = registerX * convertionFactor;
	imu->angularVelocityY = registerY * convertionFactor;
	imu->angularVelocityZ = registerZ * convertionFactor;

}


void LSM6DSO_ReadSensors(LSM6DSO *imu) {

	/*   Initialise raw data variable   */
	uint8_t registerData;

	/*   Read status register   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_STATUS_REG, &registerData);

	/*   If data ready   */
	if ( registerData & 0x1 ) {

		LSM6DSO_ReadAccelerometer(imu);

	}

	if ( registerData & 0x2 ) {

			LSM6DSO_ReadGyroscope(imu);

		}

}


/*   Complementary filter   */
void LSM6DSO_EstimateOrientation(LSM6DSO *imu) {

	/*   Conversion factor calculation ( int16_t )   */
	float convertionFactor  = ( 32768.0f / 3.141592f );							// For +- 1 radian

	/*   Calculate internal filter value from external    */
	float alpha			= imu->extCFilterCoefficient	/ 2550.0f;					// Convert Filter-coeffiectnt to alpha
	float time			= imu->extSampleTime 			/ 1000.0f;					// Convert ms to s

	/*   Inertial frame of reference angle estimation   */
	float accelAngleX 	= atan2f(imu->acclerationY , imu->acclerationZ);		// Roll calculation
	float accelAngleY 	= asinf( imu->acclerationX 	/ 9.81f );					// Pitch calculation

	/*   Calculate Euler-rates   */
	float gyroAngleX	= imu->angularVelocityX + sinf(imu->intAngleX) * tanf(imu->intAngleY) * imu->angularVelocityY + sinf(imu->intAngleX) * tanf(imu->intAngleY) * imu->angularVelocityZ;
	float gyroAngleY 	= imu->angularVelocityY * cosf(imu->intAngleX) - sinf(imu->intAngleX) * imu->angularVelocityZ;

	/*   Complementary filter calculation   */
	imu->intAngleX		= accelAngleX * alpha + ( 1.0f - alpha ) * ( imu->intAngleX + time * gyroAngleX );
	imu->intAngleY 		= accelAngleY * alpha + ( 1.0f - alpha ) * ( imu->intAngleY + time * gyroAngleY );

	/*   Convert radians to int16   */
	imu->extAngleX = imu->intAngleX * convertionFactor;
	imu->extAngleY = imu->intAngleY * convertionFactor;

}


/*   Acceleration clamping limiter   */
float accelerationLimiter(float acceleration) {

	/*   If acceleration is above gravity   */
	if ( fabs( acceleration ) > 9.80f ) {

		return ( 9.80f * ( ( acceleration >= 0 ) ? 1 : -1 ) );

	}

	/*   If acceleration is within limit   */
	return acceleration;

}


/*   Check connection   */
uint8_t LSM6DSO_ConnectionCheck(LSM6DSO *imu) {

	/*   Initialise data variable   */
	uint8_t registerData;

	/*   Read register with identification   */
	LSM6DSO_ReadRegister(imu, LSM6DSO_WHO_AM_I, &registerData);

	/*   If communication has failed   */
	if ( registerData != 0x6C ) return LSM6DSO_SPI_FAIL;

	/*   If communication is successful   */
	return LSM6DSO_SPI_SUCCESS;

}
