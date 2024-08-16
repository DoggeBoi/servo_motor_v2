
#include "analogue.h"
#include <math.h>


/*   Sensor Initialisation   */
uint8_t adcSensorInit(ADC *sensor, ADC_Conversion_Func conversionFunction, ADC_HandleTypeDef *adcHandle) {

	/*   Initialise status variable   */
	uint8_t status = 0;

	/*   Initialise ADC handle   */
	sensor->adcHandle = adcHandle;

	/*   Connect sensor specific conversion function   */
	sensor->conversionFunction = conversionFunction;

	/*   Initialise base ADC low pass filter coefficient   */
	sensor->extIIRFilterCoefficient = ADC_IIRFILTER_COEFFICIENT;

	/*	Calibrate ADC   */
	status += HAL_ADCEx_Calibration_Start(adcHandle, ADC_SINGLE_ENDED);

	/*   Calibration completion time   */
	HAL_Delay(10);

	/*   Return status variable   */
	return status;

}


/*   Sensor ADC polling update   */
void adcSensorUpdate(ADC *sensor) {

	/*   Calculate internal filter value from external    */
	float alpha				= sensor->extIIRFilterCoefficient / 255.0f;					// Convert Filter-coefficient to alpha

	/*   Start ADC   */
	HAL_ADC_Start(sensor->adcHandle);

	/*   ADC polling conversion   */
	HAL_ADC_PollForConversion(sensor->adcHandle, ADC_MAX_DELAY);

	/*   Get ADC raw data value   */
	sensor->rawData = HAL_ADC_GetValue(sensor->adcHandle);

	/*   Stop ADC   */
	HAL_ADC_Stop(sensor->adcHandle);

	/*   Convert and store data   */
	sensor->converData = ( int16_t ) ( ( 1.0f - alpha ) * sensor->conversionFunction(sensor->rawData) + alpha * sensor->converData ) ;

}


/*   Sensor range check   */
uint8_t adcSensorRangeCheck(ADC *sensor, uint8_t *flag, uint16_t maxValue, uint16_t minValue) {

	if ( sensor->converData >= maxValue || sensor->converData <= minValue) {

		*flag = 1;
		return 1;

	}

	return 0;

}

/*   Sensor specific conversion function   */
int16_t rawToIntTemp(uint16_t rawData) {

	/*   Initialise variables   */
	float voltage;
	float temp;

	/*   Data to temperature conversion   */
	voltage = ( rawData / 4095.0f ) * 3.3f;							// Calculate voltage from ADC measurement
	temp	= ( ( ( 1.43f - voltage ) / 0.0043f ) + 15.0f );

	/*   Convert and return data   */
	return (int16_t) ( temp * 100.0f );								//  1/100 of a degree

}

int16_t rawToMotorTemp(uint16_t rawData) {

	/*   Initialise variables   */
	float voltage;
	float impedance;
	float temp;

	/*   Data to temperature conversion   */
	voltage 	= ( rawData / 4095.0f ) * 3.3f;							// Calculate voltage from ADC measurement
	impedance	= ( 100000.0f / ( ( 3.3f / voltage ) - 1.0f ) );		// NTC resistance from voltage splitter
	temp		= ( 1.0f / ( ( ( log( impedance / 100000.0f ) ) / ( 3950.0f ) ) + ( 1.0f / 298.15f ) ) ) - 273.15f;											// Calculate NTC temp from resistance

	return (int16_t) ( temp * 100.0f );									//  1/100 of a degree

}

int16_t rawToVoltage(uint16_t rawData) {

	/*   Initialise variables   */
	float voltage;
	float batteryVoltage;

	/*   Data to temperature conversion   */
	voltage 	= ( rawData / 4095.0f ) * 3.3f;							// Calculate voltage from ADC measurement
	batteryVoltage = voltage * ( ( 1.5f + 10.0f ) / 1.5f );

	return (int16_t) ( batteryVoltage * 1000.00f );						//  mV

}

int16_t rawToCurrent(uint16_t rawData) {

	/*   Initialise variables   */
	float voltage;
	float current;

	/*   Data to temperature conversion   */
	voltage 	= ( rawData / 4095.0f ) * 3.3f;							// Calculate voltage from ADC measurement
	current		=  -3.544f * voltage + 5.84584f - 0.023f;

	return (int16_t) ( current * 10000.00f );							//  1/10000 of AMP
}

