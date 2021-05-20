// -------------------------- ENERGY MEASUREMENT FUNCTIONS -------------------------------

#include "energy.h"

uint16_t energy = 0;
InitEnergy_t initEnergyStruct; 
Energy_t energyStruct; 

void initEnergyMeasurement(void){
	uint16_t testvoltage = 0;
	//#if defined(LORAWAN) || defined(SIGFOX)
	LTC2942_Init(LTC2942_LRWAN);
	LTC2942_SetPrescaler(LTC2942_LRWAN, PRESCALAR_M_1);
	LTC2942_SetAlertConfig(LTC2942_LRWAN, ALERT_DISABLED);
	LTC2942_SetAccumulatedCharge(LTC2942_LRWAN, 0);
	LTC2942_SetShutdown(LTC2942_LRWAN, 0);
	
	testvoltage = LTC2942_GetVoltage(LTC2942_LRWAN)*1000;
	PRINTF_LN("Voltage LRWAN: %d mV", testvoltage);
	
	LTC2942_SetShutdown(LTC2942_LRWAN, 1);
	//#endif
	
	//#ifdef NBIOT
	LTC2942_Init(LTC2942_NBIOT);
	uint8_t test = LTC2942_GetControl(LTC2942_NBIOT);
	PRINTF_LN("Test control reg: %d", test);
	LTC2942_SetPrescaler(LTC2942_NBIOT, PRESCALAR_M_1);
	LTC2942_SetAlertConfig(LTC2942_NBIOT, ALERT_DISABLED);
	LTC2942_SetAccumulatedCharge(LTC2942_NBIOT, 0);
	LTC2942_SetShutdown(LTC2942_NBIOT, 0);
	
	testvoltage = LTC2942_GetVoltage(LTC2942_NBIOT)*1000;
	PRINTF_LN("Voltage NBIOT: %d mV", testvoltage);
	
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
	//#endif
	
}

void startEnergyMeasurement(LTC2942_SENSOR sensor){
	LTC2942_SetShutdown(sensor, 0);
	LTC2942_SetAccumulatedCharge(sensor, 0);
	energy = LTC2942_GetmAh(sensor)*10000;
}

uint16_t stopEnergyMeasurement(LTC2942_SENSOR sensor){
	energy = LTC2942_GetmAh(sensor)*10000 - energy;
	float voltage = LTC2942_GetVoltage(sensor);
	uint16_t uwh = (uint16_t) (energy * voltage);
	LTC2942_SetShutdown(sensor, 1);
	return uwh;
	//return 0;
}