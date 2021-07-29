#ifndef __ENERGY
#define __ENERGY

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ltc2942.h"

#include "vcom.h"

typedef enum {
		INIT = (uint8_t)0, 
		REGISTRATION = (uint8_t)1,
		SEND = (uint8_t)2
} MESSAGE_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Energy_t {
	uint8_t 	general_deviceID; 
	uint16_t 	general_bootID;
	
	uint16_t  nbiot_packetNumber;
  uint16_t  nbiot_energy;
	char 			nbiot_conditions[150];
	uint8_t 	nbiot_initStatus; 
	uint8_t		nbiot_closeStatus;
	uint32_t	nbiot_closeTime;
	uint16_t	nbiot_payloadSize;
	
	uint16_t  sigfox_packetNumber;
  uint16_t  sigfox_energy;
	char 			sigfox_conditions[10];
	uint8_t 	sigfox_initStatus;
	uint16_t	sigfox_payloadSize;
	
	uint16_t  lorawan_packetNumber;
  uint16_t  lorawan_energy;
	char 			lorawan_conditions[10];
	uint8_t 	lorawan_initStatus;
	uint16_t	lorawan_payloadSize;
} Energy_t;  

extern Energy_t energyStruct; 

extern uint16_t energy;


void initEnergyMeasurement(void);
void startEnergyMeasurement(LTC2942_SENSOR sensor);
uint16_t stopEnergyMeasurement(LTC2942_SENSOR sensor);

void clearEnergyStruct(bool totalClear);

#ifdef __cplusplus
}
#endif

#endif /* __ENERGY */
