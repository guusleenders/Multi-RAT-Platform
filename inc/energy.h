#ifndef __ENERGY
#define __ENERGY

#include <stdint.h>
#include <stdbool.h>
#include "ltc2942.h"

#include "vcom2.h"

typedef enum {
		INIT = (uint8_t)0, 
		REGISTRATION = (uint8_t)1,
		SEND = (uint8_t)2
} MESSAGE_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct InitEnergy_t {
	 uint8_t deviceID; 
	 uint16_t bootID;
	 uint16_t packetNumber;
	 MESSAGE_TYPE packetType; 
   uint16_t  nbiotEnergy;
   uint16_t  sigfoxEnergy;
	 uint16_t lorawanEnergy;
} InitEnergy_t;  



typedef struct Energy_t {
	uint8_t deviceID; 
	uint16_t bootID;
	uint16_t packetNumber;
  uint16_t  nbiotEnergy;
	char nbiotConditions[50];
	MESSAGE_TYPE nbiotPacketType; 
  uint16_t  sigfoxEnergy;
	char sigfoxConditions[10];
	MESSAGE_TYPE sigfoxPacketType; 
	uint16_t lorawanEnergy;
	char lorawanConditions[10];
	MESSAGE_TYPE lorawanPacketType; 
} Energy_t;  


extern InitEnergy_t initEnergyStruct; 
extern Energy_t energyStruct; 
extern uint16_t energy;


void initEnergyMeasurement(void);
void startEnergyMeasurement(LTC2942_SENSOR sensor);
uint16_t stopEnergyMeasurement(LTC2942_SENSOR sensor);


#ifdef __cplusplus
}
#endif

#endif /* __ENERGY */
