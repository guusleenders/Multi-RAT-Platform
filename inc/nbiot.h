#ifndef __NBIOT
#define __NBIOT

#include <stdint.h>
#include <stdbool.h>
#include "bg96.h"
#include "vcom2.h"
#include "string.h"

#include "energy.h"

extern BG96_Powerdown_t powerStatus;

// ---------------------------- NBIOT FUNCTIONS ---------------------------------
void initNBIoT(void);														// Init NB-IoT modem
void registerNBIoT(void);												// Register to NB-IoT network
int8_t _sendNBIoT(bool sendingMeasuredEnergy, char * payload);
int8_t sendNBIoT(void);														// Send NB-IoT data
void sendEnergyStruct( void );
void stopEnergyMeasurementNBIoT( void );								// Stop NB-IoT energy measurement

#ifdef __cplusplus
}
#endif

#endif /* __NBIOT */
