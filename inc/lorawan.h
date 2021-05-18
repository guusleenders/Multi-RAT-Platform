#ifndef __LORAWAN
#define __LORAWAN

#include <stdint.h>
#include <stdbool.h>
#include "lora.h"
#include "LoRaMac.h"
#include "hw.h"

#include "energy.h"

#ifdef __cplusplus
extern "C" {
#endif
	
// --------------------------- LORA DEFINITIONS --------------------------------
#define LORAWAN_MAX_BAT   254

//#define CAYENNE_LPP 										// CAYENNE_LPP is myDevices Application server.
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99

#define LORAWAN_PAYLOAD_SIZE							15
#define LORAWAN_ADR_STATE 								LORAWAN_ADR_ON 	// LoRaWAN Adaptive Data Rate; Please note that when ADR is enabled the end-device should be static
#define LORAWAN_DEFAULT_DATA_RATE 				DR_0 						// LoRaWAN Default data Rate Data Rate; Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
#define LORAWAN_APP_PORT 									2 							// LoRaWAN application port; do not use 224. It is reserved for certification
#define LORAWAN_DEFAULT_CLASS 						CLASS_A 				// LoRaWAN default endNode class port
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE LORAWAN_UNCONFIRMED_MSG // LoRaWAN default confirm state
#define LORAWAN_APP_DATA_BUFF_SIZE 				20 							// User application data buffer size

// ---------------------------- LORA FUNCTIONS ---------------------------------
void initLoRaWAN(void);													// Init lora modem
void registerLoRaWAN(void);											// Register to LoRaWAN network (join mechanism)
void sendLoRaWAN(void); 													// Send LoRaWAN data

void LORA_RxData(lora_AppData_t *AppData);				// Callback when LoRa endNode has received a frame
void LORA_HasJoined(void);												// Callback when LoRa endNode has just joined
void LORA_ConfirmClass(DeviceClass_t Class);			// Callback when LoRa endNode has just switch the class
void LORA_TxNeeded(void);												// Callback when server needs endNode to send a frame
uint8_t LORA_GetBatteryLevel(void);							// Callback to get the battery level in % of full charge (254 full charge, 0 no charge)
void LORA_Done(void);	

void LoraMacProcessNotify(void);									// TX timer callback function

extern LoraFlagStatus AppProcessRequest;
extern LoraFlagStatus LoraMacProcessRequest;


#ifdef __cplusplus
}
#endif

#endif /* __LORAWAN */
