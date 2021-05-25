// -------------------------------- LORA FUNCTIONS -------------------------------------

#include "lorawan.h"

static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE]; // User application data

//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 }; // User application data structure
lora_AppData_t AppData = { AppDataBuff,  0, 0 };

// Load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { LORA_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify,
																								LORA_Done
                                              };
LoraFlagStatus LoraMacProcessRequest = LORA_RESET;
LoraFlagStatus AppProcessRequest = LORA_RESET;
bool isConnectedLoRaWAN = false;
bool loradone = false; 
																							
static uint8_t AppLedStateOn = RESET; 		// Specifies the state of the application LED

//static TimerEvent_t TxTimer;
#ifdef USE_B_L072Z_LRWAN1
TimerEvent_t TxLedTimer; 					// Timer to handle the application Tx Led to toggle
void OnTimerLedEvent(void *context);
#endif

// Initialises the Lora Parameters
 LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE,
                                     LORAWAN_DEFAULT_DATA_RATE,
                                     LORAWAN_PUBLIC_NETWORK
                                    };

void initLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	// ---------- Init LoRa hardware, functions,... ---------- 
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
	#ifdef DEBUG
	PRINTF_LN("- Initialised");
	#endif 
	
	//initEnergyStruct.packetType = INIT;
	//initEnergyStruct.lorawanEnergy = stopEnergyMeasurement(LTC2942_LRWAN);
}

void registerLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	// ---------- Join LoRaWAN network ---------- 
	LORA_Join();
	#ifdef DEBUG
	PRINTF_LN("- Join procedure done");
	#endif 
	
	// Stop energy measurement in hasjoined function
}

void sendLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	if(!isConnectedLoRaWAN){
		energyStruct.lorawan_initStatus = 0;
		initLoRaWAN();
		registerLoRaWAN();
	}else{
		energyStruct.lorawan_initStatus = 1; 
	}
	
	loradone = false; 
	
	// ---------- Check if joined ---------- 
  if (LORA_JoinStatus() != LORA_SET) {
    /*Not joined, try again later*/
		#ifdef DEBUG
		PRINTF_LN("- Not joined, try again later");
		#endif
    LORA_Join();
    return;
  }

	// ---------- Compose message ---------- 
  uint32_t i = 0;
  //batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */
  AppData.Port = LORAWAN_APP_PORT;
  AppData.Buff[i++] = energyStruct.general_deviceID;
  AppData.Buff[i++] = energyStruct.general_bootID;
  AppData.Buff[i++] = (energyStruct.lorawan_packetNumber >> 8) & 0xFF;;
	AppData.Buff[i++] = energyStruct.lorawan_packetNumber  & 0xFF;
	
	for(uint8_t p = 0; p < LORAWAN_PAYLOAD_SIZE-4; p++){
		AppData.Buff[i++] = 0x00;
	}
  AppData.BuffSize = i;
	
	// ---------- Send message ---------- 
	#ifdef DEBUG
	PRINTF_LN("- Sending packet");
	#endif
  LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	#ifdef DEBUG
	PRINTF_LN("- LoRaWAN send command done");
	#endif
	
	// Stop energy measurement in loradone function
}

void LoraMacProcessNotify(void){
  LoraMacProcessRequest = LORA_SET;
}

void LORA_HasJoined(void){
	// ---------- LoRa has joined callback ---------- 
	#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF_LN("- Joined done");
	#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
	isConnectedLoRaWAN = true;

	/*uint16_t uwh = stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.lorawanPacketType = REGISTRATION;
	energyStruct.lorawanEnergy = uwh;*/
	#ifdef DEBUG
	PRINTF("\r\n||LoRa join energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ)||\r\n", energy, uwh, uwh*3.6f);
	#endif

}

void LORA_RxData(lora_AppData_t *AppData){
	// ---------- LoRa Data is comming in ---------- 
	// TODO: this code is standard user code 
	#ifdef DEBUG
  PRINTF_LN("- LoRaWAN packet received on port %d\n\r", AppData->Port);
	#endif
	
  switch (AppData->Port){
    case 3:
      // this port switches the class
      if (AppData->BuffSize == 1){
        switch (AppData->Buff[0]){
          case 0:
          {
            LORA_RequestClass(CLASS_A);
            break;
          }
          case 1:
          {
            LORA_RequestClass(CLASS_B);
            break;
          }
          case 2:
          {
            LORA_RequestClass(CLASS_C);
            break;
          }
          default:
            break;
        }
      }
      break;
    case LORAWAN_APP_PORT:
      if (AppData->BuffSize == 1){
        AppLedStateOn = AppData->Buff[0] & 0x01;
        if (AppLedStateOn == RESET){
          PRINTF("LED OFF\n\r");
          LED_Off(LED_BLUE) ;
        }
        else{
          PRINTF("LED ON\n\r");
          LED_On(LED_BLUE) ;
        }
      }
      break;
    case LPP_APP_PORT:
    {
      AppLedStateOn = (AppData->Buff[2] == 100) ?  0x01 : 0x00;
      if (AppLedStateOn == RESET){
        PRINTF("LED OFF\n\r");
        LED_Off(LED_BLUE) ;

      }
      else{
        PRINTF("LED ON\n\r");
        LED_On(LED_BLUE) ;
      }
      break;
    }
    default:
      break;
  }
}

void LORA_ConfirmClass(DeviceClass_t Class){
	#ifdef DEBUG
  PRINTF("- Switching to class %c done\n\r", "ABC"[Class]);
	#endif

  // Optional: informs the server that switch has occurred ASAP
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

void LORA_TxNeeded(void){
	// ---------- LoRa TX Needed ---------- 
	#ifdef DEBUG
	PRINTF_LN("- LoRa TX Needed");
	#endif
	
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

bool isDoneLoRaWAN(void){
	return loradone;
}
void LORA_Done(void){
	// ---------- LoRa is done, stop everything ---------- 
	#ifdef DEBUG
	PRINTF_LN("- LoRa DONE");
	#endif
	loradone = true;
	uint16_t uwh = 	stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.lorawan_energy = uwh;
	PRINTF("\r\n||LoRa energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
	
	energyStruct.lorawan_packetNumber++; 
}

uint8_t LORA_GetBatteryLevel(void){
  uint16_t batteryLevelmV;
  uint8_t batteryLevel = 0;

  batteryLevelmV = HW_GetBatteryLevel();

  // Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) 
  if (batteryLevelmV > VDD_BAT){
    batteryLevel = LORAWAN_MAX_BAT;
  }else if (batteryLevelmV < VDD_MIN){
    batteryLevel = 0;
  }else{
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  return batteryLevel;
}


#ifdef USE_B_L072Z_LRWAN1
void OnTimerLedEvent(void *context){
  LED_Off(LED_RED1) ;
}
#endif
