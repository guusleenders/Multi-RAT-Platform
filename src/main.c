
/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: main.c
 *      Created: 2020-11-27
 *       Author: Guus Leenders
 *      Version: 0.2
 *
 *  Description: text
 *      some more text
 *
 */
 
/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"

#include "scheduler.h"
#include "st_sigfox_api.h"
#include "radio.h"
#include "sgfx_credentials.h"
#include "hw_eeprom.h"
#include "bsp.h"

#include "lora.h"
#include "LoRaMac.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom2.h"
#include "version.h"

#include "ltc2942.h"
#include "sgfx_sx1276_driver.h"

#include "bg96.h"


// -------------------------- GENERAL DEFINITIONS ------------------------------
#define NBIOT
#define SIGFOX
#define LORAWAN

#define USER_BUTTON_ALT_PIN                         GPIO_PIN_0
#define USER_BUTTON_ALT_GPIO_PORT                   GPIOA
//#define STDBY_ON
#define DEBUG	
#define SEND_DELAY																	120*1000

typedef enum {
		INIT = (uint8_t)0, 
		REGISTRATION = (uint8_t)1,
		SEND = (uint8_t)2
} MESSAGE_TYPE;

// --------------------------- SIGFOX DEFINITIONS ------------------------------
#define PAC_LEN 8
#define ID_LEN 4
/* Default Configuration zone*/
/* Can be chaged according to application zone*/
#define APPLI_RC   ST_RC1

sfx_u8 error = 0;
uint8_t err_id;

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



// ---------------------------- GENERAL FUNCTIONS ---------------------------------
static void sendTest(void);
static void onTimerEvent(void *context);
static void initEnergyMeasurement(void);

// --------------------------- SIGFOX FUNCTIONS --------------------------------
static void initSigfox( void );													 // Init Sigfox modem
static void registerSigfox( void );										   // Register Sigfox to network (empty function)
static void sendSigfox( void );													 // Send Sigfox data

static sfx_error_t st_sigfox_open( st_sfx_rc_t SgfxRc ); // Open the sigfox library; @param The Region configuration

#ifndef STDBY_ON 
static void send_data_request( void  );					 // To post interrupt to backgroud; managed by scheduler
static void send_data_request_from_irq( void * context );
static void user_button_init( void );									   // Initialize the user btton to request sending data
																												 // when STDBY_ON the reset button is used instead of the push button 
#endif

// ---------------------------- LORA FUNCTIONS ---------------------------------
static void initLoRaWAN(void);													// Init lora modem
static void registerLoRaWAN(void);											// Register to LoRaWAN network (join mechanism)
static void sendLoRaWAN(void); 													// Send LoRaWAN data

static void LORA_RxData(lora_AppData_t *AppData);				// Callback when LoRa endNode has received a frame
static void LORA_HasJoined(void);												// Callback when LoRa endNode has just joined
static void LORA_ConfirmClass(DeviceClass_t Class);			// Callback when LoRa endNode has just switch the class
static void LORA_TxNeeded(void);												// Callback when server needs endNode to send a frame
static uint8_t LORA_GetBatteryLevel(void);							// Callback to get the battery level in % of full charge (254 full charge, 0 no charge)
static void LORA_Done(void);	

static void LoraMacProcessNotify(void);									// TX timer callback function

// ---------------------------- NBIOT FUNCTIONS ---------------------------------
static void initNBIoT(void);														// Init NB-IoT modem
static void registerNBIoT(void);												// Register to NB-IoT network
static void sendNBIoT(void);														// Send NB-IoT data

// -------------------------- GENERAL VARIABLES ---------------------------------
static TimerEvent_t TxTimer;
bool sendTestHappened = false; 
static uint16_t energy = 0;

struct InitEnergy_t {
	 uint8_t deviceID; 
	 uint8_t bootID;
	 uint16_t packetNumber;
	 MESSAGE_TYPE packetType; 
   uint16_t  nbiotEnergy;
   uint16_t  sigfoxEnergy;
	 uint16_t lorawanEnergy;
} initEnergyStruct;  

struct Energy_t {
	uint8_t deviceID; 
	uint8_t bootID;
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
} energyStruct;  



// ---------------------------- LORA VARIABLES ---------------------------------
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
																							
static uint8_t AppLedStateOn = RESET; 		// Specifies the state of the application LED

//static TimerEvent_t TxTimer;
#ifdef USE_B_L072Z_LRWAN1
static TimerEvent_t TxLedTimer; 					// Timer to handle the application Tx Led to toggle
static void OnTimerLedEvent(void *context);
#endif

// Initialises the Lora Parameters
static  LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE,
                                     LORAWAN_DEFAULT_DATA_RATE,
                                     LORAWAN_PUBLIC_NETWORK
                                    };
																		
// -------------------------------- MAIN ---------------------------------------
int main( void ){


  HAL_Init(); 					// STM32 HAL library initialization
  SystemClock_Config(); // Configure the system clock  
  DBG_Init();   				// Configure the debug mode
  HW_Init();						// Configure the hardware
	vcom2_Init( NULL );
  HW_EEPROM_Init(); 		// Initialise Eeprom factory Setting at device Birth
	
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED2);
	
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	
	PRINTF_LN("Started...");
	initEnergyStruct.bootID = randr(0, 255);
	energyStruct.bootID = initEnergyStruct.bootID;
	initEnergyStruct.deviceID = 0;
	energyStruct.deviceID = 0;
	initEnergyStruct.packetNumber = 0;
	energyStruct.packetNumber = 0;
	
	PRINTF_LN("- Boot ID: %d", energyStruct.bootID);
	
	
	PRINTF_LN("Initializing...");
	initEnergyMeasurement();
	
	#ifdef NBIOT
	PRINTF_LN("1. NB-IoT");
	initNBIoT();
	#endif
	
	#ifdef SIGFOX
	PRINTF_LN("2. Sigfox");
	initSigfox();
	#endif
	
	#ifdef LORAWAN
	PRINTF_LN("3. LoRaWAN");
	initLoRaWAN();
	#endif
	
	PRINTF_LN("Registering...");
	#ifdef NBIOT
	PRINTF_LN("1. NB-IoT");
	registerNBIoT();
	#endif
	
	#ifdef SIGFOX
	PRINTF_LN("2. Sigfox");
	registerSigfox();
	#endif
	
	#ifdef LORAWAN
	PRINTF_LN("3. LoRaWAN");
	registerLoRaWAN();
	#endif
	
	// Set low power mode: stop mode (timers on)
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
	
	SCH_RegTask(SEND_TASK, sendTest);		  // Record send data task
	
	// Init button 
  user_button_init();										// Initialise user button
  
	// Set timers for every 30 seconds (defined by SEND_DELAY in ms)
	TimerInit(&TxTimer, onTimerEvent);
	TimerSetValue(&TxTimer,  SEND_DELAY);
	TimerStart(&TxTimer);
	
	PRINTF_LN("");
	PRINTF_LN("");
	
  /* main loop*/
  while( 1 ){
    SCH_Run( ); 
  }
}

void SCH_Idle( void ){
	if (AppProcessRequest == LORA_SET){
    AppProcessRequest = LORA_RESET; 				// Reset notification flag
		sendLoRaWAN();													// Send package
	}
	if (LoraMacProcessRequest == LORA_SET){
		LoraMacProcessRequest = LORA_RESET;		  // Reset notification flag
		LoRaMacProcess();
	}
  BACKUP_PRIMASK();
  DISABLE_IRQ( );
	LPM_EnterLowPower();
  RESTORE_PRIMASK( );
}

static void sendTest(void){
	PRINTF_LN("Starting testing sequence...");
	
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
	
	#ifdef NBIOT
	PRINTF("1. NB-IoT \n");
	sendNBIoT();
	#endif
	
	
	HAL_Delay(500);
	
	#ifdef SIGFOX
	PRINTF("2. SIGFOX \n");
	initSigfox();
	sendSigfox();
	#endif
	
	HAL_Delay(500);
	
	#ifdef LORAWAN
	PRINTF("3. LORAWAN \n");

	LoRaMacInitializationReset();
	sendLoRaWAN();
	#endif
	
	HAL_Delay(500);
	
	energyStruct.packetNumber++;
	
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq ); // Enable user to press button after transmittion
}

static void onTimerEvent(void *context){
	TimerStart(&TxTimer);
	send_data_request(); 
}


// -------------------------- ENERGY MEASUREMENT FUNCTIONS -------------------------------
static void initEnergyMeasurement(void){
	#if defined(LORAWAN) || defined(SIGFOX)
	LTC2942_Init(LTC2942_LRWAN);
	LTC2942_SetPrescaler(LTC2942_LRWAN, PRESCALAR_M_1);
	LTC2942_SetAlertConfig(LTC2942_LRWAN, ALERT_DISABLED);
	LTC2942_SetAccumulatedCharge(LTC2942_LRWAN, 0);
	LTC2942_SetShutdown(LTC2942_LRWAN, 0);
	
	uint16_t testvoltage = LTC2942_GetVoltage(LTC2942_LRWAN)*1000;
	PRINTF_LN("Voltage LRWAN: %d mV", testvoltage);
	
	LTC2942_SetShutdown(LTC2942_LRWAN, 1);
	#endif
	
	#ifdef NBIOT
	LTC2942_Init(LTC2942_NBIOT);
	LTC2942_SetPrescaler(LTC2942_NBIOT, PRESCALAR_M_1);
	LTC2942_SetAlertConfig(LTC2942_NBIOT, ALERT_DISABLED);
	LTC2942_SetAccumulatedCharge(LTC2942_NBIOT, 0);
	LTC2942_SetShutdown(LTC2942_NBIOT, 0);
	
	testvoltage = LTC2942_GetVoltage(LTC2942_NBIOT)*1000;
	PRINTF_LN("Voltage NBIOT: %d mV", testvoltage);
	
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
	#endif
	
}

static void startEnergyMeasurement(LTC2942_SENSOR sensor){
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
}
     

// -------------------------------- SIGFOX FUNCTIONS -------------------------------------
static void initSigfox( void ){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	sfx_error_t error;
  uint8_t dev_id[ID_LEN];
  uint8_t dev_pac[PAC_LEN];
  st_sfx_rc_t SgfxRc=APPLI_RC;
	
	error=st_sigfox_open( SgfxRc);															// Open Sifox Lib
  HW_EEPROM_WRITE( E2pData.SgfxKey, CREDENTIALS_KEY_PRIVATE); // Use private key
  if ( error == SFX_ERR_NONE ){
    PRINTF(" OK\n\r");
  }
  else{
    PRINTF(" error %d\n\r", error);
  }
	

  SIGFOX_API_get_device_id(dev_id);
  SIGFOX_API_get_initial_pac(dev_pac);
	SGFX_SX1276_setPower(14); // power between 10 and 20dBm
		
	PRINTF_LN("- Initialised");
	
	#ifdef DEBUG
	PRINTF("%d dBm\r\n",SGFX_SX1276_getPower( ) );
  PRINTF("devId=") ; for(int i =0; i<ID_LEN; i++) {PRINTF("%02X",dev_id[ID_LEN-1-i]);} PRINTF("\n\r");
  PRINTF("devPac="); for(int i =0; i<PAC_LEN; i++) {PRINTF("%02X",dev_pac[i]);} PRINTF("\n\r");
	#endif
	
	initEnergyStruct.packetType = INIT;
	initEnergyStruct.sigfoxEnergy = stopEnergyMeasurement(LTC2942_LRWAN);
}

static void registerSigfox( void ){
	
}

static void sendSigfox( void ){
	
	//startEnergyMeasurement(LTC2942_LRWAN);
	
  uint8_t ul_msg[12] = {0x00, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11}; 
  uint8_t dl_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t  ul_size =0;
  uint32_t nbTxRepeatFlag=1;
  int i=0;
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint32_t batteryLevel=0;// HW_GetBatteryLevel( );                       // in mV


  ul_msg[ul_size++] = (uint8_t) ((batteryLevel*100)/3300);
  ul_msg[ul_size++] = ( pressure >> 8 ) & 0xFF;
  ul_msg[ul_size++] = pressure & 0xFF;
  ul_msg[ul_size++] = ( temperature >> 8 ) & 0xFF;
  ul_msg[ul_size++] = temperature & 0xFF;
  ul_msg[ul_size++] = ( humidity >> 8 ) & 0xFF;
  ul_msg[ul_size++] = humidity & 0xFF;

	
	PRINTF("- Data to be sent:");
  for (i=0; i<ul_size; i++){
    PRINTF("%02X ", ul_msg[i]) ;
  }
	PRINTF_LN("");
  //BSP_LED_On( LED_BLUE );
	#ifdef DEBUG
  PRINTF_LN("- Start sending Sigfox data");
	#endif 
	// -- Send frame on Sigfox network
  SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, SFX_FALSE);
	
  #ifdef DEBUG
  PRINTF_LN("- Done sending Sigfox data");
	#endif 
  //BSP_LED_Off( LED_BLUE );
	
	/*uint16_t uwh = stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.sigfoxPacketType = SEND;
	energyStruct.sigfoxEnergy = uwh;
	PRINTF("\r\n||Sigfox energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ)||\r\n", energy, uwh, uwh*3.6f);*/
}

#ifndef STDBY_ON 
static void send_data_request_from_irq( void * context ){
  /* send task to background*/
  send_data_request();
}
static void send_data_request( void ){
  /* send task to background*/
  SCH_SetTask( SEND_TASK );
}
#endif

sfx_error_t st_sigfox_open( st_sfx_rc_t SgfxRc ){
  sfx_error_t error = SFX_ERR_NONE;

  // Record RCZ
  switch(SgfxRc.id){
    case RC1_ID: {      
      error = SIGFOX_API_open(&SgfxRc.param);
      break;
    }
    case RC2_ID:{
      sfx_u32 config_words[3] = {RC2_SET_STD_CONFIG_SM_WORD_0, RC2_SET_STD_CONFIG_SM_WORD_1, RC2_SET_STD_CONFIG_SM_WORD_2 };
      error = SIGFOX_API_open(&SgfxRc.param );
      if ( error == SFX_ERR_NONE ){
        error = SIGFOX_API_set_std_config(  config_words, RC2_SET_STD_TIMER_ENABLE);
      }
      break;
    }
    case RC3C_ID:{
      sfx_u32 config_words[3] = {0x00000003,0x00001388,0x00000000};
      error = SIGFOX_API_open(&SgfxRc.param );
      if ( error == SFX_ERR_NONE ){
        error = SIGFOX_API_set_std_config( config_words, NA);
      }
      break;
    }
    case RC4_ID:{
      sfx_u32 config_words[3] = {RC4_SET_STD_CONFIG_SM_WORD_0, RC4_SET_STD_CONFIG_SM_WORD_1, RC4_SET_STD_CONFIG_SM_WORD_2 };
      error = SIGFOX_API_open(&SgfxRc.param );
      if ( error == SFX_ERR_NONE ){
        error = SIGFOX_API_set_std_config( config_words, RC4_SET_STD_TIMER_ENABLE);
      }
      break;
    }
    default:{
      error = SFX_ERR_API_OPEN;
      break;
    }
  }
  return error;
}

#ifndef STDBY_ON 
/* when STDBY_ON the reset button is used instead of the push button */
static void user_button_init( void ){

  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_PULLUP;
  initStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
  
  /* send everytime button is pushed */
  HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq );
}
#endif

// -------------------------------- LORA FUNCTIONS -------------------------------------

static void initLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
	PRINTF_LN("- Initialised");
	
	initEnergyStruct.packetType = INIT;
	initEnergyStruct.lorawanEnergy = stopEnergyMeasurement(LTC2942_LRWAN);
}

static void registerLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	LORA_Join();
	PRINTF_LN("- Joined");
	
	// Stop energy measurement in hasjoined function
}

static void sendLoRaWAN(void){
	startEnergyMeasurement(LTC2942_LRWAN);
	


  if (LORA_JoinStatus() != LORA_SET) {
    /*Not joined, try again later*/
		#ifdef DEBUG
		PRINTF_LN("- Not joined, try again later");
		#endif
    LORA_Join();
    return;
  }

	#ifndef CAYENNE_LPP
  //int32_t latitude, longitude = 0;
  //uint16_t altitudeGps = 0;
	#endif

	#ifdef USE_B_L072Z_LRWAN1
  TimerInit(&TxLedTimer, OnTimerLedEvent);

  TimerSetValue(&TxLedTimer, 200);

  LED_On(LED_RED1) ;

  TimerStart(&TxLedTimer);
	#endif

  uint32_t i = 0;

  //batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

  AppData.Buff[i++] = energyStruct.deviceID;
  AppData.Buff[i++] = energyStruct.bootID;
  AppData.Buff[i++] = (energyStruct.packetNumber >> 8) & 0xFF;;
	AppData.Buff[i++] = energyStruct.packetNumber  & 0xFF;
	
	for(uint8_t p = 0; p < LORAWAN_PAYLOAD_SIZE-4; p++){
		AppData.Buff[i++] = 0x00;
	}
  AppData.BuffSize = i;
	#ifdef DEBUG
	PRINTF_LN("- Sending packet");
	#endif
  LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	PRINTF_LN("- LoRaWAN send command done");
//	if (LoraMacProcessRequest == LORA_SET)
//    {
//      /*reset notification flag*/
//      LoraMacProcessRequest = LORA_RESET;
//      LoRaMacProcess();
//    }

  /* USER CODE END 3 */
	
	//LPM_SetOffMode(LPM_APPLI_Id, LPM_Enable);
}

void LoraMacProcessNotify(void){
  LoraMacProcessRequest = LORA_SET;
}

static void LORA_HasJoined(void){
	#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF_LN("- Joined done");
	#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
	isConnectedLoRaWAN = true;
	//BSP_LED_On(LED_GREEN);
	
	uint16_t uwh = stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.lorawanPacketType = REGISTRATION;
	energyStruct.lorawanEnergy = uwh;
	PRINTF("\r\n||LoRa join energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ)||\r\n", energy, uwh, uwh*3.6f);

}

static void LORA_RxData(lora_AppData_t *AppData){
  /* USER CODE BEGIN 4 */
	#ifdef DEBUG
  PRINTF_LN("- LoRaWAN packet received on port %d\n\r", AppData->Port);
	#endif
	
  switch (AppData->Port){
    case 3:
      /*this port switches the class*/
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

static void LORA_ConfirmClass(DeviceClass_t Class){
	#ifdef DEBUG
  PRINTF("- Switching to class %c done\n\r", "ABC"[Class]);
	#endif

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded(void){
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_Done(void){
	uint16_t uwh = 	stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.lorawanEnergy = uwh;
	energyStruct.lorawanPacketType = SEND;
	PRINTF("\r\n||LoRa energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
	LTC2942_SetShutdown(LTC2942_LRWAN, 1);
}


/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t LORA_GetBatteryLevel(void){
  uint16_t batteryLevelmV;
  uint8_t batteryLevel = 0;

  batteryLevelmV = HW_GetBatteryLevel();


  /* Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) */
  if (batteryLevelmV > VDD_BAT){
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN){
    batteryLevel = 0;
  }
  else{
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  return batteryLevel;
}


#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent(void *context){
  LED_Off(LED_RED1) ;
}
#endif

// -------------------------------- NBIOT FUNCTIONS -------------------------------------

static void initNBIoT(void){
	startEnergyMeasurement(LTC2942_NBIOT);

	BG96_Init();
	BG96_PowerOn();
	
	char buffer[30];
	memset(buffer, '\0', 30);

	BG96_SetErrorFormat(BG96_ERROR_VERBOSE);
	BG96_SetNetworkReporting(BG96_NETWORK_REPORTING_DISABLE);
	BG96_CheckSIMPIN(buffer);
	BG96_SetMode(BG96_NBIOT_MODE);
	BG96_EnablePSMIndication();
	BG96_SetPowerSavingMode(1, "", "", "\"00000100\"", "\"00001111\""); // set tau timer to ..., active timer to 30s (seems to work best in network)
	BG96_SetPowerSavingModeImmediately(); // Not available in current firmware
	PRINTF_LN("- Initialised");

	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	initEnergyStruct.packetType = INIT;
	initEnergyStruct.nbiotEnergy = uwh;
	PRINTF("\r\n||NB-IoT init energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
}

static void registerNBIoT(void){
	startEnergyMeasurement(LTC2942_NBIOT);
	
	BG96_SelectNetwork(20601, BG96_NETWORK_NBIOT);
	BG96_ConnectToOperator(60000);
	PRINTF_LN("- Connected to operator");
	
	uint8_t celevel; 
	BG96_GetCELevel(&celevel);
	PRINTF_LN("- CE Level: %d", celevel);
	
	BG96_DisableNetworkStatus();
	
	/*BG96_WaitForPowerDown(240000);
	PRINTF_LN("- Entered PSM");
	
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Status: powered down confirmed");
	}*/

	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	energyStruct.nbiotPacketType = REGISTRATION;
	energyStruct.nbiotEnergy = uwh;
	PRINTF("\r\n||NB-IoT registration energy used: %d uAh/10 (%d uWh/10, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
	
	sendNBIoT();
	/*
	BG96_ActivateContext();
	BG96_UDP_Start("62.235.63.122",8891);
	BG96_UDP_SendData("Hello world");
	BG96_UDP_Stop();
	BG96_DeactivateContext();
	
	BG96_WaitForPowerDown(240000);
	PRINTF_LN("- Entered PSM");
	
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Status: powered down confirmed");
	}
	
	energy = LTC2942_GetmAh(LTC2942_NBIOT)*10000 - energy;
	voltage = LTC2942_GetVoltage(LTC2942_NBIOT);
	uwh = (uint16_t) (energy * voltage);
	mj = uwh * 3.6;
	PRINTF("\r\n||NB-IoT first packet energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, mj);
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
	*/
}

static void sendNBIoT(void){
	startEnergyMeasurement(LTC2942_NBIOT);
	
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Status: powered down");
		PRINTF_LN("- Waking from psm");
		BG96_WakeFromPSM();
	}else{
		PRINTF_LN("- Status: powered on");
	}
	
	
	BG96_UDP_Start("62.235.63.122",8891);
	char buffer[100]; 
	//			  			 1  2  3  4  5  6  7  8  9  10 11 12 13
	sprintf(buffer, "%d,%d,%d,%d,%s,%d,%d,%s,%d,%d,%s,%d",    	energyStruct.deviceID, \
																															energyStruct.bootID, \
																															energyStruct.packetNumber, \
																															energyStruct.nbiotEnergy, \
																															energyStruct.nbiotConditions, \
																															energyStruct.nbiotPacketType, \
																															energyStruct.sigfoxEnergy, \
																															energyStruct.sigfoxConditions, \
																															energyStruct.sigfoxPacketType, \
																															energyStruct.lorawanEnergy, \
																															energyStruct.lorawanConditions, \
																															energyStruct.lorawanPacketType);		
	BG96_UDP_SendData(buffer);
	BG96_UDP_Stop();
	BG96_DeactivateContext();
	
	PRINTF_LN("- Done sending NB-IoT");
	PRINTF_LN("- Waiting for PSM");
	
	BG96_WaitForPowerDown(240000);
	PRINTF_LN("- Entered PSM");
	
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Status: powered down confirmed");
	}
	
	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	energyStruct.nbiotPacketType = SEND;
	energyStruct.nbiotEnergy = uwh;
	uint32_t mj = uwh * 3.6f;
	PRINTF("\r\n||NB-IoT packet energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, mj);
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
}
