/******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   this is the main!
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other 
 *    contributors to this software may be used to endorse or promote products 
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this 
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under 
 *    this license is void and will automatically terminate your rights under 
 *    this license. 
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
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
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"

#include "ltc2941.h"
#include "sgfx_sx1276_driver.h"

// -------------------------- GENERAL DEFINITIONS ------------------------------
#define USER_BUTTON_ALT_PIN                         GPIO_PIN_0
#define USER_BUTTON_ALT_GPIO_PORT                   GPIOA
//#define STDBY_ON
#define DEBUG	
#define SEND_DELAY																	30000

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

#define APP_TX_DUTYCYCLE 									1000 					  // Defines the application data transmission duty cycle. 5s, value in [ms; 10000].
#define LORAWAN_ADR_STATE 								LORAWAN_ADR_ON 	// LoRaWAN Adaptive Data Rate; Please note that when ADR is enabled the end-device should be static
#define LORAWAN_DEFAULT_DATA_RATE 				DR_0 						// LoRaWAN Default data Rate Data Rate; Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
#define LORAWAN_APP_PORT 									2 							// LoRaWAN application port; do not use 224. It is reserved for certification
#define LORAWAN_DEFAULT_CLASS 						CLASS_A 				// LoRaWAN default endNode class port
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE LORAWAN_UNCONFIRMED_MSG // LoRaWAN default confirm state
#define LORAWAN_APP_DATA_BUFF_SIZE 				64 							// User application data buffer size



// ---------------------------- GENERAL FUNCTIONS ---------------------------------
static void sendTest(void);
static void onTimerEvent(void *context);

// --------------------------- SIGFOX FUNCTIONS --------------------------------
static sfx_error_t st_sigfox_open( st_sfx_rc_t SgfxRc ); // Open the sigfox library; @param The Region configuration
static void sendSigfox( void );													 // Send data to back end sigfox server

#ifndef STDBY_ON 
static void send_data_request( void  );					 // To post interrupt to backgroud; managed by scheduler
static void send_data_request_from_irq( void * context );
static void user_button_init( void );									   // Initialize the user btton to request sending data
																												 // when STDBY_ON the reset button is used instead of the push button 
#endif

// ---------------------------- LORA FUNCTIONS ---------------------------------
static void LORA_RxData(lora_AppData_t *AppData);				// Callback when LoRa endNode has received a frame
static void LORA_HasJoined(void);												// Callback when LoRa endNode has just joined
static void LORA_ConfirmClass(DeviceClass_t Class);			// Callback when LoRa endNode has just switch the class
static void LORA_TxNeeded(void);												// Callback when server needs endNode to send a frame
static uint8_t LORA_GetBatteryLevel(void);							// Callback to get the battery level in % of full charge (254 full charge, 0 no charge)
static void sendLoRaWAN(void);													// LoRa endNode send request

//static void LoraStartTx(TxEventType_t EventType);			// Start the tx process
//static void OnTxTimerEvent(void *context);						// TX timer callback function
static void LoraMacProcessNotify(void);									// TX timer callback function

// -------------------------- GENERAL VARIABLES ---------------------------------
static TimerEvent_t TxTimer;
bool sendTestHappened = false; 

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
                                                LoraMacProcessNotify
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
  sfx_error_t error;
  uint8_t dev_id[ID_LEN];
  uint8_t dev_pac[PAC_LEN];
  st_sfx_rc_t SgfxRc=APPLI_RC;

  HAL_Init(); 					// STM32 HAL library initialization
  SystemClock_Config(); // Configure the system clock  
  DBG_Init();   				// Configure the debug mode
  HW_Init();						// Configure the hardware
  HW_EEPROM_Init(); 		// Initialise Eeprom factory Setting at device Birth

  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED2);

		
	LTC2941_Init();
	uint8_t test = LTC2941_GetStatus();
	PRINTF("LTC2641: %d||", test);
	uint8_t test1 = LTC2941_GetControl();
	PRINTF("%d||", test1);
	
	LTC2941_SetPrescaler(PRESCALAR_M_1);
	LTC2941_SetAlertConfig(ALERT_DISABLED);
	LTC2941_SetBatteryAlert(VBAT_ALERT_OFF);
	LTC2941_SetAccumulatedCharge(0);
	
	test1 = LTC2941_GetControl();
	PRINTF("%d||", test1);
  #ifdef DEBUG
	PRINTF("wakeup1");
	#endif
	
  // -- Init Sigfox
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
  
	#ifdef DEBUG
	PRINTF("%d dBm\r\n",SGFX_SX1276_getPower( ) );
  PRINTF("devId=") ; for(int i =0; i<ID_LEN; i++) {PRINTF("%02X",dev_id[ID_LEN-1-i]);} PRINTF("\n\r");
  PRINTF("devPac="); for(int i =0; i<PAC_LEN; i++) {PRINTF("%02X",dev_pac[i]);} PRINTF("\n\r");
	#endif
	
	// Init LoRaWAN
	LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
	#ifdef DEBUG
	PRINTF("lora init done");
	#endif
	
	//LORA_Join();
	#ifdef DEBUG
	PRINTF("lora join done");
	#endif
	
	// Set low power mode: stop mode (timers on)
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
	
	SCH_RegTask( SEND_TASK, sendTest );		  // Record send data task
	
	// Init button 
  user_button_init( );										// Initialise user button
  
	// Set timers for every 30 seconds (defined by SEND_DELAY in ms)
	TimerInit(&TxTimer, onTimerEvent);
	TimerSetValue(&TxTimer,  SEND_DELAY);
	TimerStart(&TxTimer);
	
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
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
	uint16_t test3 = LTC2941_GetAccumulatedCharge();
	PRINTF("||%d||",test3);
	#ifdef DEBUG
	PRINTF("1. SIGFOX \n");
	#endif
	sendSigfox();
	#ifdef DEBUG
	PRINTF("2. LORAWAN \n");
	#endif
	LoRaMacInitializationReset();
	sendLoRaWAN();
	uint16_t test4 = LTC2941_GetAccumulatedCharge();
	PRINTF("||%d||",test4);
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq ); // Enable user to press button after transmittion
	
}

static void onTimerEvent(void *context){
	TimerStart(&TxTimer);
	send_data_request(); 
}

// -------------------------------- SIGFOX FUNCTIONS -------------------------------------
static void sendSigfox( void ){
	PRINTF(" | in sendSigfox | ");
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

	#ifdef DEBUG
  PRINTF("senddata....");
	#endif 
	
  for (i=0; i<ul_size; i++){
    PRINTF("%02X ", ul_msg[i]) ;
  }
  BSP_LED_On( LED_BLUE );
	


	// -- Send frame on Sigfox network
  SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, SFX_FALSE);
  
  BSP_LED_Off( LED_BLUE );
	
	#ifdef DEBUG
  PRINTF("done\n\r");
	#endif
	
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
void LoraMacProcessNotify(void){
  LoraMacProcessRequest = LORA_SET;
}

static void LORA_HasJoined(void){
	#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
	#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
	isConnectedLoRaWAN = true;
	//BSP_LED_On(LED_GREEN);
}

static void sendLoRaWAN(void){
	//LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);
  //LORA_Join();
	
  /* USER CODE BEGIN 3 */
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t batteryLevel;
  sensor_t sensor_data;

	PRINTF("STARTING SEND");
	
  if (LORA_JoinStatus() != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
	#ifdef DEBUG
  PRINTF("SEND REQUEST\n\r");
	#endif 
	
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

  BSP_sensor_Read(&sensor_data);

	#ifdef CAYENNE_LPP
  uint8_t cchannel = 0;
  temperature = (int16_t)(sensor_data.temperature * 10);         /* in °C * 10 */
  pressure    = (uint16_t)(sensor_data.pressure * 100 / 10);      /* in hPa / 10 */
  humidity    = (uint16_t)(sensor_data.humidity * 2);            /* in %*2     */
  uint32_t i = 0;

  batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;

  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_BAROMETER;
  AppData.Buff[i++] = (pressure >> 8) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_TEMPERATURE;
  AppData.Buff[i++] = (temperature >> 8) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_HUMIDITY;
  AppData.Buff[i++] = humidity & 0xFF;
	#if defined( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
	#else
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT;
  AppData.Buff[i++] = batteryLevel * 100 / 254;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT;
  AppData.Buff[i++] = AppLedStateOn;
	#endif  /* REGION_XX915 */
	#else  /* not CAYENNE_LPP */

  temperature = (int16_t)(sensor_data.temperature * 100);         /* in °C * 100 */
  pressure    = (uint16_t)(sensor_data.pressure * 100 / 10);      /* in hPa / 10 */
  humidity    = (uint16_t)(sensor_data.humidity * 10);            /* in %*10     */
  //latitude = sensor_data.latitude;
  //longitude = sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

	#if defined( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = (pressure >> 8) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = (temperature >> 8) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = (humidity >> 8) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
	#else  /* not REGION_XX915 */
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = (pressure >> 8) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = (temperature >> 8) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = (humidity >> 8) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
	AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
  AppData.Buff[i++] = 0x00;
//  AppData.Buff[i++] = (latitude >> 16) & 0xFF;
//  AppData.Buff[i++] = (latitude >> 8) & 0xFF;
//  AppData.Buff[i++] = latitude & 0xFF;
//  AppData.Buff[i++] = (longitude >> 16) & 0xFF;
//  AppData.Buff[i++] = (longitude >> 8) & 0xFF;
//  AppData.Buff[i++] = longitude & 0xFF;
//  AppData.Buff[i++] = (altitudeGps >> 8) & 0xFF;
//  AppData.Buff[i++] = altitudeGps & 0xFF;
	#endif  /* REGION_XX915 */
	#endif  /* CAYENNE_LPP */
  AppData.BuffSize = i;
	#ifdef DEBUG
	PRINTF("LORASEND");
	#endif
  LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	PRINTF("LORA SEND DONE");
//	if (LoraMacProcessRequest == LORA_SET)
//    {
//      /*reset notification flag*/
//      LoraMacProcessRequest = LORA_RESET;
//      LoRaMacProcess();
//    }

  /* USER CODE END 3 */
	
	//LPM_SetOffMode(LPM_APPLI_Id, LPM_Enable);
}


static void LORA_RxData(lora_AppData_t *AppData){
  /* USER CODE BEGIN 4 */
	#ifdef DEBUG
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);
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
  /* USER CODE END 4 */
}

//static void OnTxTimerEvent(void *context){
//  /*Wait for next tx slot*/
//  TimerStart(&TxTimer);

//  AppProcessRequest = LORA_SET;
//}

//static void LoraStartTx(TxEventType_t EventType)
//{
//  if (EventType == TX_ON_TIMER)
//  {
//    /* send everytime timer elapses */
//    TimerInit(&TxTimer, OnTxTimerEvent);
//    TimerSetValue(&TxTimer,  APP_TX_DUTYCYCLE);
//    OnTxTimerEvent(NULL);
//  }
//  else
//  {
//    /* send everytime button is pushed */
//    GPIO_InitTypeDef initStruct = {0};

//    initStruct.Mode = GPIO_MODE_IT_RISING;
//    initStruct.Pull = GPIO_PULLUP;
//    initStruct.Speed = GPIO_SPEED_HIGH;

//    HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct);
//    HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send);
//  }
//}

static void LORA_ConfirmClass(DeviceClass_t Class){
	#ifdef DEBUG
  PRINTF("switch to class %c done\n\r", "ABC"[Class]);
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
