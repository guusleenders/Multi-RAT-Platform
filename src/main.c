
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

#include "sigfox.h"
#include "lorawan.h"
#include "nbiot.h"
#include "energy.h"

#include "stm32l0xx_hal_wwdg.h"
#include "stm32l0xx_hal_rng.h"
#include "stm32l0xx_hal_gpio.h"

#include "hw_eeprom.h"

#include "mlm32l07x01.h"

#include "timeServer.h"
#include "vcom2.h"

/*
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
#include "stm32l0xx_hal.h"

#include "stm32l0xx_hal_wwdg.h"
#include "stm32l0xx_hal_rng.h"
#include "stm32l0xx_hal_gpio.h"

#include "mlm32l07x01.h"

#include "bg96.h"

#include "bq27441.h"

#include "sigfox.h"
#include "lorawan.h"
#include "nbiot.h"
#include "energy.h"

*/
// -------------------------- GENERAL DEFINITIONS ------------------------------


#define NBIOT
#define SIGFOX
#define LORAWAN

#define SHOW_SIGFOX_ID
#ifdef DEVICE_3
#define DEVICE_ID 1
#else
#define DEVICE_ID 0
#endif

#define USER_BUTTON_ALT_PIN                         GPIO_PIN_0
#define USER_BUTTON_ALT_GPIO_PORT                   GPIOA
//#define STDBY_ON
#define DEBUG	
#define SEND_DELAY																	300*1000//300*1000


// ---------------------------- GENERAL FUNCTIONS ---------------------------------
static void sendTest(void);
static void onTimerEvent(void *context);
static void onTimerResult(void *context);
void send_data_request_result( void );
static void sendResult(void);

// -------------------------- GENERAL VARIABLES ---------------------------------
static TimerEvent_t TxTimer;
static TimerEvent_t ResultTimer;

RNG_HandleTypeDef hrng;
WWDG_HandleTypeDef   WwdgHandle;

#define EEPROM_BASE_ADDRESS     ((uint32_t)(FLASH_BASE + 0x80000U))

HAL_StatusTypeDef writeEEPROMByte(uint32_t address, uint8_t data){
	HAL_StatusTypeDef status;
	uint32_t eepromAddress = address + EEPROM_BASE_ADDRESS;
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram(); 
	status = HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_BYTE, eepromAddress, data);
	HAL_FLASHEx_DATAEEPROM_Lock();
 
	return status;
}
 
uint8_t readEEPROMByte(uint32_t address){
	uint8_t data = 0;
	uint32_t eepromAddress = address + EEPROM_BASE_ADDRESS;
	data = *(__IO uint8_t *)eepromAddress; // Read byte at address
	return data;
}


// -------------------------------- MAIN ---------------------------------------
int main( void ){


  HAL_Init(); 					// STM32 HAL library initialization
  SystemClock_Config(); // Configure the system clock  
  DBG_Init();   				// Configure the debug mode
	HW_Init();						// Configure the hardware
	HW_EEPROM_Init(); 		// Initialise Eeprom factory Setting at device Birth
	#ifdef DEBUG
	vcom2_Init( NULL );
	#endif
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
  GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); 
	
  //BSP_LED_On(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED2);
	
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Disable);

	#ifdef DEBUG
	PRINTF_LN("Started...");
	#endif
	
	
	uint32_t random = 0; 
	uint16_t bootID = 0;
	
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET){ 
    PRINTF_LN("- Started from WDT");
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
	
	PRINTF_LN("- Setting up WDT");
	WwdgHandle.Instance = WWDG;

  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Window    = 80;
  WwdgHandle.Init.Counter   = 127;
	
	__HAL_RCC_RNG_CLK_ENABLE();
	hrng.Instance = RNG;
  HAL_RNG_Init(&hrng);
	
	
	
	if(HAL_RNG_GenerateRandomNumber(&hrng, &random) == HAL_OK){
		srand((uint16_t)random);
		bootID = random % 255;
  }
	
	HAL_RNG_DeInit(&hrng);
	
	energyStruct.general_bootID = bootID;
	energyStruct.general_deviceID = DEVICE_ID;
	
	#ifdef DEBUG
	PRINTF_LN("- Boot ID: %d", energyStruct.general_bootID);
	#endif
	
	#ifdef SHOW_SIGFOX_ID
	uint8_t dev_id[ID_LEN];
  uint8_t dev_pac[PAC_LEN];
	SIGFOX_API_get_device_id(dev_id);
  SIGFOX_API_get_initial_pac(dev_pac);
  PRINTF("devId=") ; for(int i =0; i<ID_LEN; i++) {PRINTF("%02X",dev_id[ID_LEN-1-i]);} PRINTF("\n\r");
  PRINTF("devPac="); for(int i =0; i<PAC_LEN; i++) {PRINTF("%02X",dev_pac[i]);} PRINTF("\n\r");
	#endif
	
	#ifdef DEBUG
	PRINTF_LN("Initializing...");
	#endif
	
	initEnergyMeasurement();
	
	
	// Set low power mode: stop mode (timers on)
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
	
	
	// Init button 
  user_button_init();										// Initialise user button
  
	// Set timers for every 30 seconds (defined by SEND_DELAY in ms)
	TimerInit(&TxTimer, onTimerEvent);
	TimerInit(&ResultTimer, onTimerResult);
	TimerSetValue(&TxTimer,  SEND_DELAY);
	TimerSetValue(&ResultTimer,  SEND_DELAY/4);
	
	clearEnergyStruct(true);
	
	#ifdef DEBUG
	PRINTF_LN("EEPROM VAL: %d", E2pData.FrameCounter);
	#endif
	
	sendTest();
	 
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

	//PRINTF_LN("Starting testing sequence...");
	
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
	
	clearEnergyStruct(false);
	
	#ifdef NBIOT
	PRINTF("1. NB-IoT \n");
	sendNBIoT();
	#endif
	
	HAL_Delay(500);
	
	#ifdef SIGFOX
	PRINTF("2. SIGFOX \n");
	sendSigfox();
	#endif
	
	HAL_Delay(500);
	
	#ifdef LORAWAN
	PRINTF("3. LORAWAN \n");
	LoRaMacInitializationReset();
	sendLoRaWAN();
	#endif
	
	#ifdef DEBUG
	PRINTF_LN("EEPROM TEST");
	PRINTF_LN("EEPROM VAL: %d", E2pData.FrameCounter);
	#endif
	
	//HAL_Delay(1000);
	//sendEnergyStruct();
	
	
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq ); // Enable user to press button after transmittion
	
	SCH_RegTask(RESULT_TASK, sendResult);		  // Record send data task
	TimerInit(&ResultTimer, onTimerResult);
	TimerSetValue(&ResultTimer,  SEND_DELAY/4);
	TimerStart(&ResultTimer); // Schedule next testing cycle

}

static void sendResult(void){
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
	sendEnergyStruct();
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq ); // Enable user to press button after transmittion
	
	SCH_RegTask(SEND_TASK, sendTest);		  // Record send data task
	TimerInit(&TxTimer, onTimerEvent);
	TimerSetValue(&TxTimer,  SEND_DELAY-SEND_DELAY/4);
	TimerStart(&TxTimer); // Schedule next testing cycle
	
	PRINTF_LN("- Waiting for next schedule");
}

static void onTimerEvent(void *context){
	send_data_request(); 
	
}

static void onTimerResult(void *context){
	send_data_request_result(); 
	
}

void send_data_request_result( void ){
  /* send task to background*/
  SCH_SetTask( RESULT_TASK );
}
