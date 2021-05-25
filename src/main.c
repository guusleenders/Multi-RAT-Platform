
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

#define DEVICE_ID 0

#define USER_BUTTON_ALT_PIN                         GPIO_PIN_0
#define USER_BUTTON_ALT_GPIO_PORT                   GPIOA
//#define STDBY_ON
#define DEBUG	
#define SEND_DELAY																	120*1000


// ---------------------------- GENERAL FUNCTIONS ---------------------------------
static void sendTest(void);
static void onTimerEvent(void *context);


// -------------------------- GENERAL VARIABLES ---------------------------------
static TimerEvent_t TxTimer;
bool sendTestHappened = false; 


RNG_HandleTypeDef hrng;
WWDG_HandleTypeDef   WwdgHandle;
																		
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
		bootID = random % 999;
  }
	
	HAL_RNG_DeInit(&hrng);
	
	energyStruct.general_bootID = initEnergyStruct.bootID;
	energyStruct.general_deviceID = DEVICE_ID;
	
	#ifdef DEBUG
	PRINTF_LN("- Boot ID: %d", energyStruct.general_bootID);
	#endif
	
	#ifdef DEBUG
	PRINTF_LN("Initializing...");
	#endif
	
	initEnergyMeasurement();
	
	sendTest();
	
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
	
	clearEnergyStruct();
	
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
	
	while(!isDoneLoRaWAN()){
		SCH_Run( ); 
	}
	
	HAL_Delay(500);
	
	sendEnergyStruct();
	
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq ); // Enable user to press button after transmittion
	TimerStart(&TxTimer); // Schedule next testing cycle

}

static void onTimerEvent(void *context){
	send_data_request(); 
	
}
