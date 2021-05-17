// -------------------------------- SIGFOX FUNCTIONS -------------------------------------

#include "sigfox.h"

static void initSigfox( void ){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	SX1276SetXO(1);
	
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
	
	SX1276SetXO(0);
	
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
	
	startEnergyMeasurement(LTC2942_LRWAN);
	
  uint8_t ul_msg[12] = {0x00, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11}; 
  uint8_t dl_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t  ul_size =0;
  uint32_t nbTxRepeatFlag=1;
  int i=0;
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint32_t batteryLevel=0; // HW_GetBatteryLevel( );                       // in mV


  ul_msg[ul_size++] = (uint8_t) ((batteryLevel*100)/3300);
  ul_msg[ul_size++] = ( pressure >> 8 ) & 0xFF;
  ul_msg[ul_size++] = pressure & 0xFF;
  ul_msg[ul_size++] = ( temperature >> 8 ) & 0xFF;
  ul_msg[ul_size++] = temperature & 0xFF;
  ul_msg[ul_size++] = ( humidity >> 8 ) & 0xFF;
  ul_msg[ul_size++] = humidity & 0xFF;

	SIGFOX_API_close(); // Make sure sigfox api is closed before opening
	
	st_sfx_rc_t SgfxRc=APPLI_RC;
	
	error=st_sigfox_open( SgfxRc);															// Open Sifox Lib
  HW_EEPROM_WRITE( E2pData.SgfxKey, CREDENTIALS_KEY_PRIVATE); // Use private key
  if ( error == SFX_ERR_NONE ){
    PRINTF(" OK\n\r");
  }
  else{
    PRINTF(" error %d\n\r", error);
  }
	
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
  sfx_error_t error = SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, SFX_FALSE);
	if(error != SFX_ERR_NONE)
			PRINTF_LN("- SIGFOX ERROR %d", error);
	SIGFOX_API_close();
	
  #ifdef DEBUG
  PRINTF_LN("- Done sending Sigfox data");
	#endif 
  //BSP_LED_Off( LED_BLUE );
	
	uint16_t uwh = stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.sigfoxPacketType = SEND;
	energyStruct.sigfoxEnergy = uwh;
	PRINTF("\r\n||Sigfox energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ)||\r\n", energy, uwh, uwh*3.6f);
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

