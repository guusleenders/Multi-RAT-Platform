// -------------------------------- SIGFOX FUNCTIONS -------------------------------------

#include "sigfox.h"

bool sigfoxInitialised = false; 

void initSigfox( void ){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	SX1276SetXO(1);
	
	sfx_error_t sfxerror;
  uint8_t dev_id[ID_LEN];
  uint8_t dev_pac[PAC_LEN];
  st_sfx_rc_t SgfxRc=APPLI_RC;
	
	sfxerror=st_sigfox_open( SgfxRc);															// Open Sifox Lib
  HW_EEPROM_WRITE( E2pData.SgfxKey, CREDENTIALS_KEY_PRIVATE); // Use private key
  if ( sfxerror == SFX_ERR_NONE ){
    PRINTF(" OK\n\r");
  }
  else{
    PRINTF(" error %d\n\r", sfxerror);
  }
	

  SIGFOX_API_get_device_id(dev_id);
  SIGFOX_API_get_initial_pac(dev_pac);
	SGFX_SX1276_setPower(SIGFOX_POWER); // power between 10 and 20dBm
	
	sprintf(energyStruct.sigfox_conditions, "%d|", SIGFOX_POWER);
	
	PRINTF_LN("- Initialised");
	
	SX1276SetXO(0);
	
	#ifdef DEBUG
	PRINTF("%d dBm\r\n",SGFX_SX1276_getPower( ) );
  PRINTF("devId=") ; for(int i =0; i<ID_LEN; i++) {PRINTF("%02X",dev_id[ID_LEN-1-i]);} PRINTF("\n\r");
  PRINTF("devPac="); for(int i =0; i<PAC_LEN; i++) {PRINTF("%02X",dev_pac[i]);} PRINTF("\n\r");
	#endif
	
	sigfoxInitialised = true;
}

void registerSigfox( void ){
	
}

void sendSigfox( void ){
	startEnergyMeasurement(LTC2942_LRWAN);
	
	if(!sigfoxInitialised){
		energyStruct.sigfox_initStatus = 0;
		initSigfox();
	}else{
		energyStruct.sigfox_initStatus = 1; 
	}
	
	
  uint8_t ul_msg[12] = {0x00, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11}; 
  uint8_t dl_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t  ul_size =0;
  uint32_t nbTxRepeatFlag=1;
  int i=0;

  ul_msg[ul_size++] = energyStruct.general_deviceID;
  ul_msg[ul_size++] = energyStruct.general_bootID;
  ul_msg[ul_size++] = (energyStruct.sigfox_packetNumber >> 8) & 0xFF;;
  ul_msg[ul_size++] = energyStruct.sigfox_packetNumber  & 0xFF;
	// Incremental: energyStruct.sigfox_packetNumber%8
	// Random: rand()%8
	for(uint8_t p = 0; p < rand()%8; p++){
		ul_msg[ul_size++] = 0;
	}

	energyStruct.sigfox_payloadSize = ul_size;
	
	SIGFOX_API_close(); // Make sure sigfox api is closed before opening
	
	st_sfx_rc_t SgfxRc=APPLI_RC;
	
	sfx_error_t sfxerror=st_sigfox_open( SgfxRc);															// Open Sifox Lib
  HW_EEPROM_WRITE( E2pData.SgfxKey, CREDENTIALS_KEY_PRIVATE); // Use private key
  if ( sfxerror == SFX_ERR_NONE ){
    PRINTF(" OK\n\r");
  }
  else{
    PRINTF(" error %d\n\r", sfxerror);
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
  sfxerror = SIGFOX_API_send_frame(ul_msg, ul_size, dl_msg, nbTxRepeatFlag, SFX_FALSE);
	if(sfxerror != SFX_ERR_NONE)
			PRINTF_LN("- SIGFOX ERROR %d", sfxerror);
	SIGFOX_API_close();
	
  #ifdef DEBUG
  PRINTF_LN("- Done sending Sigfox data");
	#endif 
  //BSP_LED_Off( LED_BLUE );
	
	uint16_t uwh = stopEnergyMeasurement(LTC2942_LRWAN);
	energyStruct.sigfox_energy = uwh;
	PRINTF("\r\n||Sigfox energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ)||\r\n", energy, uwh, uwh*3.6f);
	
	energyStruct.sigfox_packetNumber++; 
}

#ifndef STDBY_ON 
void send_data_request_from_irq( void * context ){
  /* send task to background*/
  send_data_request();
}
void send_data_request( void ){
  /* send task to background*/
  SCH_SetTask( SEND_TASK );
}
#endif

sfx_error_t st_sigfox_open( st_sfx_rc_t SgfxRc ){
  sfx_error_t sfxerror = SFX_ERR_NONE;

  // Record RCZ
  switch(SgfxRc.id){
    case RC1_ID: {      
      sfxerror = SIGFOX_API_open(&SgfxRc.param);
      break;
    }
    case RC2_ID:{
      sfx_u32 config_words[3] = {RC2_SET_STD_CONFIG_SM_WORD_0, RC2_SET_STD_CONFIG_SM_WORD_1, RC2_SET_STD_CONFIG_SM_WORD_2 };
      sfxerror = SIGFOX_API_open(&SgfxRc.param );
      if ( sfxerror == SFX_ERR_NONE ){
        sfxerror = SIGFOX_API_set_std_config(  config_words, RC2_SET_STD_TIMER_ENABLE);
      }
      break;
    }
    case RC3C_ID:{
      sfx_u32 config_words[3] = {0x00000003,0x00001388,0x00000000};
      sfxerror = SIGFOX_API_open(&SgfxRc.param );
      if ( sfxerror == SFX_ERR_NONE ){
        sfxerror = SIGFOX_API_set_std_config( config_words, NA);
      }
      break;
    }
    case RC4_ID:{
      sfx_u32 config_words[3] = {RC4_SET_STD_CONFIG_SM_WORD_0, RC4_SET_STD_CONFIG_SM_WORD_1, RC4_SET_STD_CONFIG_SM_WORD_2 };
      sfxerror = SIGFOX_API_open(&SgfxRc.param );
      if ( sfxerror == SFX_ERR_NONE ){
        sfxerror = SIGFOX_API_set_std_config( config_words, RC4_SET_STD_TIMER_ENABLE);
      }
      break;
    }
    default:{
      sfxerror = SFX_ERR_API_OPEN;
      break;
    }
  }
  return sfxerror;
}

#ifndef STDBY_ON 
/* when STDBY_ON the reset button is used instead of the push button */
void user_button_init( void ){

  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_PULLUP;
  initStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
  
  /* send everytime button is pushed */
  HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_request_from_irq );
}
#endif

