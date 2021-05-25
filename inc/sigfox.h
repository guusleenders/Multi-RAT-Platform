#ifndef __SIGFOX
#define __SIGFOX

#include <stdint.h>
#include <stdbool.h>

#include "hw.h"
#include "scheduler.h"
#include "st_sigfox_api.h"
#include "radio.h"
#include "sgfx_credentials.h"
#include "hw_eeprom.h"
#include "bsp.h"
#include "mlm32l07x01.h"
#include "sgfx_sx1276_driver.h"
#include "vcom2.h"

#include "energy.h"

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------- SIGFOX DEFINITIONS ------------------------------
#define PAC_LEN 8
#define ID_LEN 4
/* Default Configuration zone*/
/* Can be changed according to application zone*/
#define APPLI_RC   ST_RC1

#define SIGFOX_POWER 14

//sfx_u8 error = 0;

// --------------------------- SIGFOX FUNCTIONS --------------------------------
void initSigfox( void );													 // Init Sigfox modem
void registerSigfox( void );										   // Register Sigfox to network (empty function)
void sendSigfox( void );													 // Send Sigfox data

static sfx_error_t st_sigfox_open( st_sfx_rc_t SgfxRc ); // Open the sigfox library; @param The Region configuration

#ifndef STDBY_ON 
void send_data_request( void  );					 // To post interrupt to backgroud; managed by scheduler
void send_data_request_from_irq( void * context );
void user_button_init( void );									   // Initialize the user btton to request sending data
																												 // when STDBY_ON the reset button is used instead of the push button 
#endif


#ifdef __cplusplus
}
#endif

#endif /* __SIGFOX */

