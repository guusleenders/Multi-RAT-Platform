#ifndef __BG96
#define __BG96

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "hw.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_uart.h"
#include "stm32l0xx_hal_gpio.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_dma.h"
#include "low_power_manager.h"
#include "tiny_vsnprintf.h"
#include "scheduler.h"

#include "vcom.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BG96_RX_PORT GPIOA
#define BG96_RX_PIN GPIO_PIN_13
#define BG96_TX_PORT GPIOA
#define BG96_TX_PIN GPIO_PIN_14
#define BG96_SERIAL_AF GPIO_AF6_LPUART1

#define BG96_POWERKEY_PORT GPIOB
#define BG96_POWERKEY_PIN GPIO_PIN_13														
#define BG96_RESETKEY_PORT GPIOA
#define BG96_RESETKEY_PIN GPIO_PIN_8	

// Inits
void BG96_Init( void );
void BG96_PowerOn( void );

// Serial functions
void BG96_Send( const char *format, ... );
void BG96_Dma_IRQHandler( void );
static void BG96_PrintDMA( void );
static void BG96_StartDMA(char* buf, uint16_t buffLen);
void BG96_IRQHandler(void);
FlagStatus BG96_IsNewCharReceived(void);
uint8_t BG96_GetNewChar(void);
void BG96_ReceiveToBuffer( void );
void BG96_ParseResult( char *buffer );

// AT functions
void BG96_SendATCommand( char *buffer );



#ifdef __cplusplus
}
#endif

#endif /* __BG96 */
