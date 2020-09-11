#ifndef __BG96
#define __BG96

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_uart.h"
#include "stm32l0xx_hal_gpio.h"

#include "vcom.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BG96_RX_PORT GPIOA
#define BG96_RX_PIN GPIO_PIN_10
#define BG96_TX_PORT GPIOA
#define BG96_TX_PIN GPIO_PIN_9
#define BG96_SERIAL_AF GPIO_AF4_USART1

#define BG96_POWERKEY_PORT GPIOB
#define BG96_POWERKEY_PIN GPIO_PIN_13														
#define BG96_RESETKEY_PORT GPIOA
#define BG96_RESETKEY_PIN GPIO_PIN_8	

void BG96_Init( void );
void BG96_PowerOn( void );

void BG96_SendATCommand( char *buffer );

#ifdef __cplusplus
}
#endif

#endif /* __BG96 */
