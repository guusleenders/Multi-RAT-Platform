
#include <stdarg.h>
#include "hw.h"
#include "vcom2.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_dma.h"
#include "low_power_manager.h"
#include "tiny_vsnprintf.h"
#include "scheduler.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef TRACE
#define BUFSIZE_TX 512
#else
#define BUFSIZE_TX 128
#endif

#define BUFSIZE_RX 8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

typedef struct {
  char buff[BUFSIZE_TX];   /* buffer to transmit */
  __IO int iw;             /* 1st free index in BuffTx */
  int ir;                  /* next char to read in buffTx */
  __IO int dmabuffSize;
} circ_buff_tx_t;

typedef struct {
  char buff[BUFSIZE_RX];   /* buffer to receive */
  __IO int iw;             /* 1st free index in BuffRx */
  int ir;                  /* next char to read in buffRx */
} circ_buff_rx_t;

static struct {
  circ_buff_rx_t rx;        /* UART rx buffer context*/
  circ_buff_tx_t tx;        /* UART tx buffer context */
} uart_context;             /* UART context*/

static struct {
  char buffer[10];        /* low power buffer*/
  int len;                /* low power buffer length */
} SleepBuff;              /* low power structure*/



/* Functions Definition ------------------------------------------------------*/
static UART_HandleTypeDef UartHandle;

void vcom2_Init(void (*TxCb)(void)){
	
	__USART2_CLK_ENABLE();
  vcom2_IoInit();
	
	UartHandle.Instance  = USART2;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
	
	HAL_UART_Init(&UartHandle);
	
}

void vcom2_DeInit(void){
  //TODO 
	LL_LPUART_DeInit(UARTX);
}

void vcom2_IoInit(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = UARTX_RX_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = GPIO_AF4_USART2;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UARTX_RX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = UARTX_TX_PIN;
  HAL_GPIO_Init(UARTX_TX_GPIO_PORT, &GPIO_InitStructure);

}

void vcom2_IoDeInit(void){
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HW_GPIO_Init(UARTX_TX_GPIO_PORT, UARTX_TX_PIN, &GPIO_InitStructure);
  HW_GPIO_Init(UARTX_RX_GPIO_PORT, UARTX_RX_PIN, &GPIO_InitStructure);
}


void vcom2_Send( const char *format, ... ){
  va_list args;
  va_start(args, format);
 
  uint8_t lenTop;
  char tempBuff[BUFSIZE_TX];
  
  int32_t freebuff;
  int len=0;

  if (SleepBuff.len!=0)
  {
    /*if SleepBuff has been filled before entering lowpower, prepend it */
    memcpy(&tempBuff[0], SleepBuff.buffer, SleepBuff.len);
    len = tiny_vsnprintf_like(&tempBuff[SleepBuff.len], sizeof(tempBuff), format, args); 
    len += SleepBuff.len;
    /*erase SleepBuff*/
    memset(SleepBuff.buffer, 0,sizeof(SleepBuff.buffer) );
    SleepBuff.len=0;
  }
  else
  {
  /*convert into string at buff[0] of length iw*/
    len = tiny_vsnprintf_like(&tempBuff[0], sizeof(tempBuff), format, args); 
  }
  
  /* calculate free buffer size*/
  freebuff = BUFSIZE_TX - ((BUFSIZE_TX+uart_context.tx.iw-uart_context.tx.ir)%BUFSIZE_TX);
	
	
	HAL_UART_Transmit(&UartHandle, (uint8_t *) tempBuff, len, HAL_MAX_DELAY);
  
  va_end(args);
}
