#include "bg96.h"

#ifdef TRACE
#define BUFSIZE_TX 512
#else
#define BUFSIZE_TX 128
#endif

#define BUFSIZE_RX 8

static UART_HandleTypeDef BG96_UARTHandle;

static void BG96_Receive(char rx); //Takes one character that has been received and save it in uart_context.buffRx
static void BG96_PrintDMA(void); // prepare DMA print
static void BG96_StartDMA(char* buf, uint16_t buffLen); //Starts DMA transfer into UART


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
} SleepBuff;

void BG96_Init( void ){
	// --- Init Serial stuff ---
	__LPUART1_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Pin = BG96_RX_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = BG96_SERIAL_AF;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BG96_RX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BG96_TX_PIN;
  HAL_GPIO_Init(BG96_TX_PORT, &GPIO_InitStructure);
	

	LL_LPUART_InitTypeDef LPUART_InitStruct;
  /*## Configure the UART peripheral ######################################*/


  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_HSI);
  UARTX_CLK_ENABLE();

	/*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for UART */
  HAL_NVIC_SetPriority(UARTX_IRQn, IRQ_PRIORITY_USARTX, 0);
  HAL_NVIC_EnableIRQ(UARTX_IRQn);

  LPUART_InitStruct.BaudRate = 9600;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
  LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;

  LL_LPUART_Init(UARTX, &LPUART_InitStruct);
    /* Configuring the LPUART specific LP feature - the wakeup from STOP */
  LL_LPUART_EnableInStopMode(UARTX);

  LL_LPUART_Enable(UARTX);

  while (LL_LPUART_IsActiveFlag_TEACK(UARTX) == RESET);
  while (LL_LPUART_IsActiveFlag_REACK(UARTX) == RESET);
	
	// --- Receive init ---
	/* enable RXNE */
  LL_LPUART_EnableIT_RXNE(UARTX);
  /* WakeUp from stop mode on start bit detection*/
  LL_LPUART_SetWKUPType(UARTX, LL_LPUART_WAKEUP_ON_STARTBIT);

  LL_LPUART_EnableIT_WKUP(UARTX);
  /* Enable the UART Parity Error Interrupt */
  LL_LPUART_EnableIT_PE(UARTX);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  LL_LPUART_EnableIT_ERROR(UARTX);
	
	SCH_RegTask( VCOM_TASK, BG96_ReceiveToBuffer );
	
	// --- Init GPIO for power pins ---
	// Power pin
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = BG96_POWERKEY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BG96_POWERKEY_PORT, &GPIO_InitStruct);
	// Reset
	GPIO_InitStruct.Pin = BG96_RESETKEY_PIN;
	HAL_GPIO_Init(BG96_RESETKEY_PORT, &GPIO_InitStruct);
}

void BG96_Send( const char *format, ... ){
  va_list args;
  va_start(args, format);
 
  uint8_t lenTop;
  char tempBuff[BUFSIZE_TX];
  
  int32_t freebuff;
  int len=0;

  if (SleepBuff.len!=0){
    /*if SleepBuff has been filled before entering lowpower, prepend it */
    memcpy(&tempBuff[0], SleepBuff.buffer, SleepBuff.len);
    len = tiny_vsnprintf_like(&tempBuff[SleepBuff.len], sizeof(tempBuff), format, args); 
    len += SleepBuff.len;
    /*erase SleepBuff*/
    memset(SleepBuff.buffer, 0,sizeof(SleepBuff.buffer) );
    SleepBuff.len=0;
  }
  else{
  /*convert into string at buff[0] of length iw*/
    len = tiny_vsnprintf_like(&tempBuff[0], sizeof(tempBuff), format, args); 
  }
  
  /* calculate free buffer size*/
  freebuff = BUFSIZE_TX - ((BUFSIZE_TX+uart_context.tx.iw-uart_context.tx.ir)%BUFSIZE_TX);

  while (len>freebuff){
    /*wait enough free char in buff*/
    /*1 char at 9600 lasts approx 1ms*/
    /*even in the while loop, DMA IRQ handler will update iw-uart_context.tx.ir*/
    DelayMs(1);
    freebuff = BUFSIZE_TX - ((BUFSIZE_TX+uart_context.tx.iw-uart_context.tx.ir)%BUFSIZE_TX);
    
    /*in case vcom_Send called by higher or equal priority irq than DMA priority, 
     vcom_Dma_IRQHandler should be triggered by software*/
    BACKUP_PRIMASK();

    DISABLE_IRQ( );
    if (HAL_NVIC_GetPendingIRQ(DMA1_Channel4_5_6_7_IRQn)== 1)
    {
      BG96_Dma_IRQHandler();
    }
    ENABLE_IRQ( );
  }

  if ((uart_context.tx.iw+len)<BUFSIZE_TX){
    memcpy( &uart_context.tx.buff[uart_context.tx.iw], &tempBuff[0], len);
    uart_context.tx.iw+=len;
  }
  else{
    /*cut buffer in high/low part*/
    lenTop= BUFSIZE_TX - (uart_context.tx.iw);
    /*copy beginning at top part of the circ buf*/
    memcpy( &uart_context.tx.buff[uart_context.tx.iw], &tempBuff[0], lenTop);
     /*copy end at bottom part of the circ buf*/
    memcpy( &uart_context.tx.buff[0], &tempBuff[lenTop], len-lenTop);
    uart_context.tx.iw = len-lenTop;
  }

  if (! LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_7) ){
    BG96_PrintDMA();
  }
  
  
  va_end(args);
}

FlagStatus BG96_IsNewCharReceived(void){
  FlagStatus status;
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  status = ((uart_context.rx.iw == uart_context.rx.ir) ? RESET : SET);
  
  RESTORE_PRIMASK();
  return status;
}

void BG96_IRQHandler(void){
  if ( LL_LPUART_IsActiveFlag_TC(UARTX) && (LL_LPUART_IsEnabledIT_TC(UARTX) != RESET) ){/*tx*/
    /*last uart char has just been sent out to terminal*/
    LL_LPUART_ClearFlag_TC(UARTX);
    /*enable lowpower since finished*/
    LPM_SetStopMode(LPM_UART_TX_Id, LPM_Enable);
  }
  /*rx*/
  {
    __IO int rx_ready = 0;
    char rx = AT_ERROR_RX_CHAR;
    
    /* UART Wake Up interrupt occured ------------------------------------------*/
    if (LL_LPUART_IsActiveFlag_WKUP(UARTX) && (LL_LPUART_IsEnabledIT_WKUP(UARTX) != RESET)){
      LL_LPUART_ClearFlag_WKUP(UARTX);

      /* forbid stop mode */
      LPM_SetStopMode(LPM_UART_RX_Id, LPM_Disable);
    }

    if (LL_LPUART_IsActiveFlag_RXNE(UARTX) && (LL_LPUART_IsEnabledIT_RXNE(UARTX) != RESET)){
      /* no need to clear the RXNE flag because it is auto cleared by reading the data*/
      rx = LL_LPUART_ReceiveData8(UARTX);
      rx_ready = 1;
      
      /* allow stop mode*/
      LPM_SetStopMode(LPM_UART_RX_Id, LPM_Enable);
    }

    if (LL_LPUART_IsActiveFlag_PE(UARTX) || LL_LPUART_IsActiveFlag_FE(UARTX) || LL_LPUART_IsActiveFlag_ORE(UARTX) || LL_LPUART_IsActiveFlag_NE(UARTX)){
      DBG_PRINTF("Error when receiving\n\r");
      /* clear error IT */
      LL_LPUART_ClearFlag_PE(UARTX);
      LL_LPUART_ClearFlag_FE(UARTX);
      LL_LPUART_ClearFlag_ORE(UARTX);
      LL_LPUART_ClearFlag_NE(UARTX);
      
      rx = AT_ERROR_RX_CHAR;
      
      rx_ready = 1;
    }
    
    if (rx_ready == 1){
      BG96_Receive(rx);
    }
  }
}

static void BG96_Receive(char rx){
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.rx.buff[uart_context.rx.iw] = rx;
  next_free = (uart_context.rx.iw + 1) % sizeof(uart_context.rx.buff);
  if (next_free != uart_context.rx.iw){
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx.iw = next_free;
  }
  else{
    /* force the end of a command in case of overflow so that we can process it */
    uart_context.rx.buff[uart_context.rx.iw] = '\r';
    DBG_PRINTF("uart_context.buffRx buffer overflow %d\r\n");
  }
  SCH_SetTask(VCOM_TASK);
}

uint8_t BG96_GetNewChar(void){
  uint8_t NewChar;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  NewChar = uart_context.rx.buff[uart_context.rx.ir];
  uart_context.rx.ir = (uart_context.rx.ir + 1) % sizeof(uart_context.rx.buff);

  RESTORE_PRIMASK();
  return NewChar;
}

void BG96_Dma_IRQHandler( void ){
  if (LL_DMA_IsActiveFlag_TC7(DMA1) ){
    /*clear interrupt and flag*/
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_7);
    LL_DMA_ClearFlag_TC7(DMA1);
    /* update tx read index*/
    uart_context.tx.ir += uart_context.tx.dmabuffSize;
    uart_context.tx.ir = (uart_context.tx.ir)%BUFSIZE_TX;
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_7);
    LL_LPUART_DisableDMAReq_TX(UARTX);
  }
  if ( uart_context.tx.ir!= uart_context.tx.iw){
    /*continue if more has been written in buffer meanwhile*/
    BG96_PrintDMA();
  }
}

static void BG96_PrintDMA(void){
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  uint16_t write_idx=  (uart_context.tx.iw);
  uint16_t read_idx=   (uart_context.tx.ir);
  /*shall not go in stop mode while printing*/
  LPM_SetStopMode(LPM_UART_TX_Id, LPM_Disable);

  if (write_idx > read_idx){
    /*contiguous buffer[ir..iw]*/
    uart_context.tx.dmabuffSize= write_idx - read_idx;

    BG96_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
  else{
    /*[ir:BUFSIZE_TX-1] and [0:iw]. */
     uart_context.tx.dmabuffSize= BUFSIZE_TX-read_idx;
     /*only [ir:BUFSIZE_TX-1] sent, rest will be sent in dma  handler*/
     BG96_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
  RESTORE_PRIMASK();
}

static void BG96_StartDMA(char* buf, uint16_t buffLen){
  LL_DMA_InitTypeDef DMA_InitStruct;
  /*switch dma clock ON*/
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  /*dma initialisation*/  
  DMA_InitStruct.Direction= LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.PeriphOrM2MSrcAddress=(uint32_t) &UARTX->TDR;
  DMA_InitStruct.PeriphOrM2MSrcDataSize= LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;  
  DMA_InitStruct.Mode= LL_DMA_MODE_NORMAL;
  DMA_InitStruct.MemoryOrM2MDstAddress= (uint32_t) buf;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.MemoryOrM2MDstDataSize= LL_DMA_MDATAALIGN_BYTE;  
  DMA_InitStruct.NbData= buffLen;  
  DMA_InitStruct.PeriphRequest=LL_DMA_REQUEST_5;
  DMA_InitStruct.Priority=LL_DMA_PRIORITY_LOW;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_7,&DMA_InitStruct );
    /*enable DMA transmit complete interrupt*/
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
  /* enable DMA nvic*/
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, IRQ_PRIORITY_USARTX, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  /* enable LPUART DMA request*/
  LL_LPUART_EnableDMAReq_TX(UARTX);
  /*enable DMA channel*/
  LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_7); 
  /*enable LPUART transmitt complete interrupt*/
  LL_LPUART_EnableIT_TC(UARTX);
}

void BG96_PowerOn( void ){
	HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(500);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_RESET); 
  
  HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(500);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_RESET); 
	
	BG96_SendATCommandCheckReply("","RDY", 10000);
	BG96_SendATCommandCheckReply("AT\r\n","OK", 1000);
	BG96_SendATCommandCheckReply("ATE0\r\n","OK", 1000);
}

#define REPLY_SIZE 50

static bool totalReplyBufferDone = false;
static bool waitingForReply = false;
static char totalReplyBuffer[REPLY_SIZE];
void BG96_ReceiveToBuffer( void ){
  static char replyBuffer[REPLY_SIZE];
  static unsigned i = 0;
  unsigned cmd_size = 0;

  /* Process all commands */
  while (BG96_IsNewCharReceived() == SET){
		
    replyBuffer[i] = BG96_GetNewChar();

		#if 0 /* echo On    */
    PRINTF("%c", command[i]);
		#endif

    if (replyBuffer[i] == AT_ERROR_RX_CHAR){
      memset(replyBuffer, '\0', i);
      i = 0;
      //com_error(AT_RX_ERROR);
      break;
    }
    else{
			if ((replyBuffer[i] == '\r') || (replyBuffer[i] == '\n')){
				if (i != 0){
					replyBuffer[i] = '\0';
					/* need to put static i=0 to avoid realtime issue when CR+LF is used by hyperterminal */
					cmd_size = i; 
					i = 0;
					if(waitingForReply)
							BG96_ParseResult(replyBuffer);
					
					memset(replyBuffer, '\0', cmd_size);

				}
			}
			else if (i == (REPLY_SIZE - 1)){
				memset(replyBuffer, '\0', i);
				i = 0;
				//com_error(AT_TEST_PARAM_OVERFLOW);
			}
			else{
				i++;
			}
		}
  }
}

void BG96_ParseResult( char *buffer ){
	totalReplyBufferDone = true;
	memcpy(totalReplyBuffer, buffer, REPLY_SIZE);
}

void BG96_SendATCommand( char *buffer ){
	if(strlen(buffer)>0)
		BG96_Send(buffer);
}

BG96_Status_t BG96_SendATCommandCheckReply( char *buffer , char *replyBuffer, uint16_t timeout){
	BG96_SendATCommand(buffer);
	waitingForReply = true;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(!totalReplyBufferDone && ( ( tickNow - tickstart ) ) < timeout){
    SCH_Run(); 
		tickNow = HW_RTC_GetTimerValue();
  }
	totalReplyBufferDone = false;
	waitingForReply = false;
	if(StringStartsWith(totalReplyBuffer, replyBuffer)){
		return BG96_OK;
	}else if(( tickNow - tickstart )  >= timeout){
		PRINTF("TIMEOUT");
		return BG96_TIMEOUT;
	}else{
		PRINTF("ERROR %s", totalReplyBuffer);
		return BG96_ERROR;
	}
}

BG96_Status_t BG96_SendATCommandGetReply( char *buffer , char *replyBuffer, uint16_t timeout){
	BG96_SendATCommand(buffer);
	waitingForReply = true;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(!totalReplyBufferDone && ( ( tickNow - tickstart ) ) < timeout){
    SCH_Run(); 
		tickNow = HW_RTC_GetTimerValue();
  }
	totalReplyBufferDone = false;
	waitingForReply = false;
	if(( tickNow - tickstart )  >= timeout){
		PRINTF("TIMEOUT");
		return BG96_TIMEOUT;
	}else{
		strcpy(replyBuffer, totalReplyBuffer);
		return BG96_OK;
	}
}

