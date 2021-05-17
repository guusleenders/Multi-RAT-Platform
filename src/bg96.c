#include "bg96.h"

#ifdef TRACE
#define BUFSIZE_TX 512
#else
#define BUFSIZE_TX 128
#endif

#define BUFSIZE_RX 8

#define FIRSTBOOT

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

static bool totalReplyBufferDone = false;
static bool waitingForReply = false;
static char totalReplyBuffer[BG96_REPLY_SIZE];

// TCP properties
uint8_t _contextID = 1;
uint8_t _connectID = 1;

// HTTP properties
uint8_t _responseHeaderID = 1;
uint8_t _requestHeaderID = 1;
uint8_t _SSLContextID = 1;
uint8_t _contentType = 1;

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


  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_HSI); // Change to RCC_CCIPR_LPUART1SEL for low power-ness, first boot LL_RCC_LPUART1_CLKSOURCE_HSI
  UARTX_CLK_ENABLE();

	/*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for UART */
  HAL_NVIC_SetPriority(UARTX_IRQn, IRQ_PRIORITY_USARTX, 0);
  HAL_NVIC_EnableIRQ(UARTX_IRQn);

LPUART_InitStruct.BaudRate = 9600; // Change to 9600 for low power-ness; first boot: 115200
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
	HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_RESET);
	// Reset
	GPIO_InitStruct.Pin = BG96_RESETKEY_PIN;
	HAL_GPIO_Init(BG96_RESETKEY_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_RESET);
	// DTR
	GPIO_InitStruct.Pin = BG96_DTR_PIN;
	HAL_GPIO_Init(BG96_DTR_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BG96_DTR_PORT, BG96_DTR_PIN, GPIO_PIN_RESET);
	// Status
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( BG96_STATUS_PORT, BG96_STATUS_PIN, &GPIO_InitStruct );

	
}

void BG96_DeInit(void){
  LL_LPUART_DeInit(UARTX);
}

void BG96_IoDeInit(void){
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HW_GPIO_Init(BG96_TX_PORT, BG96_TX_PIN, &GPIO_InitStructure);

  HW_GPIO_Init(BG96_RX_PORT, BG96_RX_PIN, &GPIO_InitStructure);
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


void BG96_ReceiveToBuffer( void ){
  static char replyBuffer[BG96_REPLY_SIZE];
  static unsigned i = 0;
  unsigned cmd_size = 0;

  /* Process all commands */
  while (BG96_IsNewCharReceived() == SET){
		
    replyBuffer[i] = BG96_GetNewChar();

		#if 0 /* echo On    */
    PRINTF("%c", replyBuffer[i]);
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
			}else if(replyBuffer[i] == '>'){ // include '>' for data input 
					/* need to put static i=0 to avoid realtime issue when CR+LF is used by hyperterminal */
					cmd_size = i; 
					i = 0;
					if(waitingForReply)
							BG96_ParseResult(replyBuffer);
					memset(replyBuffer, '\0', cmd_size);
			}
			else if (i == (BG96_REPLY_SIZE - 1)){
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
	memcpy(totalReplyBuffer, buffer, BG96_REPLY_SIZE);
}

void BG96_SendATCommand( char *buffer ){
	if(strlen(buffer)>0)
		BG96_Send(buffer);
}

BG96_Status_t BG96_SendATCommandCheckReply( char *buffer , char *replyBuffer, uint32_t timeout){
	BG96_SendATCommand(buffer);
	waitingForReply = true;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(!totalReplyBufferDone && ( ( tickNow - tickstart ) ) < timeout){
    SCH_Run(); 
		tickNow = HW_RTC_GetTimerValue();
  }
	HAL_Delay(2);
	totalReplyBufferDone = false;
	waitingForReply = false;
	if(StringStartsWith(totalReplyBuffer, replyBuffer)){
		return BG96_OK;
	}else if(( tickNow - tickstart )  >= timeout){
		PRINTF_LN("TIMEOUT");
		return BG96_TIMEOUT;
	}else{
		PRINTF_LN("ERROR _%s_%s", totalReplyBuffer, replyBuffer);
		return BG96_ERROR;
	}
}

BG96_Status_t BG96_SendATCommandGetReply( char *buffer , char *replyBuffer, uint32_t timeout){
	BG96_SendATCommand(buffer);
	waitingForReply = true;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(!totalReplyBufferDone && ( ( tickNow - tickstart ) ) < timeout){
    SCH_Run(); 
		tickNow = HW_RTC_GetTimerValue();
  }
	HAL_Delay(2);
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
	
BG96_Status_t BG96_PowerOn( void ){
	//BG96_SaveConfiguration();
	HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(500);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_RESET); 
  
  HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(400);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_RESET); 
	
	//HAL_Delay(20000);
	//BG96_SetBaudRate(9600);
	//char powerOnBuffer[10];
	BG96_Status_t status = BG96_SendATCommandCheckReply("", "RDY", 10000); // RDY
	status = BG96_SendATCommandGetReply("", "APP RDY", 2000); // APP RDY // TODO: if this is RDY again, wait for APP RDY
	//status = BG96_SendATCommandCheckReply("ATE0\r\n", "ATE0", 1000);
	//BG96_SaveConfiguration();
	if(status != BG96_OK)
		return status;
	status = BG96_SendATCommandCheckReply("AT\r\n", "OK", 1000);
	if(status != BG96_OK)
		return status;
	status = BG96_ResetParameters();
	if(status != BG96_OK)
		return status;
	status = BG96_SendATCommandCheckReply("ATE0\r\n", "ATE0", 1000);
	BG96_SendATCommandCheckReply("", "OK", 1000);
	
		
	#ifdef FIRSTBOOT
	BG96_Status_t BG96_SetBaudRate( uint16_t baud );
	#endif
	
	return status;
}

BG96_Status_t BG96_SetBaudRate( uint16_t baud ){
	char buffer[30]; 
	sprintf(buffer, "AT+IPR=%d\r\n", baud);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
	
	// TODO: Auto save via BG96_SaveConfiguration (but with new baud rate)
}

BG96_Status_t BG96_PowerDown( void ){
	BG96_Status_t status  = BG96_SendATCommandCheckReply("AT+QPOWD=1\r\n", "OK", 300);
	if(status != BG96_OK)
		return status;
  status = BG96_SendATCommandCheckReply("", "POWERED DOWN", 60000);
	return status;
}

BG96_Status_t BG96_SaveConfiguration( void ){
	return BG96_SendATCommandCheckReply("AT&W\r\n", "OK", 300);
}

BG96_Status_t BG96_ResetConfiguration( void ){
	return BG96_SendATCommandCheckReply("ATZ\r\n", "OK", 300);
}

BG96_Status_t BG96_ResetParameters( void ){
	return BG96_SendATCommandCheckReply("AT&F0\r\n", "OK", 300);
}

BG96_Status_t BG96_GetIMEI( char* buffer ){
	BG96_Status_t status  = BG96_SendATCommandGetReply("AT+GSN\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetFirmwareInfo( char* buffer ){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+CGMR\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetHardwareInfo( char* buffer ){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+CGMM\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_SetErrorFormat( uint8_t error ){
	char buffer[30]; 
	sprintf(buffer, "AT+CMEE=%d\r\n", error);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_SetNetworkReporting( uint8_t creg ){
	char buffer[30]; 
	sprintf(buffer, "AT+CREG=%d\r\n", creg);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_CheckSIMPIN( char* reply ){
	PRINTF_LN("check sim pin");
	BG96_SendATCommandGetReply("AT+CPIN?\r\n", reply, 300);
	return BG96_SendATCommandCheckReply("", "OK", 300);
}

BG96_Status_t BG96_SetSIMPIN( char* pin ){
	char buffer[5]; 
	sprintf(buffer, "AT+CPIN=%s\r\n", pin);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_SetGSMBand( char* band ){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"band\",%s,%s,%s\r\n", band, BG96_LTE_NO_CHANGE, BG96_LTE_NO_CHANGE);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_SetCatM1Band( char* band ){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"band\",%s,%s,%s\r\n", BG96_GSM_NO_CHANGE, band, BG96_LTE_NO_CHANGE);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_SetNBIoTBand( char* band ){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"band\",%s,%s,%s\r\n", BG96_GSM_NO_CHANGE, BG96_LTE_NO_CHANGE, band);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_GetBandConfiguration( char* buffer ){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QCFG=\"band\"\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}
 
BG96_Status_t BG96_SetScrambleConfiguration( bool scramble ){
	if(scramble){
		return BG96_SendATCommandCheckReply("AT+QCFG=\"nbsibscramble\",0\r\n", "OK", 300);
	}else{
		return BG96_SendATCommandCheckReply("AT+QCFG=\"nbsibscramble\",1\r\n", "OK", 300);
	}
}

BG96_Status_t BG96_SetMode( uint8_t mode){
	BG96_Status_t status;
	if(mode == BG96_AUTO_MODE){
		// Modem configuration : AUTO_MODE
    // *Priority Table (Cat.M1 -> Cat.NB1 -> GSM)
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanseq\",00,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanmode\",0,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"iotopmode\",2,1\r\n", "OK", 300);
  }else if(mode == BG96_GSM_MODE){
		// Modem configuration : GSM_MODE
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanseq\",01,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanmode\",1,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"iotopmode\",2,1\r\n", "OK", 300); 
  }else if(mode == BG96_CATM1_MODE){
    //  Modem configuration : CATM1_MODE
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanseq\",02,1", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanmode\",3,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"iotopmode\",0,1\r\n", "OK", 300);
  }else if(mode == BG96_NBIOT_MODE){
    // Modem configuration : CATNB1_MODE ( NB-IoT )
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanseq\",03,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"nwscanmode\",3,1\r\n", "OK", 300);
    status = BG96_SendATCommandCheckReply("AT+QCFG=\"iotopmode\",1,1\r\n", "OK", 300);
  }
	return status;
}

BG96_Status_t BG96_GetSignalQuality(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+CSQ\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetSignalStength(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QCSQ\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetNetworkInfo(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QNWINFO\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetNetworkStatus(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandCheckReply("AT+CEREG=4\r\n", "OK", 300);
	if(status != BG96_OK)
			return status;
	status = BG96_SendATCommandGetReply("AT+CEREG?\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetCELevel(uint8_t * celevel){
  char buffer[50];
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QCFG=\"celevel\"\r\n", buffer, 300);
	if(status != BG96_OK)
			return status;
	if(!StringStartsWith(buffer, "+CME ERROR")){
		// in buffer: +QCFG: "celevel",<celevel>
		*celevel = buffer[17]-'0';
	}
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_DisableNetworkStatus( void ){
	BG96_Status_t status  =  BG96_SendATCommandCheckReply("AT+CEREG=0\r\n", "OK", 300);
	if(status != BG96_OK)
			return status;
	return status; // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_GetAvailableNetworks(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+COPS=?\r\n", buffer, 48928);
	if(status != BG96_OK)
			return status;
	return BG96_SendATCommandCheckReply("", "OK", 300); // Check out last (closing) OK\r\n
}

BG96_Status_t BG96_SelectNetwork(uint16_t networkId, uint8_t mode){
	char buffer[50]; 
	sprintf(buffer, "AT+COPS=1,2,\"%d\",%d\r\n", networkId, mode);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "OK", 48928); // Delay up to 180s, limited by int size
	//BG96_SendATCommandCheckReply("", "APP RDY", 300);
	//BG96_SendATCommandCheckReply("", "+CEREG", 300);
	return status;
}

BG96_Status_t BG96_SetPDPContext(char * url){
	char buffer[50]; 
	sprintf(buffer, "AT+CGDCONT=1,\"IP\",%s\r\n", url);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300); 
}

BG96_Status_t BG96_ConnectToOperator( uint32_t timeout ){
	PRINTF_LN("Started connecting to operator");
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	BG96_Status_t status  =  BG96_ERROR; 
	char buffer[30];
	while(status != BG96_OK  && ( ( tickNow - tickstart ) ) < timeout){
		status = BG96_SendATCommandCheckReply("AT+CGATT?\r\n", "+CGATT: 1", 300);
		BG96_SendATCommandGetReply("", buffer, 300); // Check out last (closing) OK\r\n
		if(StringStartsWith(buffer, "+CGATT: 1")){ // If cgat 1 is detected instead of OK, then is connected!
			status = BG96_OK;
		}
		if(status != BG96_OK)
			HAL_Delay(100+rand()%200); // Wait for random amount of time (100-299ms)
		tickNow = HW_RTC_GetTimerValue();
	}
	if(( tickNow - tickstart ) >= timeout){
		return BG96_TIMEOUT;
	}
	return status; 
}

BG96_Status_t BG96_SetEDRXConfiguration(uint8_t enable, uint8_t mode, char * edrx){
	char buffer[30]; 
	sprintf(buffer, "AT+CEDRXS=%d,%d,%s\r\n", enable, mode, edrx);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_GetEDRXConfiguration(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+CEDRXRDP\r\n", buffer, 300);
	if(status != BG96_OK)
		return status;
	return BG96_SendATCommandCheckReply("", "OK", 300);
}

BG96_Status_t BG96_GetPacketCounter( char* buffer ){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QGDCNT?\r\n", buffer, 300);
	if(status != BG96_OK)
		return status;
	return BG96_SendATCommandCheckReply("", "OK", 300);
}

BG96_Status_t BG96_ConfigureURCIndication( char* indication ){
	char buffer[30]; 
	sprintf(buffer, "AT+QURCCFG=\"urcport\",%s\r\n", indication);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_SetModemOptimization( void ){
	return BG96_SendATCommandCheckReply("AT+QPSMEXTCFG=6,1,120,2,1,1\r\n\r\n", "OK", 300);
}


BG96_Status_t BG96_Sleep( void ){
	BG96_Status_t status = BG96_SendATCommandCheckReply("AT+QSCLK=1\r\n", "OK", 300);
	HAL_GPIO_WritePin(BG96_DTR_PORT, BG96_DTR_PIN, GPIO_PIN_SET); 
	return status;
}

BG96_Status_t BG96_Wake( void ){
	HAL_GPIO_WritePin(BG96_DTR_PORT, BG96_DTR_PIN, GPIO_PIN_SET); 
	return BG96_SendATCommandCheckReply("", "RDY", 6000);
}

BG96_Status_t BG96_WakeFromPSM( uint32_t timeout ){
	BG96_Status_t status = BG96_ERROR;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(BG96_IsPoweredDown() && ( ( tickNow - tickstart ) ) < timeout){
		HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_SET); // Wake module
		HAL_Delay(80);
		HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_RESET); // Wake module
		
		//HAL_Delay(4000); // Let reconnection establish
		tickNow = HW_RTC_GetTimerValue();
	}
	status = BG96_SendATCommandCheckReply("", "RDY", 7000);
	PRINTF_LN("Got RDY");
	status = BG96_SendATCommandCheckReply("", "APP RDY", 7000);
	PRINTF_LN("GOT APPRDY");
	if(BG96_IsPoweredDown()){
		return BG96_TIMEOUT;
	}else{
		return BG96_OK;
	}
}

BG96_Status_t BG96_WakeFromPSMToSend( void ){
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Status: powered down");
		PRINTF_LN("- Waking from psm");
		if(BG96_WakeFromPSM(10000) == BG96_OK){
			PRINTF("- Wake from PSM ok");
			if(BG96_WaitForConnection(20000) != BG96_OK){
				PRINTF("- Wait for connection fail");
				return BG96_ERROR;
			}else{
				PRINTF("- Wait for connection ok");
			}
		}else{
			PRINTF_LN("- Wake from PSM fail");
			return BG96_ERROR;
		}
	}else{
		PRINTF_LN("- Status: powered on");
	}

	if(!BG96_CheckIfContextActivated()){
		PRINTF_LN("- Context not active");
		BG96_ActivateContext();
		PRINTF_LN("- Context activated");
	}else{
		PRINTF_LN("- Context already active");
	}
	return BG96_OK;
}

// Not tested
BG96_Status_t BG96_GetTime(char * timeresult){
	char response[30];
	BG96_Status_t status  =  BG96_ERROR; 
	status = BG96_SendATCommandGetReply("AT+QLTS=2", response, 300);
	if(status != BG96_OK)
		return status;
	
	char * startLocation = strstr(response, "+QLTS: ");
	if(startLocation == NULL)
			return BG96_ERROR;
	
	strncpy(timeresult, startLocation+9, 2);
	return BG96_OK;
}

BG96_Status_t BG96_AbortRRC(){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"rrcabort\",1\r\n");
	return BG96_SendATCommandCheckReply(buffer, "OK", 1000);
}

BG96_Status_t BG96_ConfigureFeatures(char * val){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"nccconfig\",%s\r\n", val);
	return BG96_SendATCommandCheckReply(buffer, "OK", 1000);
}


BG96_Status_t BG96_SetPowerSavingMode(uint8_t mode, char * requestedRAU, char * requestedGPRSREADY, char * requestedTAU, char * requestedActiveTimer){
	char buffer[100]; 
	sprintf(buffer, "AT+QPSMS=%d,%s,%s,%s,%s\r\n", mode, requestedRAU, requestedGPRSREADY, requestedTAU, requestedActiveTimer);
	return BG96_SendATCommandCheckReply(buffer, "OK", 1000);
}

BG96_Status_t BG96_SetPowerSavingModeImmediately(){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"psm/enter\",1\r\n");
	return BG96_SendATCommandCheckReply(buffer, "OK", 1000);
}

BG96_Status_t BG96_GetPowerSavingMode(char * buffer){
	BG96_Status_t status  =  BG96_SendATCommandGetReply("AT+QPSMS?\r\n", buffer, 300); // QPSMS in stead of CPSMS: returns network parameters in stead of requested things
	if(status != BG96_OK)
		return status; 
	return BG96_SendATCommandCheckReply("", "OK", 300);
}

BG96_Status_t BG96_SetPowerSavingModeSettings(uint32_t threshold, uint8_t version){
	char buffer[30]; 
	sprintf(buffer, "AT+QPSMCFG=%d,%d\r\n", threshold, version);
	return BG96_SendATCommandCheckReply(buffer, "OK", 1000);
}

BG96_Status_t BG96_EnablePSMIndication( void ){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"psm/urc\",1\r\n");
	return BG96_SendATCommandCheckReply(buffer, "ERROR", 1000);
}

BG96_Status_t BG96_DisablePSMIndication( void ){
	char buffer[30]; 
	sprintf(buffer, "AT+QCFG=\"psm/urc\",0\r\n");
	return BG96_SendATCommandCheckReply(buffer, "ERROR", 1000);
}

BG96_Status_t BG96_WaitForPowerDown( uint32_t timeout ){
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	PRINTF_LN("- Ticks started at %08x", tickstart);
	while(!BG96_IsPoweredDown() && ( ( tickNow - tickstart ) ) < timeout){
		HAL_Delay(50);
		tickNow = HW_RTC_GetTimerValue();
	}
	if(( tickNow - tickstart ) >= timeout){
		return BG96_TIMEOUT;
	}
	return BG96_OK;
}

bool BG96_IsPoweredDown(){
	if(HAL_GPIO_ReadPin(BG96_STATUS_PORT, BG96_STATUS_PIN) == GPIO_PIN_SET)
		return true;
	else
		return false;
}

BG96_Status_t BG96_WaitForConnection(uint32_t timeout){
	char buffer[100];
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	BG96_Status_t status = BG96_ERROR;
	BG96_SendATCommandCheckReply("AT+CEREG=2\r\n", "OK", 300);
	while(status != BG96_OK && ( ( tickNow - tickstart ) ) < timeout){
		//HAL_Delay(500+rand()%500);
		status = BG96_SendATCommandGetReply("AT+CEREG?\r\n", buffer, 1000);
		if(status == BG96_OK){
			char *search = strchr(buffer, '9'); //Check if nbiot is connected = mode 9
			PRINTF_LN("- %s", buffer);
			if(search == NULL){ // If no 9 is found
				PRINTF_LN("- NBIOT mode not found");
				if(StringStartsWith(buffer, "+CEREG: 1,\"")){ // Check for CEREG: 1,"
					status = BG96_OK; // If that is found: OK
				}else{
					status = BG96_ERROR; // If not, all else is lost
				}
			}else{
				status = BG96_OK; // 9 is found
				PRINTF_LN("- NBIOT mode is found");
			}
		}
		tickNow = HW_RTC_GetTimerValue();
	}
	if(status != BG96_OK){
		status = BG96_SendATCommandCheckReply("AT+CEREG?\r\n", "+CEREG: 4,1,\"", 300); // Check one last time
	}
	BG96_SendATCommandCheckReply("AT+CEREG=0\r\n", "OK", 300);
		
	return status;
}	

// --- GNSS AT commands ---
BG96_Status_t BG96_GNSS_Enable(uint8_t mode, uint8_t fixTime,  uint8_t accuracy, uint16_t fixCount, uint16_t fixDelay){
	char buffer[50]; 
	sprintf(buffer, "AT+QGPS=%d,%d,%d,%d,%d\r\n", mode, fixTime, accuracy, fixCount, fixDelay);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "OK", 300);
	return status;
}

BG96_Status_t BG96_GNSS_Disable( void ){
	BG96_Status_t status = BG96_SendATCommandCheckReply("AT+QGPSEND\r\n", "OK", 300);
	return status;
}
// Not tested
BG96_Status_t BG96_GNSS_GetLocation(float * latitudePointer, float * longitudePointer, uint8_t timeout){
  char response[BG96_REPLY_SIZE]; 
  memset(response, 0 , BG96_REPLY_SIZE);
	BG96_Status_t status;
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	
	while(status != BG96_OK && !StringStartsWith(response, "+QGPSLOC: ") && ( ( tickNow - tickstart ) ) < timeout){
		status = BG96_SendATCommandGetReply("AT+QGPSLOC?\r\n", response, 300);
		tickNow = HW_RTC_GetTimerValue();
	}
  
  char * startLocation = strstr(response, "+QGPSLOC: ");
  
  char tempBuffer[10];
  strncpy(tempBuffer, startLocation+19, 2);
  tempBuffer[2] = '\0';
  int latitudeDegrees = atoi(tempBuffer);
  
  strncpy(tempBuffer, startLocation+21,8);
  tempBuffer[8] = '\0';
  float latitudeMinutes = atof(tempBuffer);

  int latitudeSign;
  if(*(startLocation + 28) == 'N'){
	  latitudeSign = 1;
  }else{
	  latitudeSign = -1;
  }
  
  strncpy(tempBuffer, startLocation+30,3);
  tempBuffer[3] = '\0';
  int longitudeDegrees = atoi(tempBuffer);
  
  strncpy(tempBuffer, startLocation+33,8);
  tempBuffer[6] = '\0';
  float longitudeMinutes = atof(tempBuffer);
  
  int longitudeSign;
  if(*(startLocation + 40) == 'E'){
	  longitudeSign = 1;
  }else{
	  longitudeSign = -1;
  }
 
  *latitudePointer = (latitudeDegrees + latitudeMinutes/60)*latitudeSign;
  *longitudePointer = (longitudeDegrees + longitudeMinutes/60)*longitudeSign;  
	
	return BG96_OK;
} 

// --- TCP/UDP/IP commands ---

void BG96_SelectContextID(uint8_t context){
	_contextID = context;
}

void BG96_SelectConnectID(uint8_t connect){
	_connectID = connect;
}

BG96_Status_t bg96_GetPacketCounters(uint16_t * downlink, uint16_t * uplink){
	BG96_Status_t status  =  BG96_ERROR; 
	const uint8_t bufsize = 32;
  char buffer[bufsize];
  memset(buffer, '\0', bufsize);
  BG96_SendATCommandGetReply("AT+QGDCNT?\r\n", buffer, 300);
  status = BG96_SendATCommandCheckReply("", "OK", 300);
	if(status != BG96_OK)	
		return status;
	
  char * nextPointer;
  nextPointer = strtok(buffer," ,\"");
	
  uint8_t index = 0;
	
  while (nextPointer != NULL){
		if(index == 2){
			*uplink = atoi(nextPointer);
		}
		if(index == 2){
			*downlink = atoi(nextPointer);
		}
		nextPointer = strtok(NULL, ",\"");
		index ++;
  }
	return BG96_OK;
}

BG96_Status_t BG96_ResetPacketCounters( void ){
	return BG96_SendATCommandCheckReply("AT+QGDCNT=0\r\n", "OK", 300);
}	

BG96_Status_t BG96_ConfigureContext( void ){
	char buffer[60]; 
	sprintf(buffer, "AT+QICSGP=%d,1,\"m2minternet.proximus.be\",\"\",\"\",1\r\n", _contextID);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}	

BG96_Status_t BG96_ConfigureContextAPN(uint8_t contextType, char * apn, char * username, char * password, uint8_t authentication ){
	char buffer[60]; 
	sprintf(buffer, "AT+QICSGP=%d, %d, \"%s\", \"%s\", \"%s\", %d\r\n", _contextID, contextType, apn, username, password, authentication);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}	

BG96_Status_t BG96_ActivateContext( void ){
	char buffer[20]; 
	sprintf(buffer, "AT+QIACT=%d\r\n", _contextID);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "OK", 1000);
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(status != BG96_OK && ( ( tickNow - tickstart ) ) < 30000){
		HAL_Delay(100+rand()%200);
		status = BG96_SendATCommandCheckReply(buffer, "OK", 1000);
		tickNow = HW_RTC_GetTimerValue();
	}
	return status;
}

BG96_Status_t BG96_DeactivateContext( void ){
	char buffer[20]; 
	sprintf(buffer, "AT+QIDEACT=%d\r\n", _contextID);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

bool BG96_CheckIfContextActivated( void ){
	char buffer[100];
	BG96_Status_t status = BG96_SendATCommandGetReply("AT+QIACT?\r\n", buffer, 300);
	char bufferCheck[30]; 
	sprintf(bufferCheck, "+QIACT: %d,1", _contextID);
	if(StringStartsWith(buffer, bufferCheck)){
		return true;
	}
	return false;
}

BG96_Status_t BG96_UDP_Start(char * ipaddress, uint32_t port){ // Start as client
	char buffer[70]; 
	sprintf(buffer, "AT+QIOPEN=%d,%d,\"UDP\",\"%s\",%d\r\n", _contextID, _connectID, ipaddress, port);
	BG96_Status_t status =  BG96_SendATCommandCheckReply(buffer, "OK", 300);
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(status != BG96_OK && ( ( tickNow - tickstart ) ) < 30000){
		HAL_Delay(100+rand()%200);
		status =  BG96_SendATCommandCheckReply(buffer, "OK", 300);
		tickNow = HW_RTC_GetTimerValue();
	}
	if(status != BG96_OK)
		return status;
	BG96_SendATCommandGetReply("", buffer, 300);
	char bufferCheck[40]; 
	sprintf(bufferCheck, "+QIOPEN: %d,0", _connectID);
	if(StringStartsWith(buffer, bufferCheck)){
		PRINTF_LN("buffer check ok");
		
		return BG96_OK;
	}
	bufferCheck[11] = '\0';
	if(StringStartsWith(buffer, bufferCheck)){
		PRINTF_LN("buffer check !ok");
		return BG96_ERROR;
	}
	PRINTF_LN("other");
	PRINTF_LN("%s",buffer);
	return BG96_ERROR;
}

BG96_Status_t BG96_UDP_Stop( void ){ 
	char buffer[60]; 
	sprintf(buffer, "AT+QICLOSE=%d\r\n", _connectID);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_UDP_StartService(char * ipaddress, uint32_t port){ // Start service
	char buffer[60]; 
	sprintf(buffer, "AT+QIOPEN=%d,%d,\"UDP SERVICE\",\"%s\",%d,0,0\r\n", _contextID, _connectID, ipaddress, port);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_UDP_GetStatus(char * replyBuffer){ // Start service
	char buffer[30]; 
	sprintf(buffer, "AT+QISTATE=1,%d\r\n", _connectID);
	return BG96_SendATCommandGetReply(buffer, replyBuffer, 300);
}

BG96_Status_t BG96_UDP_SendData( char * data, uint32_t timeout){
	char buffer[60]; 
	BG96_Status_t status = BG96_ERROR;
	sprintf(buffer, "AT+QISEND=%d\r\n", _connectID);
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	
	while(status != BG96_OK && ( ( tickNow - tickstart ) ) < timeout){
		HAL_Delay(500+rand()%500);
		status = BG96_SendATCommandCheckReply(buffer, ">", 300);
		tickNow = HW_RTC_GetTimerValue();
	}
	if(status != BG96_OK)
		return status;
	BG96_SendATCommand(data);
	status = BG96_SendATCommandCheckReply("\x1A", "", 1000);
	return BG96_SendATCommandCheckReply("","SEND OK",300);
}

BG96_Status_t BG96_UDP_SendDataTo(char * ipaddress, uint32_t port, char * data){
	char buffer[60]; 
	sprintf(buffer, "AT+QISEND=%d,%d,\"%s\",%d\r\n", _connectID, strlen(data), ipaddress, port);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, ">", 300);
	if(status != BG96_OK)
		return status;
	return BG96_SendATCommandCheckReply(data, "SEND OK", 1000);
}

BG96_Status_t BG96_SetDNSServer(char * primary, char * secondary){
	char buffer[60]; 
	sprintf(buffer, "AT+QISEND=%d,\"%s\",\"%s\"\r\n", _contextID, primary, secondary);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

// --- HTTP Commands ---
BG96_Status_t BG96_HTTP_SetContextID(uint8_t id){
	char buffer[60]; 
	_contextID = id;
	sprintf(buffer, "AT+QHTTPCFG=\"contextid\",%d\r\n", id);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_HTTP_SetResponseHeaderID(uint8_t id){
	char buffer[60]; 
	_responseHeaderID = id;
	sprintf(buffer, "AT+QHTTPCFG=\"responseheader\",%d\r\n", id);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_HTTP_SetRequestHeaderID(uint8_t id){
	char buffer[60]; 
	_requestHeaderID = id;
	sprintf(buffer, "AT+QHTTPCFG=\"requestheader\",%d\r\n", id);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_HTTP_SetSSLContextID(uint8_t id){
	char buffer[60]; 
	_SSLContextID = id;
	sprintf(buffer, "AT+QHTTPCFG=\"sslctxid\",%d\r\n", id);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_HTTP_SetContentType(uint8_t id){
	char buffer[60]; 
	_contentType = id;
	sprintf(buffer, "AT+QHTTPCFG=\"contenttype\",%d\r\n", id);
	return BG96_SendATCommandCheckReply(buffer, "OK", 300);
}

BG96_Status_t BG96_HTTP_SetURL(char * url){
	char buffer[60]; 
	sprintf(buffer, "AT+QHTTPURL=%d\r\n", strlen(url));
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "CONNECT", 300);
	if(status != BG96_OK)
		return status;
	return BG96_SendATCommandCheckReply(url, "OK", 300);
}

BG96_Status_t BG96_HTTP_Get(uint16_t timeout){
	char buffer[60]; 
	sprintf(buffer, "AT+QHTTPURL=%d\r\n", timeout);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "OK", 300);
	if(status != BG96_OK)
		return status;
	return BG96_SendATCommandCheckReply("", "+QHTTPGET:", 2000);
}

BG96_Status_t BG96_HTTP_ReadResponse(uint16_t wait_time, char * buffer, uint16_t timeout){ //TODO: Tested (this is not tested)
	sprintf(buffer, "AT+QHTTPREAD=%d\r\n", wait_time);
	BG96_Status_t status = BG96_SendATCommandCheckReply(buffer, "CONNECT", 300);
	if(status != BG96_OK)
		return status;
	
	// Buffer needs to be initialized as \0?
	uint16_t i = 0;
	char responseBuffer[BG96_HTTP_RESPONSE_SIZE];
	memset(responseBuffer, '\0', BG96_HTTP_RESPONSE_SIZE);
	uint32_t tickstart = HW_RTC_GetTimerValue();
	uint32_t tickNow = HW_RTC_GetTimerValue();
	while(!StringStartsWith(responseBuffer, "+QHTTPREAD") && ( ( tickNow - tickstart ) ) < timeout){
		status = BG96_SendATCommandGetReply("", responseBuffer+i, 300);
		if(status != BG96_OK)
			return status;
		i = strlen(responseBuffer);
		tickNow = HW_RTC_GetTimerValue();
	}
	if(( tickNow - tickstart )  >= timeout){
		return BG96_TIMEOUT;
	}
	return BG96_OK;
}
