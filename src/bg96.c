#include "bg96.h"

static UART_HandleTypeDef BG96_UARTHandle;

void BG96_Init( void ){
	// --- Init Serial stuff ---
	__USART1_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Pin = BG96_RX_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = BG96_SERIAL_AF;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BG96_RX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BG96_TX_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(BG96_TX_PORT, &GPIO_InitStructure);
	
			
	BG96_UARTHandle.Instance  = USART1;
  BG96_UARTHandle.Init.BaudRate = 9600;
  BG96_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
  BG96_UARTHandle.Init.StopBits = UART_STOPBITS_1;
  BG96_UARTHandle.Init.Parity = UART_PARITY_NONE;
  BG96_UARTHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  BG96_UARTHandle.Init.Mode = UART_MODE_TX_RX;
	
	HAL_UART_Init(&BG96_UARTHandle);
	
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

void BG96_PowerOn( void ){
	HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(500);
  HAL_GPIO_WritePin(BG96_POWERKEY_PORT, BG96_POWERKEY_PIN, GPIO_PIN_RESET); 
  
  HAL_Delay(10);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_SET); 
  HAL_Delay(500);
  HAL_GPIO_WritePin(BG96_RESETKEY_PORT, BG96_RESETKEY_PIN, GPIO_PIN_RESET); 
	
	char buffer[5] = {'A','T','E','1','\r'};
	uint8_t receiveBuffer[50];
	BG96_SendATCommand(buffer);
	//HAL_UART_Transmit(&BG96_UARTHandle, buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_UART_Receive(&BG96_UARTHandle, receiveBuffer, sizeof(receiveBuffer), HAL_MAX_DELAY);
	
	PRINTF("%s", receiveBuffer);
}

void BG96_SendATCommand( char *buffer){
	HAL_UART_Transmit(&BG96_UARTHandle, (uint8_t *) buffer, sizeof(buffer), HAL_MAX_DELAY);
}
