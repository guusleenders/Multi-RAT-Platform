#include "ltc2942.h"
#include "vcom.h"

const float LTC2942_CHARGE_LSB = 0.085;
const uint8_t prescalarTable[8] = {1, 2, 4, 8, 16, 32, 64, 128};
uint8_t prescalar = 0;

static uint32_t I2C_EXPBD_Timeout = LTC2942_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef LTC2942_LRWAN_Handle;
static I2C_HandleTypeDef LTC2942_NBIOT_Handle;


void LTC2942_Init( LTC2942_SENSOR sensor ){
	I2C_Init(sensor);
}

uint8_t LTC2942_GetStatus(LTC2942_SENSOR sensor) {
		uint8_t data = 0;
    LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_STATUS_REG, 1, &data);
	  return data; 
}

uint8_t LTC2942_GetControl(LTC2942_SENSOR sensor) {
		uint8_t data = 0;
    LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, 1, &data);
		return data; 
}

/*
// Only for LTC2941
void LTC2942_SetBatteryAlert(LTC2942_SENSOR sensor, LTC2942_VBAT_ALERT voltage) {
    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_VBAT_ALERT_MASK, MEASURE_VBAT_ALERT_POS, voltage);
}
*/

void LTC2942_SetADCSingleVoltage(LTC2942_SENSOR sensor) {
    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_ADC_MODE_MASK, LTC2942_ADC_MODE_POS, LTC2942_ADC_VOLTAGE);
}

void LTC2942_SetADCSingleTemperature(LTC2942_SENSOR sensor) {
    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_ADC_MODE_MASK, LTC2942_ADC_MODE_POS, LTC2942_ADC_TEMPERATURE);
}

void LTC2942_SetPrescaler(LTC2942_SENSOR sensor, LTC2942_PRESCALAR prescale) {
    prescalar = prescalarTable[prescale];

    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_PRESCALAR_MASK, LTC2942_PRESCALAR_POS, prescale);
}

void LTC2942_SetAlertConfig(LTC2942_SENSOR sensor, LTC2942_ALERT_CONF config) {
    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_ALERT_MODE_MASK, LTC2942_ALERT_MODE_POS, config);
}

void LTC2942_SetShutdown(LTC2942_SENSOR sensor, bool enable) {
    LTC2942_UpdateReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_CONTROL_REG, LTC2942_SHUTDOWN_MASK, LTC2942_SHUTDOWN_POS, enable);
}

uint16_t LTC2942_GetAccumulatedCharge(LTC2942_SENSOR sensor){
	uint8_t data[2] = {0,0};
	LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_ACCUM_CHARGE_MSB_REG, 1, &data[0]);
	LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_ACCUM_CHARGE_LSB_REG, 1, &data[1]);
	uint16_t result =  (data[0] << 8) | data[1];
	return result;
}

void LTC2942_SetAccumulatedCharge(LTC2942_SENSOR sensor, uint16_t charge){
	uint8_t data[2] = {0,0}; // [0]: msb; [1]: lsb
	data[0] = charge>>8;
	data[1] = charge & 0x00ff;
	LTC2942_WriteReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_ACCUM_CHARGE_MSB_REG, 1, &data[0]);
	LTC2942_WriteReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_ACCUM_CHARGE_LSB_REG, 1, &data[1]);
}

float LTC2942_GetVoltage(LTC2942_SENSOR sensor){
	LTC2942_SetADCSingleVoltage(sensor);
	HAL_Delay(11); // >10ms for adc
	uint8_t data[2] = {0,0};
	LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_VOLTAGE_MSB_REG, 1, &data[0]);
	LTC2942_ReadReg(sensor, LINEAR_LTC2942_ADDRESS, LTC2942_VOLTAGE_LSB_REG, 1, &data[1]);
	uint16_t result =  (data[0] << 8) | data[1];
	float voltage = 6.0*result/65535.0;
			
	return voltage;
}

float LTC2942_GetCoulomb( LTC2942_SENSOR sensor ) {
	return LTC2942_GetmAh(sensor) *3.6f;
}

float LTC2942_GetJoule( LTC2942_SENSOR sensor ) {
	float voltage = LTC2942_GetVoltage(sensor);
	return LTC2942_GetCoulomb(sensor) * voltage; // E = C * U
}

float LTC2942_GetmAh( LTC2942_SENSOR sensor ) {
    uint16_t data = 0;
    float coulomb = 0;

    data = LTC2942_GetAccumulatedCharge(sensor);
		if(sensor == LTC2942_LRWAN){
			coulomb = (float)(data * LTC2942_CHARGE_LSB * prescalar * 0.05) / (LTC2942_LRWAN_RSENSE * 128);
		}else if(sensor == LTC2942_NBIOT){
			coulomb = (float)(data * LTC2942_CHARGE_LSB * prescalar * 0.05) / (LTC2942_NBIOT_RSENSE * 128);
		}
    return coulomb;
}

LTC2942_Error_et LTC2942_ReadReg(LTC2942_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data ){
	uint8_t result = 0;
	if(sensor == LTC2942_LRWAN){
		result = I2C_ReadData(&LTC2942_LRWAN_Handle, addr<<1, RegAddr, Data, NumByteToRead );
	}else if(sensor == LTC2942_NBIOT){
		result = I2C_ReadData(&LTC2942_NBIOT_Handle, addr<<1, RegAddr, Data, NumByteToRead );
	}
  if ( result )
    return LTC2942_ERROR;
  else
    return LTC2942_OK;
}

LTC2942_Error_et LTC2942_WriteReg(LTC2942_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data ){

  if ( NumByteToWrite > 1 ) RegAddr |= 0x80;

	uint8_t result = 0;
	if(sensor == LTC2942_LRWAN){
		result = I2C_WriteData(&LTC2942_LRWAN_Handle,  addr<<1, RegAddr, Data, NumByteToWrite );
	}else if(sensor == LTC2942_NBIOT){
		result = I2C_WriteData(&LTC2942_NBIOT_Handle, addr<<1, RegAddr, Data, NumByteToWrite );
	}
	
  if ( result )
    return LTC2942_ERROR;
  else
    return LTC2942_OK;
}

LTC2942_Error_et LTC2942_UpdateReg(LTC2942_SENSOR sensor, uint8_t addr, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val) {
   uint8_t tmp;
		
	 LTC2942_ReadReg(sensor, addr, reg, 1, &tmp);
	 
   tmp &= mask;
   tmp |= (val << shift) & ~mask;

   return LTC2942_WriteReg(sensor, addr, reg, 1, &tmp);
	
}

/******************************* I2C Routines *********************************/


static uint8_t I2C_Init( LTC2942_SENSOR sensorType ){
	
	if(sensorType == LTC2942_LRWAN){
		if(HAL_I2C_GetState( &LTC2942_LRWAN_Handle) == HAL_I2C_STATE_RESET ){

			/* I2C_EXPBD peripheral configuration */

			LTC2942_LRWAN_Handle.Init.Timing 				 = LTC2942_TIMING_100KHZ;    /* 400KHz */
			LTC2942_LRWAN_Handle.Init.OwnAddress1    = 0x33;
			LTC2942_LRWAN_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			
			LTC2942_LRWAN_Handle.Instance            = LTC2942_LRWAN_INSTANCE;

			/* Init the I2C */
			I2C_MspInit(LTC2942_LRWAN);
			HAL_I2C_Init( &LTC2942_LRWAN_Handle );
		}

		if( HAL_I2C_GetState( &LTC2942_LRWAN_Handle) == HAL_I2C_STATE_READY ){
			return 0;
		}
		else{
			return 1;
		}
	}else if(sensorType == LTC2942_NBIOT){
		if(HAL_I2C_GetState( &LTC2942_NBIOT_Handle) == HAL_I2C_STATE_RESET ){

			/* I2C_EXPBD peripheral configuration */

			LTC2942_NBIOT_Handle.Init.Timing 				 = LTC2942_TIMING_100KHZ;    /* 400KHz */
			LTC2942_NBIOT_Handle.Init.OwnAddress1    = 0x33;
			LTC2942_NBIOT_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			
			LTC2942_NBIOT_Handle.Instance            = LTC2942_NBIOT_INSTANCE;

			/* Init the I2C */
			I2C_MspInit(LTC2942_NBIOT);
			HAL_I2C_Init( &LTC2942_NBIOT_Handle );
		}

		if( HAL_I2C_GetState( &LTC2942_NBIOT_Handle) == HAL_I2C_STATE_READY ){
			return 0;
		}
		else{
			return 1;
		}
	}else{
		return 0;
	}
	
}

static uint8_t I2C_WriteData(I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( handler, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    I2C_Error(handler, Addr );
    return 1;
  }
  else{
    return 0;
  }
}

static uint8_t I2C_ReadData( I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( handler, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    I2C_Error(handler, Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}

static void I2C_Error( I2C_HandleTypeDef* handler, uint8_t Addr ){
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit( handler );

  /* Re-Initiaize the I2C comunication bus */
	if(handler->Instance == LTC2942_LRWAN_INSTANCE){
		I2C_Init(LTC2942_LRWAN);
	}else if(handler->Instance == LTC2942_NBIOT_INSTANCE){
		I2C_Init(LTC2942_NBIOT);
	}
  
}


static void I2C_MspInit(LTC2942_SENSOR sensorType){
  GPIO_InitTypeDef  GPIO_InitStruct;

	if(sensorType == LTC2942_LRWAN){
		LTC2942_LRWAN_SCL_SDA_GPIO_CLK_ENABLE(); // Enable I2C GPIO clocks; both are on port b

		// I2C_EXPBD SCL and SDA pins configuration
		GPIO_InitStruct.Pin        = LTC2942_LRWAN_SCL_PIN | LTC2942_LRWAN_SDA_PIN;
		GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed 		 = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pull       = GPIO_NOPULL;
		GPIO_InitStruct.Alternate  = LTC2942_LRWAN_SCL_SDA_AF;

		HAL_GPIO_Init( LTC2942_LRWAN_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

		LTC2942_LRWAN_CLK_ENABLE();		// Enable the I2C_EXPBD peripheral clock
		LTC2942_LRWAN_FORCE_RESET(); 	//Force the I2C peripheral clock reset
		LTC2942_LRWAN_RELEASE_RESET(); // Release the I2C peripheral clock reset

		/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(LTC2942_LRWAN_EV_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(LTC2942_LRWAN_EV_IRQn);
		
	}else if(sensorType == LTC2942_NBIOT){
		LTC2942_NBIOT_SCL_SDA_GPIO_CLK_ENABLE(); // Enable I2C GPIO clocks; both are on port b

		// I2C_EXPBD SCL and SDA pins configuration
		GPIO_InitStruct.Pin        = LTC2942_NBIOT_SCL_PIN | LTC2942_NBIOT_SDA_PIN;
		GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed 		 = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pull       = GPIO_NOPULL;
		GPIO_InitStruct.Alternate  = LTC2942_NBIOT_SCL_SDA_AF;

		HAL_GPIO_Init( LTC2942_NBIOT_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

		LTC2942_NBIOT_CLK_ENABLE();		// Enable the I2C_EXPBD peripheral clock
		LTC2942_NBIOT_FORCE_RESET(); 	//Force the I2C peripheral clock reset
		LTC2942_NBIOT_RELEASE_RESET(); // Release the I2C peripheral clock reset

		/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(LTC2942_NBIOT_EV_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(LTC2942_NBIOT_EV_IRQn);
	}
	


}
