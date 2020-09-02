#include "ltc2941.h"

const float LTC2941_CHARGE_LSB = 0.085;
const uint8_t prescalarTable[8] = {1, 2, 4, 8, 16, 32, 64, 128};
uint8_t prescalar = 0;


void LTC2941_Init( void ){
	I2C_Init();
}

uint8_t LTC2941_GetStatus(void) {
		uint8_t data = 0;
    LTC2941_ReadReg(LINEAR_LTC2941_ADDRESS, LTC2941_STATUS_REG, 1, &data);
	  return data; 
}

uint8_t LTC2941_GetControl(void) {
		uint8_t data = 0;
    LTC2941_ReadReg(LINEAR_LTC2941_ADDRESS, LTC2941_CONTROL_REG, 1, &data);
		return data; 
}

void LTC2941_SetBatteryAlert(LTC2941_VBAT_ALERT voltage) {
    LTC2941_UpdateReg(LINEAR_LTC2941_ADDRESS, LTC2941_CONTROL_REG, LTC2941_VBAT_ALERT_MASK, MEASURE_VBAT_ALERT_POS, voltage);
}

void LTC2941_SetPrescaler(LTC2941_PRESCALAR prescale) {
    prescalar = prescalarTable[prescale];

    LTC2941_UpdateReg(LINEAR_LTC2941_ADDRESS, LTC2941_CONTROL_REG, LTC2941_PRESCALAR_MASK, MEASURE_PRESCALAR_POS, prescale);
}

void LTC2941_SetAlertConfig(LTC2941_ALERT_CONF config) {
    LTC2941_UpdateReg(LINEAR_LTC2941_ADDRESS, LTC2941_CONTROL_REG, LTC2941_ALERT_MODE_MASK, MEASURE_ALERT_MODE_POS, config);
}

void LTC2941_SetShutdown(bool enable) {
    LTC2941_UpdateReg(LINEAR_LTC2941_ADDRESS, LTC2941_CONTROL_REG, LTC2941_SHUTDOWN_MASK, MEASURE_SHUTDOWN_POS, enable);
}

uint16_t LTC2941_GetAccumulatedCharge(){
	uint8_t data[2] = {0,0};
	LTC2941_ReadReg(LINEAR_LTC2941_ADDRESS, LTC2941_ACCUM_CHARGE_MSB_REG, 1, &data[0]);
	LTC2941_ReadReg(LINEAR_LTC2941_ADDRESS, LTC2941_ACCUM_CHARGE_LSB_REG, 1, &data[1]);
	uint16_t result =  (data[0] << 8) | data[1];
	return result;
}

void LTC2941_SetAccumulatedCharge(uint16_t charge){
	uint8_t data[2] = {0,0}; // [0]: msb; [1]: lsb
	data[0] = charge>>8;
	data[1] = charge & 0x00ff;
	LTC2941_WriteReg(LINEAR_LTC2941_ADDRESS, LTC2941_ACCUM_CHARGE_MSB_REG, 1, &data[0]);
	LTC2941_WriteReg(LINEAR_LTC2941_ADDRESS, LTC2941_ACCUM_CHARGE_LSB_REG, 1, &data[1]);
}

float LTC2941_GetCoulomb( void ) {
	return LTC2941_GetmAh() *3.6f;
}

float LTC2941_GetJoule( void ) {
	return LTC2941_GetCoulomb() *3.3f; // E = C * U
}

float LTC2941_GetmAh( void ) {
    uint16_t data = 0;
    float coulomb = 0;

    data = LTC2941_GetAccumulatedCharge();

    coulomb = (float)(data * LTC2941_CHARGE_LSB * prescalar * 0.05) / (RSENSE * 128);

    return coulomb;
}

LTC2941_Error_et LTC2941_ReadReg( uint8_t addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data ){

  if ( I2C_ReadData( addr<<1, RegAddr, Data, NumByteToRead ) )
    return LTC2941_ERROR;
  else
    return LTC2941_OK;
}

LTC2941_Error_et LTC2941_WriteReg( uint8_t addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data ){

  if ( NumByteToWrite > 1 ) RegAddr |= 0x80;

  if ( I2C_WriteData( addr<<1, RegAddr, Data, NumByteToWrite ) )
    return LTC2941_ERROR;
  else
    return LTC2941_OK;
}

LTC2941_Error_et LTC2941_UpdateReg(uint8_t addr, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val) {
    uint8_t tmp;

    LTC2941_ReadReg(addr, reg, 1, &tmp);

    tmp &= mask;
    tmp |= (val << shift) & ~mask;

    return LTC2941_WriteReg(addr, reg, 1, &tmp);
	
}

/******************************* I2C Routines *********************************/

static uint32_t I2C_EXPBD_Timeout =
  NUCLEO_I2C_EXPBD_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef I2C_EXPBD_Handle;

static uint8_t I2C_Init( void ){
  if(HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_RESET ){

    /* I2C_EXPBD peripheral configuration */

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
    I2C_EXPBD_Handle.Init.ClockSpeed = NUCLEO_I2C_EXPBD_SPEED;
    I2C_EXPBD_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif

#if (defined (USE_STM32L0XX_NUCLEO)|| (defined (USE_B_L072Z_LRWAN1)))
    I2C_EXPBD_Handle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    I2C_EXPBD_Handle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif

    I2C_EXPBD_Handle.Init.OwnAddress1    = 0x33;
    I2C_EXPBD_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_EXPBD_Handle.Instance            = NUCLEO_I2C_EXPBD;

    /* Init the I2C */
    I2C_MspInit();
    HAL_I2C_Init( &I2C_EXPBD_Handle );
  }

  if( HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_READY ){
    return 0;
  }
  else{
    return 1;
  }
}

static uint8_t I2C_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    I2C_Error( Addr );
    return 1;
  }
  else{
    return 0;
  }
}

static uint8_t I2C_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    I2C_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}

static void I2C_Error( uint8_t Addr ){
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit( &I2C_EXPBD_Handle );

  /* Re-Initiaize the I2C comunication bus */
  I2C_Init();
}


static void I2C_MspInit( void ){
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = NUCLEO_I2C_EXPBD_SCL_PIN | NUCLEO_I2C_EXPBD_SDA_PIN;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO))|| (defined (USE_B_L072Z_LRWAN1)))
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = NUCLEO_I2C_EXPBD_SCL_SDA_AF;

  HAL_GPIO_Init( NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
  NUCLEO_I2C_EXPBD_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  NUCLEO_I2C_EXPBD_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  NUCLEO_I2C_EXPBD_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_EV_IRQn);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_ER_IRQn);
#endif

}
