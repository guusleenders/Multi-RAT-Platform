#ifndef __LTC2941
#define __LTC2941

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUCLEO_I2C_EXPBD_TIMING_100KHZ       0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define NUCLEO_I2C_EXPBD_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */

#define NUCLEO_I2C_EXPBD                            I2C1
#define NUCLEO_I2C_EXPBD_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define NUCLEO_I2C_EXPBD_SCL_SDA_AF                 GPIO_AF4_I2C1
#define NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_PORT          GPIOB
#define NUCLEO_I2C_EXPBD_SCL_PIN                    GPIO_PIN_8
#define NUCLEO_I2C_EXPBD_SDA_PIN                    GPIO_PIN_9

#define NUCLEO_I2C_EXPBD_FORCE_RESET()              __I2C1_FORCE_RESET()
#define NUCLEO_I2C_EXPBD_RELEASE_RESET()            __I2C1_RELEASE_RESET()

#define NUCLEO_I2C_EXPBD_TIMEOUT_MAX    0x2000 /*<! The value of the maximal timeout for BUS waiting loops */
#define NUCLEO_I2C_EXPBD_EV_IRQn                    I2C1_IRQn

#define LINEAR_LTC2941_ADDRESS                  0x64

#define LTC2941_STATUS_REG                      0x00
#define LTC2941_CONTROL_REG                     0x01
#define LTC2941_ACCUM_CHARGE_MSB_REG            0x02
#define LTC2941_ACCUM_CHARGE_LSB_REG            0x03
#define LTC2941_CHARGE_THRESH_HIGH_MSB_REG      0x04
#define LTC2941_CHARGE_THRESH_HIGH_LSB_REG      0x05
#define LTC2941_CHARGE_THRESH_LOW_MSB_REG       0x06
#define LTC2941_CHARGE_THRESH_LOW_LSB_REG       0x07

#define LTC2941_VBAT_ALERT_MASK     0x3F
#define LTC2941_PRESCALAR_MASK      0xC7
#define LTC2941_ALERT_MODE_MASK     0xF9
#define LTC2941_SHUTDOWN_MASK       0xFE

#define MEASURE_VBAT_ALERT_POS		6
#define MEASURE_PRESCALAR_POS		3
#define MEASURE_ALERT_MODE_POS		1
#define MEASURE_SHUTDOWN_POS		0

#define LTC2941_BATTERY_MAX         5570 // mAh

#define RSENSE					0.101189 // Calibrated; nominal value: 0.1
typedef enum {
    VBAT_ALERT_OFF = 0,
    VBAT_2_8_V = 1,
    VBAT_2_9_V = 2,
    VBAT_3_0_V = 3,
} LTC2941_VBAT_ALERT;

typedef enum {
    PRESCALAR_M_1 = 0,
    PRESCALAR_M_2 = 1,
    PRESCALAR_M_4 = 2,
    PRESCALAR_M_8 = 3,
    PRESCALAR_M_16 = 4,
    PRESCALAR_M_32 = 5,
    PRESCALAR_M_64 = 6,
    PRESCALAR_M_128 = 7,
} LTC2941_PRESCALAR;

typedef enum {
    ALERT_DISABLED = 0,
    CHARGE_COMPLETE = 1,
    ALERT_MODE = 2,
} LTC2941_ALERT_CONF;

typedef enum {
		LTC2941_OK = (uint8_t)0, 
		LTC2941_ERROR = !LTC2941_OK,
} LTC2941_Error_et;

void LTC2941_Init( void );
uint8_t LTC2941_GetStatus( void );
uint8_t LTC2941_GetControl(void);
void LTC2941_SetBatteryAlert(LTC2941_VBAT_ALERT voltage);
void LTC2941_SetPrescaler(LTC2941_PRESCALAR prescale);
void LTC2941_SetAlertConfig(LTC2941_ALERT_CONF config);
void LTC2941_SetShutdown(bool enable);

uint16_t LTC2941_GetAccumulatedCharge( void );
void LTC2941_SetAccumulatedCharge(uint16_t charge);

float LTC2941_GetmAh( void );
float LTC2941_GetCoulomb( void );
float LTC2941_GetJoule( void );

LTC2941_Error_et LTC2941_ReadReg( uint8_t addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
LTC2941_Error_et LTC2941_WriteReg( uint8_t addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );
LTC2941_Error_et LTC2941_UpdateReg(uint8_t addr, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val);


static void I2C_MspInit( void );
static void I2C_Error( uint8_t Addr );
static uint8_t I2C_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_Init( void );

#ifdef __cplusplus
}
#endif

#endif /* __LTC2941 */
