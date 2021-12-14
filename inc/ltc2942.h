#ifndef __LTC2942
#define __LTC2942

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LTC2942_TIMING_100KHZ       0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define LTC2942_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */
#define LTC2942_TIMEOUT_MAX    0x2000 /*<! The value of the maximal timeout for BUS waiting loops */

#define LTC2942_LRWAN_INSTANCE                   I2C1
#define LTC2942_LRWAN_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define LTC2942_LRWAN_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define LTC2942_LRWAN_SCL_SDA_AF                 GPIO_AF4_I2C1
#define LTC2942_LRWAN_SCL_SDA_GPIO_PORT          GPIOB
#define LTC2942_LRWAN_SCL_PIN                    GPIO_PIN_8
#define LTC2942_LRWAN_SDA_PIN                    GPIO_PIN_9

#define LTC2942_LRWAN_FORCE_RESET()              __I2C1_FORCE_RESET()
#define LTC2942_LRWAN_RELEASE_RESET()            __I2C1_RELEASE_RESET()

#define LTC2942_LRWAN_EV_IRQn                    I2C1_IRQn

#define LTC2942_NBIOT_INSTANCE                   I2C2
#define LTC2942_NBIOT_CLK_ENABLE()               __I2C2_CLK_ENABLE()
#define LTC2942_NBIOT_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define LTC2942_NBIOT_SCL_SDA_AF                 GPIO_AF5_I2C2
#define LTC2942_NBIOT_SCL_SDA_GPIO_PORT          GPIOB
#define LTC2942_NBIOT_SCL_PIN                    GPIO_PIN_13
#define LTC2942_NBIOT_SDA_PIN                    GPIO_PIN_14

#define LTC2942_NBIOT_FORCE_RESET()              __I2C2_FORCE_RESET()
#define LTC2942_NBIOT_RELEASE_RESET()            __I2C2_RELEASE_RESET()

#define LTC2942_NBIOT_EV_IRQn                    I2C2_IRQn

#define LINEAR_LTC2942_ADDRESS                  0x64

#define LTC2942_STATUS_REG                      0x00
#define LTC2942_CONTROL_REG                     0x01
#define LTC2942_ACCUM_CHARGE_MSB_REG            0x02
#define LTC2942_ACCUM_CHARGE_LSB_REG            0x03
#define LTC2942_CHARGE_THRESH_HIGH_MSB_REG      0x04
#define LTC2942_CHARGE_THRESH_HIGH_LSB_REG      0x05
#define LTC2942_CHARGE_THRESH_LOW_MSB_REG       0x06
#define LTC2942_CHARGE_THRESH_LOW_LSB_REG       0x07
#define LTC2942_VOLTAGE_MSB_REG       					0x08
#define LTC2942_VOLTAGE_LSB_REG					        0x09
#define LTC2942_VOLTAGE_THRESH_HIGH_REG      		0x0A
#define LTC2942_VOLTAGE_THRESH_LOW_REG	        0x0B
#define LTC2942_TEMPERATURE_MSB_REG       			0x0C
#define LTC2942_TEMPERATURE_LSB_REG					    0x0D
#define LTC2942_TEMPERATURE_THRESH_HIGH_REG     0x0E
#define LTC2942_TEMPERATURE_THRESH_low_REG		  0x0F

#define LTC2942_ADC_MODE_MASK     				      0x3F
#define LTC2942_PRESCALAR_MASK     						  0xC7
#define LTC2942_ALERT_MODE_MASK     						0xF9
#define LTC2942_SHUTDOWN_MASK       						0xFE

#define LTC2942_ADC_MODE_POS										6
#define LTC2942_PRESCALAR_POS										3
#define LTC2942_ALERT_MODE_POS									1
#define LTC2942_SHUTDOWN_POS										0

#define LTC2942_ADC_MODE_AUTOMATIC							3
#define LTC2942_ADC_VOLTAGE											2
#define LTC2942_ADC_TEMPERATURE									1
#define LTC2942_ADC_SLEEP												0

#define LTC2942_BATTERY_MAX         						5570 // mAh

#define LTC2942_LRWAN_RSENSE										0.071461 // device 0: 0.0175591 // Calibrated using results (BETA); including /10 scale
#define LTC2942_NBIOT_RSENSE										0.048176 // device 0: 0.01493024 // Calibrated using results (BETA); including /10 scale

typedef enum {
    VBAT_ALERT_OFF = 0,
    VBAT_2_8_V = 1,
    VBAT_2_9_V = 2,
    VBAT_3_0_V = 3,
} LTC2942_VBAT_ALERT;

typedef enum {
    PRESCALAR_M_1 = 0,
    PRESCALAR_M_2 = 1,
    PRESCALAR_M_4 = 2,
    PRESCALAR_M_8 = 3,
    PRESCALAR_M_16 = 4,
    PRESCALAR_M_32 = 5,
    PRESCALAR_M_64 = 6,
    PRESCALAR_M_128 = 7,
} LTC2942_PRESCALAR;

typedef enum {
    ALERT_DISABLED = 0,
    CHARGE_COMPLETE = 1,
    ALERT_MODE = 2,
} LTC2942_ALERT_CONF;

typedef enum {
		LTC2942_OK = (uint8_t)0, 
		LTC2942_ERROR = !LTC2942_OK,
} LTC2942_Error_et;

typedef enum {
    LTC2942_LRWAN = 0,
    LTC2942_NBIOT = 1
} LTC2942_SENSOR;

void LTC2942_Init( LTC2942_SENSOR sensor );
uint8_t LTC2942_GetStatus( LTC2942_SENSOR sensor );
uint8_t LTC2942_GetControl(LTC2942_SENSOR sensor);
// void LTC2942_SetBatteryAlert(LTC2942_SENSOR sensor, LTC2942_VBAT_ALERT voltage); // Only for ltc2941
void LTC2942_SetADCSingleVoltage(LTC2942_SENSOR sensor);
void LTC2942_SetADCSingleTemperature(LTC2942_SENSOR sensor);
void LTC2942_SetPrescaler(LTC2942_SENSOR sensor, LTC2942_PRESCALAR prescale);
void LTC2942_SetAlertConfig(LTC2942_SENSOR sensor, LTC2942_ALERT_CONF config);
void LTC2942_SetShutdown(LTC2942_SENSOR sensor, bool enable);

uint16_t LTC2942_GetAccumulatedCharge( LTC2942_SENSOR sensor );
void LTC2942_SetAccumulatedCharge(LTC2942_SENSOR sensor, uint16_t charge);

float LTC2942_GetVoltage(LTC2942_SENSOR sensor);

float LTC2942_GetmAh( LTC2942_SENSOR sensor );
float LTC2942_GetCoulomb( LTC2942_SENSOR sensor );
float LTC2942_GetJoule( LTC2942_SENSOR sensor );

LTC2942_Error_et LTC2942_ReadReg( LTC2942_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
LTC2942_Error_et LTC2942_WriteReg( LTC2942_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );
LTC2942_Error_et LTC2942_UpdateReg( LTC2942_SENSOR sensor, uint8_t addr, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val);


static void I2C_MspInit(LTC2942_SENSOR sensortype);
static void I2C_Error( I2C_HandleTypeDef* handler, uint8_t Addr );
static uint8_t I2C_ReadData( I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_WriteData( I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_Init( LTC2942_SENSOR sensortype );

#ifdef __cplusplus
}
#endif

#endif /* __LTC2942 */
