
/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: bq27441.h
 *      Created: 2020-03-12
 *       Author: Guus Leenders
 *      Version: 0.1
 *
 *  Description: Adapted from 
 *  https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library/blob/master/src/SparkFunBQ27441.h
 *
 */
#ifndef __BQ27441
#define __BQ27441

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "util.h"

#include "vcom2.h"

#ifdef __cplusplus
extern "C" {
#endif
	
typedef enum {
    BQ27441_LRWAN = 0,
    BQ27441_NBIOT = 1
} BQ27441_SENSOR;

typedef enum {
		BQ27441_OK = (uint8_t)0, 
		BQ27441_ERROR = !BQ27441_OK,
} BQ27441_Error_et;

// Parameters for the current() function, to specify which current to read
typedef enum {
	BQ27441_AVG,  						// Average Current (DEFAULT)
	BQ27441_STBY, 						// Standby Current
	BQ27441_MAX   						// Max Current
} BQ27441_Current_measure_t;

// Parameters for the capacity() function, to specify which capacity to read
typedef enum {
	BQ27441_REMAIN,     			// Remaining Capacity (DEFAULT)
	BQ27441_FULL,       			// Full Capacity
	BQ27441_AVAIL,      			// Available Capacity
	BQ27441_AVAIL_FULL, 			// Full Available Capacity
	BQ27441_REMAIN_F,   			// Remaining Capacity Filtered
	BQ27441_REMAIN_UF,  			// Remaining Capacity Unfiltered
	BQ27441_FULL_F,     			// Full Capacity Filtered
	BQ27441_FULL_UF,    			// Full Capacity Unfiltered
	BQ27441_DESIGN      			// Design Capacity
} BQ27441_Capacity_measure_t;

// Parameters for the soc() function
typedef enum {
	BQ27441_FILTERED,  				// State of Charge Filtered (DEFAULT)
	BQ27441_UNFILTERED 				// State of Charge Unfiltered
} BQ27441_Soc_measure_t;

// Parameters for the soh() function
typedef enum {
	BQ27441_PERCENT,  				// State of Health Percentage (DEFAULT)
	BQ27441_SOH_STAT  				// State of Health Status Bits
} BQ27441_Soh_measure_t;

// Parameters for the temperature() function
typedef enum {
	BQ27441_BATTERY,      		// Battery Temperature (DEFAULT)
	BQ27441_INTERNAL_TEMP 		// Internal IC Temperature
} BQ27441_Temp_measure_t;

// Parameters for the setGPOUTFunction() funciton
typedef enum {
	BQ27441_SOC_INT, 					// Set GPOUT to SOC_INT functionality
	BQ27441_BAT_LOW  					// Set GPOUT to BAT_LOW functionality
} BQ27441_Gpout_function_t;

#define BQ27441_TIMING_100KHZ       0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define BQ27441_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */
#define BQ27441_TIMEOUT_MAX    			0x2000 /*<! The value of the maximal timeout for BUS waiting loops */

#define BQ27441_LRWAN_INSTANCE                   I2C1
#define BQ27441_LRWAN_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define BQ27441_LRWAN_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define BQ27441_LRWAN_SCL_SDA_AF                 GPIO_AF4_I2C1
#define BQ27441_LRWAN_SCL_SDA_GPIO_PORT          GPIOB
#define BQ27441_LRWAN_SCL_PIN                    GPIO_PIN_8
#define BQ27441_LRWAN_SDA_PIN                    GPIO_PIN_9

#define BQ27441_LRWAN_FORCE_RESET()              __I2C1_FORCE_RESET()
#define BQ27441_LRWAN_RELEASE_RESET()            __I2C1_RELEASE_RESET()

#define BQ27441_LRWAN_EV_IRQn                    I2C1_IRQn

#define BQ27441_NBIOT_INSTANCE                   I2C2
#define BQ27441_NBIOT_CLK_ENABLE()               __I2C2_CLK_ENABLE()
#define BQ27441_NBIOT_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define BQ27441_NBIOT_SCL_SDA_AF                 GPIO_AF5_I2C2
#define BQ27441_NBIOT_SCL_SDA_GPIO_PORT          GPIOB
#define BQ27441_NBIOT_SCL_PIN                    GPIO_PIN_13
#define BQ27441_NBIOT_SDA_PIN                    GPIO_PIN_14

#define BQ27441_NBIOT_FORCE_RESET()              __I2C2_FORCE_RESET()
#define BQ27441_NBIOT_RELEASE_RESET()            __I2C2_RELEASE_RESET()

#define BQ27441_NBIOT_EV_IRQn                    I2C2_IRQn


#define BQ27441_I2C_ADDRESS 0x55 							// Default I2C address of the BQ27441-G1A
#define BQ27441_I2C_TIMEOUT 2000

#define BQ27441_UNIT_SCALE 10									// Unit scale measure: units in 1/10th mA 

// -- General Constants --
// BQ27441 constants adapted from github.com/sparkfun/SparkFun_BQ27441_Arduino_Library/blob/master/src/BQ27441_Definitions.h
#define BQ27441_UNSEAL_KEY							0x8000	// Secret code to unseal the BQ27441-G1A
#define BQ27441_DEVICE_ID								0x0421	// Default device ID


// -- Standard Commands -
// The fuel gauge uses a series of 2-byte standard commands to enable system 
// reading and writing of battery information. Each command has an associated
// sequential command-code pair.
#define BQ27441_COMMAND_CONTROL					0x00 	// Control()
#define BQ27441_COMMAND_TEMP						0x02 	// Temperature()
#define BQ27441_COMMAND_VOLTAGE					0x04 	// Voltage()
#define BQ27441_COMMAND_FLAGS						0x06 	// Flags()
#define BQ27441_COMMAND_NOM_CAPACITY		0x08 	// NominalAvailableCapacity()
#define BQ27441_COMMAND_AVAIL_CAPACITY	0x0A 	// FullAvailableCapacity()
#define BQ27441_COMMAND_REM_CAPACITY		0x0C 	// RemainingCapacity()
#define BQ27441_COMMAND_FULL_CAPACITY		0x0E 	// FullChargeCapacity()
#define BQ27441_COMMAND_AVG_CURRENT			0x10 	// AverageCurrent()
#define BQ27441_COMMAND_STDBY_CURRENT		0x12 	// StandbyCurrent()
#define BQ27441_COMMAND_MAX_CURRENT			0x14 	// MaxLoadCurrent()
#define BQ27441_COMMAND_AVG_POWER				0x18 	// AveragePower()
#define BQ27441_COMMAND_SOC							0x1C 	// StateOfCharge()
#define BQ27441_COMMAND_INT_TEMP				0x1E 	// InternalTemperature()
#define BQ27441_COMMAND_SOH							0x20 	// StateOfHealth()
#define BQ27441_COMMAND_REM_CAP_UNFL		0x28 	// RemainingCapacityUnfiltered()
#define BQ27441_COMMAND_REM_CAP_FIL			0x2A 	// RemainingCapacityFiltered()
#define BQ27441_COMMAND_FULL_CAP_UNFL		0x2C 	// FullChargeCapacityUnfiltered()
#define BQ27441_COMMAND_FULL_CAP_FIL		0x2E 	// FullChargeCapacityFiltered()
#define BQ27441_COMMAND_SOC_UNFL				0x30 	// StateOfChargeUnfiltered()


// -- Control Sub-commands --
// Issuing a Control() command requires a subsequent 2-byte subcommand. These
// additional bytes specify the particular control function desired. The 
// Control() command allows the system to control specific features of the fuel
// gauge during normal operation and additional features when the device is in 
// different access modes.
#define BQ27441_CONTROL_STATUS					0x00
#define BQ27441_CONTROL_DEVICE_TYPE			0x01
#define BQ27441_CONTROL_FW_VERSION			0x02
#define BQ27441_CONTROL_DM_CODE					0x04
#define BQ27441_CONTROL_PREV_MACWRITE		0x07
#define BQ27441_CONTROL_CHEM_ID					0x08
#define BQ27441_CONTROL_BAT_INSERT			0x0C
#define BQ27441_CONTROL_BAT_REMOVE			0x0D
#define BQ27441_CONTROL_SET_HIBERNATE		0x11
#define BQ27441_CONTROL_CLEAR_HIBERNATE	0x12
#define BQ27441_CONTROL_SET_CFGUPDATE		0x13
#define BQ27441_CONTROL_SHUTDOWN_ENABLE	0x1B
#define BQ27441_CONTROL_SHUTDOWN				0x1C
#define BQ27441_CONTROL_SEALED					0x20
#define BQ27441_CONTROL_PULSE_SOC_INT		0x23
#define BQ27441_CONTROL_RESET						0x41
#define BQ27441_CONTROL_SOFT_RESET			0x42
#define BQ27441_CONTROL_EXIT_CFGUPDATE	0x43
#define BQ27441_CONTROL_EXIT_RESIM			0x44


// -- Control Status Word - Bit Definitions --
// Bit positions for the 16-bit data of CONTROL_STATUS.
// CONTROL_STATUS instructs the fuel gauge to return status information to 
// Control() addresses 0x00 and 0x01. The read-only status word contains status
// bits that are set or cleared either automatically as conditions warrant or
// through using specified subcommands.
#define BQ27441_STATUS_SHUTDOWNEN				(1<<15)
#define BQ27441_STATUS_WDRESET					(1<<14)
#define BQ27441_STATUS_SS								(1<<13)
#define BQ27441_STATUS_CALMODE					(1<<12)
#define BQ27441_STATUS_CCA							(1<<11)
#define BQ27441_STATUS_BCA							(1<<10)
#define BQ27441_STATUS_QMAX_UP					(1<<9)
#define BQ27441_STATUS_RES_UP						(1<<8)
#define BQ27441_STATUS_INITCOMP					(1<<7)
#define BQ27441_STATUS_HIBERNATE				(1<<6)
#define BQ27441_STATUS_SLEEP						(1<<4)
#define BQ27441_STATUS_LDMD							(1<<3)
#define BQ27441_STATUS_RUP_DIS					(1<<2)
#define BQ27441_STATUS_VOK							(1<<1)


// -- Flag Command - Bit Definitions --
// Bit positions for the 16-bit data of Flags()
// This read-word function returns the contents of the fuel gauging status
// register, depicting the current operating status.
#define BQ27441_FLAG_OT									(1<<15)
#define BQ27441_FLAG_UT									(1<<14)
#define BQ27441_FLAG_FC									(1<<9)
#define BQ27441_FLAG_CHG								(1<<8)
#define BQ27441_FLAG_OCVTAKEN						(1<<7)
#define BQ27441_FLAG_ITPOR							(1<<5)
#define BQ27441_FLAG_CFGUPMODE					(1<<4)
#define BQ27441_FLAG_BAT_DET						(1<<3)
#define BQ27441_FLAG_SOC1								(1<<2)
#define BQ27441_FLAG_SOCF								(1<<1)
#define BQ27441_FLAG_DSG								(1<<0)


// -- Extended Data Commands --
// Extended data commands offer additional functionality beyond the standard
// set of commands. They are used in the same manner; however, unlike standard
// commands, extended commands are not limited to 2-byte words.
#define BQ27441_EXTENDED_OPCONFIG				0x3A 	// OpConfig()
#define BQ27441_EXTENDED_CAPACITY				0x3C 	// DesignCapacity()
#define BQ27441_EXTENDED_DATACLASS			0x3E 	// DataClass()
#define BQ27441_EXTENDED_DATABLOCK			0x3F	// DataBlock()
#define BQ27441_EXTENDED_BLOCKDATA			0x40 	// BlockData()
#define BQ27441_EXTENDED_CHECKSUM				0x60 	// BlockDataCheckSum()
#define BQ27441_EXTENDED_CONTROL				0x61 	// BlockDataControl()

// -- Configuration Class, Subclass ID's --
// To access a subclass of the extended data, set the DataClass() function
// with one of these values.
// Configuration Classes
#define BQ27441_ID_SAFETY								2   	// Safety
#define BQ27441_ID_CHG_TERMINATION			36  	// Charge Termination
#define BQ27441_ID_CONFIG_DATA					48  	// Data
#define BQ27441_ID_DISCHARGE						49  	// Discharge
#define BQ27441_ID_REGISTERS						64  	// Registers
#define BQ27441_ID_POWER								68  	// Power
// Gas Gauging Classes
#define BQ27441_ID_IT_CFG								80  	// IT Cfg
#define BQ27441_ID_CURRENT_THRESH				81  	// Current Thresholds
#define BQ27441_ID_STATE								82  	// State
// Ra Tables Classes
#define BQ27441_ID_R_A_RAM							89  	// R_a RAM
// Calibration Classes
#define BQ27441_ID_CALIB_DATA						104 	// Data
#define BQ27441_ID_CC_CAL								105 	// CC Cal
#define BQ27441_ID_CURRENT							107 	// Current
// Security Classes
#define BQ27441_ID_CODES								112 	// Codes

// -- OpConfig Register - Bit Definitions --
// Bit positions of the OpConfig Register
#define BQ27441_OPCONFIG_BIE      			(1<<13)
#define BQ27441_OPCONFIG_BI_PU_EN 			(1<<12)
#define BQ27441_OPCONFIG_GPIOPOL  			(1<<11)
#define BQ27441_OPCONFIG_SLEEP    			(1<<5)
#define BQ27441_OPCONFIG_RMFCC    			(1<<4)
#define BQ27441_OPCONFIG_BATLOWEN 			(1<<2)
#define BQ27441_OPCONFIG_TEMPS    			(1<<0)


void BQ27441_Init( BQ27441_SENSOR sensor );
BQ27441_Error_et BQ27441_Begin(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_SetCapacity(BQ27441_SENSOR sensor, uint16_t capacity);
BQ27441_Error_et BQ27441_SetDesignEnergy(BQ27441_SENSOR sensor, uint16_t energy);
BQ27441_Error_et BQ27441_SetTerminateVoltage(BQ27441_SENSOR sensor, uint16_t voltage);
BQ27441_Error_et BQ27441_SetTaperRate(BQ27441_SENSOR sensor, uint16_t rate);

/********************** Battery Characteristics Functions ********************/
BQ27441_Error_et BQ27441_Temperature(BQ27441_SENSOR sensor, BQ27441_Temp_measure_t type, uint16_t* result);
BQ27441_Error_et BQ27441_Soh(BQ27441_SENSOR sensor, BQ27441_Soh_measure_t type, uint8_t* result);
BQ27441_Error_et BQ27441_Soc(BQ27441_SENSOR sensor, BQ27441_Soc_measure_t type, uint16_t* result);
BQ27441_Error_et BQ27441_Power(BQ27441_SENSOR sensor, int16_t* result);
BQ27441_Error_et BQ27441_Capacity(BQ27441_SENSOR sensor, BQ27441_Capacity_measure_t type, uint16_t* result);
BQ27441_Error_et BQ27441_Current(BQ27441_SENSOR sensor, BQ27441_Current_measure_t type, int16_t* result);
BQ27441_Error_et BQ27441_Voltage(BQ27441_SENSOR sensor, uint16_t* result);

/************************** GPOUT Control Functions **************************/
uint8_t BQ27441_SOCFClearThreshold(BQ27441_SENSOR sensor, uint8_t* result);
BQ27441_Error_et BQ27441_SetSOCFThresholds(BQ27441_SENSOR sensor, uint8_t set, uint8_t clear);
BQ27441_Error_et BQ27441_SocFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_SocfFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_ItporFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_FcFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_ChgFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_PulseGPOUT(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_SetSOCIDelta(BQ27441_SENSOR sensor, uint8_t delta);
uint8_t BQ27441_SociDelta(BQ27441_SENSOR sensor, uint8_t* result);
BQ27441_Error_et BQ27441_DsgFlag(BQ27441_SENSOR sensor, bool* result);
BQ27441_Error_et BQ27441_SOCFSetThreshold(BQ27441_SENSOR sensor, uint8_t* result);
BQ27441_Error_et BQ27441_setSOC1Thresholds(BQ27441_SENSOR sensor, uint8_t set, uint8_t clear);
BQ27441_Error_et BQ27441_SOC1ClearThreshold(BQ27441_SENSOR sensor, uint8_t* result);
BQ27441_Error_et BQ27441_SOC1SetThreshold(BQ27441_SENSOR sensor, uint8_t* result);
BQ27441_Error_et BQ27441_SetGPOUTFunction(BQ27441_SENSOR sensor, BQ27441_Gpout_function_t function);
bool BQ27441_GPOUTFunction(BQ27441_SENSOR sensor, bool* result);
bool BQ27441_SetGPOUTPolarity(BQ27441_SENSOR sensor, bool activeHigh);
bool BQ27441_GPOUTPolarity(BQ27441_SENSOR sensor, bool* result);

	
// -- Control Subcommands --
BQ27441_Error_et BQ27441_DeviceType(BQ27441_SENSOR sensor, uint16_t* result);
BQ27441_Error_et BQ27441_EnterConfig(BQ27441_SENSOR sensor, bool userControl);
BQ27441_Error_et BQ27441_ExitConfig(BQ27441_SENSOR sensor, bool resim);
BQ27441_Error_et BQ27441_Flags(BQ27441_SENSOR sensor, uint16_t* result);
BQ27441_Error_et BQ27441_Status(BQ27441_SENSOR sensor, uint16_t* result);

// - Privates
BQ27441_Error_et BQ27441_Sealed(BQ27441_SENSOR sensor, bool* ret);
BQ27441_Error_et BQ27441_Seal(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_Unseal(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_OpConfig(BQ27441_SENSOR sensor, uint16_t* result);
BQ27441_Error_et BQ27441_WriteOpConfig(BQ27441_SENSOR sensor, uint16_t value);
BQ27441_Error_et BQ27441_SoftReset(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_ReadWord(BQ27441_SENSOR sensor, uint16_t subAddress, uint16_t* result);
BQ27441_Error_et BQ27441_ReadControlWord(BQ27441_SENSOR sensor, uint16_t function, uint16_t* result);
BQ27441_Error_et BQ27441_ExecuteControlWord(BQ27441_SENSOR sensor, uint16_t function);

// -- Extended Data Commands --
BQ27441_Error_et BQ27441_BlockDataControl(BQ27441_SENSOR sensor);
BQ27441_Error_et BQ27441_BlockDataClass(BQ27441_SENSOR sensor, uint8_t id);
BQ27441_Error_et BQ27441_BlockDataOffset(BQ27441_SENSOR sensor, uint8_t offset);
BQ27441_Error_et BQ27441_BlockDataChecksum(BQ27441_SENSOR sensor, uint8_t* csum);
BQ27441_Error_et BQ27441_ReadBlockData(BQ27441_SENSOR sensor, uint8_t offset, uint8_t* ret);
BQ27441_Error_et BQ27441_WriteBlockData(BQ27441_SENSOR sensor, uint8_t offset, uint8_t data);
BQ27441_Error_et BQ27441_ComputeBlockChecksum(BQ27441_SENSOR sensor, uint8_t* csum);
BQ27441_Error_et BQ27441_WriteBlockChecksum(BQ27441_SENSOR sensor, uint8_t csum);
BQ27441_Error_et BQ27441_ReadExtendedData(BQ27441_SENSOR sensor, uint8_t classID, uint8_t offsetn, uint8_t* retData);
BQ27441_Error_et BQ27441_WriteExtendedData(BQ27441_SENSOR sensor, uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len);

// -- I2C Read/Write -- 
BQ27441_Error_et BQ27441_ReadBytes(BQ27441_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint8_t *Data, uint16_t NumByteToRead);
BQ27441_Error_et BQ27441_WriteBytes(BQ27441_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint8_t *Data, uint16_t NumByteToWrite );

// -- I2C Routines --
static uint8_t BQ27441_I2C_Init( BQ27441_SENSOR sensorType );
static uint8_t BQ27441_I2C_WriteData(I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t BQ27441_I2C_ReadData( I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static void BQ27441_I2C_Error( I2C_HandleTypeDef* handler, uint8_t Addr );
static void BQ27441_I2C_MspInit(BQ27441_SENSOR sensorType);

#ifdef __cplusplus
}
#endif

#endif /* __BQ27441 */
