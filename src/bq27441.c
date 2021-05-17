
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
 *         File: bq27441.c
 *      Created: 2020-03-12
 *       Author: Guus Leenders
 *      Version: 0.1
 *
 *  Description: Adapted from 
 *  https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library/blob/master/src/SparkFunBQ27441.cpp
 *
 */

#include "bq27441.h"
#include "vcom.h"

static uint32_t I2C_EXPBD_Timeout = BQ27441_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef BQ27441_LRWAN_Handle;
static I2C_HandleTypeDef BQ27441_NBIOT_Handle;

bool _BQ27441_SealFlag[2] = {0, 0};
bool _BQ27441_UserConfigControl[2] = {0, 0};

float _BQ27441_CalibrationRatio[2] = {10, 2.5};	// LSB = 1mA/CR

void BQ27441_Init( BQ27441_SENSOR sensor ){
	BQ27441_I2C_Init(sensor);
}

// Initializes I2C and verifies communication with the BQ27441.
BQ27441_Error_et BQ27441_Begin(BQ27441_SENSOR sensor){
	uint16_t deviceID = 0;
	
	BQ27441_Init(sensor); // Initialize I2C master
	
	if(BQ27441_DeviceType(sensor, &deviceID) != BQ27441_OK){
		return BQ27441_ERROR;
	}
	PRINTF_LN("Device type: %d", deviceID);
	
		 // Read deviceType from BQ27441
	
	if (deviceID == BQ27441_DEVICE_ID){
		return BQ27441_OK; // If device ID is valid, return true
	}
	
	return BQ27441_ERROR; // Otherwise return false
}

// Configures the design capacity of the connected battery.
BQ27441_Error_et BQ27441_SetCapacity(BQ27441_SENSOR sensor, uint16_t capacity){
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0A (10)
	// Design capacity is a 2-byte piece of data - MSB first
	// Unit: mAh
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_STATE, 10, capacityData, 2);
}

// Configures the design energy of the connected battery.
BQ27441_Error_et BQ27441_SetDesignEnergy(BQ27441_SENSOR sensor, uint16_t energy){
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0C (12)
	// Design energy is a 2-byte piece of data - MSB first
	// Unit: mWh
	uint8_t enMSB = energy >> 8;
	uint8_t enLSB = energy & 0x00FF;
	uint8_t energyData[2] = {enMSB, enLSB};
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_STATE, 12, energyData, 2);
}

// Configures the terminate voltage.
BQ27441_Error_et BQ27441_SetTerminateVoltage(BQ27441_SENSOR sensor, uint16_t voltage){
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0F (16)
	// Termiante voltage is a 2-byte piece of data - MSB first
	// Unit: mV
	// Min 2500, Max 3700
	if(voltage<2500) voltage=2500;
	if(voltage>3700) voltage=3700;
	
	uint8_t tvMSB = voltage >> 8;
	uint8_t tvLSB = voltage & 0x00FF;
	uint8_t tvData[2] = {tvMSB, tvLSB};
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_STATE, 16, tvData, 2);
}

// Configures taper rate of connected battery.
BQ27441_Error_et BQ27441_SetTaperRate(BQ27441_SENSOR sensor, uint16_t rate){
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x1B (27)
	// Termiante voltage is a 2-byte piece of data - MSB first
	// Unit: 0.1h
	// Max 2000
	if(rate>2000) rate=2000;
	uint8_t trMSB = rate >> 8;
	uint8_t trLSB = rate & 0x00FF;
	uint8_t trData[2] = {trMSB, trLSB};
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_STATE, 27, trData, 2);
}

/********************** Battery Characteristics Functions ********************/

// Reads and returns the battery voltage
BQ27441_Error_et BQ27441_Voltage(BQ27441_SENSOR sensor, uint16_t* result){
	return BQ27441_ReadWord(sensor, BQ27441_COMMAND_VOLTAGE, result);
}

// Reads and returns the specified current measurement
BQ27441_Error_et BQ27441_Current(BQ27441_SENSOR sensor, BQ27441_Current_measure_t type, int16_t* result){
	BQ27441_Error_et error = BQ27441_ERROR;
	uint16_t res = 0;
	switch (type){
	case BQ27441_AVG:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_AVG_CURRENT, &res);
		break;
	case BQ27441_STBY:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_STDBY_CURRENT, &res);
		break;
	case BQ27441_MAX:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_MAX_CURRENT, &res);
		break;
	}
	PRINTF_LN("res: %d", res);
	*result = ((int16_t) res)*BQ27441_UNIT_SCALE/_BQ27441_CalibrationRatio[sensor];
	return error;
}

// Reads and returns the specified capacity measurement
BQ27441_Error_et BQ27441_Capacity(BQ27441_SENSOR sensor, BQ27441_Capacity_measure_t type, uint16_t* result){
	BQ27441_Error_et error = BQ27441_ERROR;
	switch (type){
	case BQ27441_REMAIN:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_REM_CAPACITY, result);
		break;
	case BQ27441_FULL:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_FULL_CAPACITY, result);
		break;
	case BQ27441_AVAIL:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_NOM_CAPACITY, result);
		break;
	case BQ27441_AVAIL_FULL:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_AVAIL_CAPACITY, result);
		break;
	case BQ27441_REMAIN_F: 
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_REM_CAP_FIL, result);
		break;
	case BQ27441_REMAIN_UF:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_REM_CAP_UNFL, result);
		break;
	case BQ27441_FULL_F:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_FULL_CAP_FIL, result);
		break;
	case BQ27441_FULL_UF:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_FULL_CAP_UNFL, result);
		break;
	case BQ27441_DESIGN:
		error = BQ27441_ReadWord(sensor, BQ27441_EXTENDED_CAPACITY, result);
		break;
	}
	*result = *result*BQ27441_UNIT_SCALE	/_BQ27441_CalibrationRatio[sensor];
	return error;
}

// Reads and returns measured average power
BQ27441_Error_et BQ27441_Power(BQ27441_SENSOR sensor, int16_t* result){
	BQ27441_Error_et error = BQ27441_ERROR;
	error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_AVG_POWER, (uint16_t*) result);
	*result = *result*BQ27441_UNIT_SCALE	/_BQ27441_CalibrationRatio[sensor];
	return error;
}

// Reads and returns specified state of charge measurement
BQ27441_Error_et BQ27441_Soc(BQ27441_SENSOR sensor, BQ27441_Soc_measure_t type, uint16_t* result){
	BQ27441_Error_et error = BQ27441_ERROR;
	switch (type){
	case BQ27441_FILTERED:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_SOC, result);
		break;
	case BQ27441_UNFILTERED:
		error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_SOC_UNFL, result);
		break;
	}
	return error;
}

// Reads and returns specified state of health measurement
BQ27441_Error_et BQ27441_Soh(BQ27441_SENSOR sensor, BQ27441_Soh_measure_t type, uint8_t* result){
	uint16_t sohRaw = 0;
	BQ27441_Error_et error = BQ27441_ReadWord(sensor, BQ27441_COMMAND_SOH, &sohRaw);
	
	uint8_t sohStatus = sohRaw >> 8;
	uint8_t sohPercent = sohRaw & 0x00FF;
	
	if (type == BQ27441_PERCENT)	
		*result = sohPercent;
	else
		*result =  sohStatus;
	
	return error; 
}

// Reads and returns specified temperature measurement
BQ27441_Error_et BQ27441_Temperature(BQ27441_SENSOR sensor, BQ27441_Temp_measure_t type, uint16_t* result){
	switch (type){
	case BQ27441_BATTERY:
		return BQ27441_ReadWord(sensor, BQ27441_COMMAND_TEMP, result);
	case BQ27441_INTERNAL_TEMP:
		return BQ27441_ReadWord(sensor, BQ27441_COMMAND_INT_TEMP, result);
	}
	return BQ27441_ERROR;
}

/************************** GPOUT Control Functions **************************/

// Get GPOUT polarity setting (active-high or active-low)
bool BQ27441_GPOUTPolarity(BQ27441_SENSOR sensor, bool* result){
	uint16_t opConfigRegister = 0;
	BQ27441_Error_et error = BQ27441_OpConfig(sensor, &opConfigRegister);
	*result = (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
	return error;
}

// Set GPOUT polarity to active-high or active-low
bool BQ27441_SetGPOUTPolarity(BQ27441_SENSOR sensor, bool activeHigh){
	uint16_t oldOpConfig = 0;
	BQ27441_OpConfig(sensor, &oldOpConfig);
	
	// Check to see if we need to update opConfig:
	if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)))
		return true;
		
	uint16_t newOpConfig = oldOpConfig;
	if (activeHigh)
		newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);
	
	return BQ27441_WriteOpConfig(sensor, newOpConfig);	
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool BQ27441_GPOUTFunction(BQ27441_SENSOR sensor, bool* result){
	uint16_t opConfigRegister = 0;
	BQ27441_Error_et error = BQ27441_OpConfig(sensor, &opConfigRegister);
	*result = (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);	
	return error;
}

// Set GPOUT function to BAT_LOW or SOC_INT
BQ27441_Error_et BQ27441_SetGPOUTFunction(BQ27441_SENSOR sensor, BQ27441_Gpout_function_t function){
	uint16_t oldOpConfig = 0;
	BQ27441_OpConfig(sensor, &oldOpConfig);
	
	// Check to see if we need to update opConfig:
	if ((function && (oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)))
		return BQ27441_OK;
	
	// Modify BATLOWN_EN bit of opConfig:
	uint16_t newOpConfig = oldOpConfig;
	if (function)
		newOpConfig |= BQ27441_OPCONFIG_BATLOWEN;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_BATLOWEN);

	// Write new opConfig
	return BQ27441_WriteOpConfig(sensor, newOpConfig);	
}

// Get SOC1_Set Threshold - threshold to set the alert flag
BQ27441_Error_et BQ27441_SOC1SetThreshold(BQ27441_SENSOR sensor, uint8_t* result){
	return BQ27441_ReadExtendedData(sensor, BQ27441_ID_DISCHARGE, 0, result);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
BQ27441_Error_et BQ27441_SOC1ClearThreshold(BQ27441_SENSOR sensor, uint8_t* result){
	return BQ27441_ReadExtendedData(sensor, BQ27441_ID_DISCHARGE, 1, result);	
}

// Set the SOC1 set and clear thresholds to a percentage
BQ27441_Error_et BQ27441_setSOC1Thresholds(BQ27441_SENSOR sensor, uint8_t set, uint8_t clear){
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
BQ27441_Error_et BQ27441_SOCFSetThreshold(BQ27441_SENSOR sensor, uint8_t* result){
	return BQ27441_ReadExtendedData(sensor, BQ27441_ID_DISCHARGE, 2, result);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27441_SOCFClearThreshold(BQ27441_SENSOR sensor, uint8_t* result){
	return BQ27441_ReadExtendedData(sensor, BQ27441_ID_DISCHARGE, 3, result);	
}

// Set the SOCF set and clear thresholds to a percentage
BQ27441_Error_et BQ27441_SetSOCFThresholds(BQ27441_SENSOR sensor, uint8_t set, uint8_t clear){
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
BQ27441_Error_et BQ27441_SocFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_SOC1;
	return error; 
}

// Check if the SOCF flag is set
BQ27441_Error_et BQ27441_SocfFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_SOCF;
	return error;
	
}

// Check if the ITPOR flag is set
BQ27441_Error_et BQ27441_ItporFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_ITPOR;
	return error;
}

// Check if the FC flag is set
BQ27441_Error_et BQ27441_FcFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_FC;
	return error;
}

// Check if the CHG flag is set
BQ27441_Error_et BQ27441_ChgFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_CHG;
	return error;
}

// Check if the DSG flag is set
BQ27441_Error_et BQ27441_DsgFlag(BQ27441_SENSOR sensor, bool* result){
	uint16_t flagState = 0;
	BQ27441_Error_et error = BQ27441_Flags(sensor, &flagState);
	*result = flagState & BQ27441_FLAG_DSG;
	return error; 
}

// Get the SOC_INT interval delta
uint8_t BQ27441_SociDelta(BQ27441_SENSOR sensor, uint8_t* result){
	return BQ27441_ReadExtendedData(sensor, BQ27441_ID_STATE, 26, result);
}

// Set the SOC_INT interval delta to a value between 1 and 100
BQ27441_Error_et BQ27441_SetSOCIDelta(BQ27441_SENSOR sensor, uint8_t delta){
	uint8_t soci = constrain(delta, 0, 100);
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
BQ27441_Error_et BQ27441_PulseGPOUT(BQ27441_SENSOR sensor){
	return BQ27441_ExecuteControlWord(sensor, BQ27441_CONTROL_PULSE_SOC_INT);
}

/******************************* Control Subcommands *********************************/

// Read the device type - should be 0x0421
BQ27441_Error_et BQ27441_DeviceType(BQ27441_SENSOR sensor, uint16_t* result){
	return BQ27441_ReadControlWord(sensor, BQ27441_CONTROL_DEVICE_TYPE, result);
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
BQ27441_Error_et BQ27441_EnterConfig(BQ27441_SENSOR sensor, bool userControl){
	if (userControl) _BQ27441_UserConfigControl[sensor] = true;
	
	bool sealed = false; 
	BQ27441_Sealed(sensor, &sealed);
	if (sealed){
		_BQ27441_SealFlag[sensor] = true;
		BQ27441_Unseal(sensor); // Must be unsealed before making changes
	}
	
	if (BQ27441_ExecuteControlWord(sensor, BQ27441_CONTROL_SET_CFGUPDATE)){
		int16_t timeout = BQ27441_I2C_TIMEOUT;
		uint16_t flags = 0;
		while ((timeout--) && (!(flags & BQ27441_FLAG_CFGUPMODE))){
			HAL_Delay(1);
			BQ27441_Flags(sensor, &flags);
		}
			
		
		if (timeout > 0)
			return BQ27441_OK;
	}
	
	return BQ27441_ERROR;
}

// Exit configuration mode with the option to perform a resimulation
BQ27441_Error_et BQ27441_ExitConfig(BQ27441_SENSOR sensor, bool resim){
	// There are two methods for exiting config mode:
	//    1. Execute the EXIT_CFGUPDATE command
	//    2. Execute the SOFT_RESET command
	// EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
	// measurement, and without resimulating to update unfiltered-SoC and SoC.
	// If a new OCV measurement or resimulation is desired, SOFT_RESET or
	// EXIT_RESIM should be used to exit config mode.
	if (resim){
		if (BQ27441_SoftReset(sensor)){
			int16_t timeout = BQ27441_I2C_TIMEOUT;
			uint16_t flags = 0;
			while ((timeout--) && (!(flags & BQ27441_FLAG_CFGUPMODE))){
				HAL_Delay(1);
				BQ27441_Flags(sensor, &flags);
			}
			if (timeout > 0){
				if (_BQ27441_SealFlag[sensor]) BQ27441_Seal(sensor); // Seal back up if we IC was sealed coming in
				return BQ27441_OK;
			}
		}
		return BQ27441_ERROR;
	}
	else{
		return BQ27441_ExecuteControlWord(sensor, BQ27441_CONTROL_EXIT_CFGUPDATE);
	}	
}

// Read the flags() command
BQ27441_Error_et BQ27441_Flags(BQ27441_SENSOR sensor, uint16_t* result){
	return BQ27441_ReadWord(sensor, BQ27441_COMMAND_FLAGS, result);
}

// Read the CONTROL_STATUS subcommand of control()
BQ27441_Error_et BQ27441_Status(BQ27441_SENSOR sensor, uint16_t* result){
	return BQ27441_ReadControlWord(sensor, BQ27441_CONTROL_STATUS, result);
}


// -- Privates-- 
// Check if the BQ27441-G1A is sealed or not.
BQ27441_Error_et BQ27441_Sealed(BQ27441_SENSOR sensor, bool* ret){
	uint16_t stat = 0;
	BQ27441_Error_et error = BQ27441_Status(sensor, &stat);
	*ret = stat & BQ27441_STATUS_SS;
	return error; 
}

// Seal the BQ27441-G1A
BQ27441_Error_et BQ27441_Seal(BQ27441_SENSOR sensor){
	return BQ27441_ReadControlWord(sensor, BQ27441_CONTROL_SEALED, NULL);
}

// UNseal the BQ27441-G1A
BQ27441_Error_et BQ27441_Unseal(BQ27441_SENSOR sensor){
	// To unseal the BQ27441, write the key to the control
	// command. Then immediately write the same key to control again.
	if (BQ27441_ReadControlWord(sensor, BQ27441_UNSEAL_KEY, NULL) == BQ27441_OK){
		return BQ27441_ReadControlWord(sensor, BQ27441_UNSEAL_KEY, NULL);
	}
	return BQ27441_ERROR;
}

// Read the 16-bit opConfig register from extended data
BQ27441_Error_et BQ27441_OpConfig(BQ27441_SENSOR sensor, uint16_t* result){ // uint16_t
	return BQ27441_ReadWord(sensor, BQ27441_EXTENDED_OPCONFIG, result);
}

// Write the 16-bit opConfig register in extended data
BQ27441_Error_et BQ27441_WriteOpConfig(BQ27441_SENSOR sensor, uint16_t value){
	uint8_t opConfigMSB = value >> 8;
	uint8_t opConfigLSB = value & 0x00FF;
	uint8_t opConfigData[2] = {opConfigMSB, opConfigLSB};
	
	// OpConfig register location: BQ27441_ID_REGISTERS id, offset 0
	return BQ27441_WriteExtendedData(sensor, BQ27441_ID_REGISTERS, 0, opConfigData, 2);	
}

// Issue a soft-reset to the BQ27441-G1A
BQ27441_Error_et BQ27441_SoftReset(BQ27441_SENSOR sensor){
	return BQ27441_ExecuteControlWord(sensor, BQ27441_CONTROL_SOFT_RESET);
}

// Read a 16-bit command word from the BQ27441-G1A
BQ27441_Error_et BQ27441_ReadWord(BQ27441_SENSOR sensor, uint16_t subAddress, uint16_t* result){
	uint8_t data[2];
	BQ27441_Error_et error = BQ27441_ReadBytes(sensor, BQ27441_I2C_ADDRESS, subAddress, data, 2);
	*result = ((uint16_t) data[1] << 8) | data[0];
	return error; 
}

// Read a 16-bit subcommand() from the BQ27441-G1A's control()
BQ27441_Error_et BQ27441_ReadControlWord(BQ27441_SENSOR sensor, uint16_t function, uint16_t* result){
	uint8_t command[2] = {(function & 0x00FF), (function >> 8)};
	uint8_t data[2] = {0, 0};
	

	BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, (uint8_t) 0, command, 2);
	
	if (BQ27441_ReadBytes(sensor, BQ27441_I2C_ADDRESS, (uint8_t) 0, data, 2) == BQ27441_OK){
		*result = ((uint16_t)data[1] << 8) | data[0];
		return BQ27441_OK;
	}

	return BQ27441_ERROR;
}

// Execute a subcommand() from the BQ27441-G1A's control()
BQ27441_Error_et BQ27441_ExecuteControlWord(BQ27441_SENSOR sensor, uint16_t function){
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};

	if (BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, (uint8_t) 0, command, 2))
		return BQ27441_ERROR;
	
	return BQ27441_OK;
}

/***************************Extended Data Commands ***************************/
 
// Issue a BlockDataControl() command to enable BlockData access
BQ27441_Error_et BQ27441_BlockDataControl(BQ27441_SENSOR sensor){
	uint8_t enableByte = 0x00;
	return BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
BQ27441_Error_et BQ27441_BlockDataClass(BQ27441_SENSOR sensor, uint8_t id){
	return BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
BQ27441_Error_et BQ27441_BlockDataOffset(BQ27441_SENSOR sensor, uint8_t offset){
	return BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
BQ27441_Error_et BQ27441_BlockDataChecksum(BQ27441_SENSOR sensor, uint8_t* csum){ //uint8_t
	return BQ27441_ReadBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_CHECKSUM, csum, 1);;
}

// Use BlockData() to read a byte from the loaded extended data
BQ27441_Error_et BQ27441_ReadBlockData(BQ27441_SENSOR sensor, uint8_t offset, uint8_t* ret){ //uint8_t
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	return BQ27441_ReadBytes(sensor, BQ27441_I2C_ADDRESS, address, ret, 1);
}

// Use BlockData() to write a byte to an offset of the loaded data
BQ27441_Error_et BQ27441_WriteBlockData(BQ27441_SENSOR sensor, uint8_t offset, uint8_t data){
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	return BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a 
// checksum based on the values.
BQ27441_Error_et BQ27441_ComputeBlockChecksum(BQ27441_SENSOR sensor, uint8_t* csum){ //uint8_t
	uint8_t data[32];
	BQ27441_ReadBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_BLOCKDATA, data, 32);

	*csum = 0;
	for (int i=0; i<32; i++){
		*csum += data[i];
	}
	*csum = 255 - *csum;
	return BQ27441_OK;
}

// Use the BlockDataCheckSum() command to write a checksum value
BQ27441_Error_et BQ27441_WriteBlockChecksum(BQ27441_SENSOR sensor, uint8_t csum){
	return BQ27441_WriteBytes(sensor, BQ27441_I2C_ADDRESS, BQ27441_EXTENDED_CHECKSUM, &csum, 1);	
}

// Read a byte from extended data specifying a class ID and position offset
BQ27441_Error_et BQ27441_ReadExtendedData(BQ27441_SENSOR sensor, uint8_t classID, uint8_t offset, uint8_t* retData){ //uint8_t
	*retData = 0;
	if (!_BQ27441_UserConfigControl[sensor]) BQ27441_EnterConfig(sensor, false);
		
	if (BQ27441_BlockDataControl(sensor) != BQ27441_OK) // // enable block data memory control
		return BQ27441_ERROR; // Return false if enable fails
	if (BQ27441_BlockDataClass(sensor, classID) != BQ27441_OK) // Write class ID using DataBlockClass()
		return BQ27441_ERROR;
	
	BQ27441_BlockDataOffset(sensor, offset / 32); // Write 32-bit block offset (usually 0)
	
	uint8_t oldCsum = 0;
	BQ27441_ComputeBlockChecksum(sensor,  &oldCsum); // Compute checksum going in
	BQ27441_BlockDataChecksum(sensor, &oldCsum);
	/*for (int i=0; i<32; i++)
		Serial.print(String(readBlockData(i)) + " ");*/
	BQ27441_ReadBlockData(sensor, offset % 32, retData); // Read from offset (limit to 0-31)
	
	if (!_BQ27441_UserConfigControl[sensor]) BQ27441_ExitConfig(sensor, false);
	
	return BQ27441_OK;
}

// Write a specified number of bytes to extended data specifying a 
// class ID, position offset.
BQ27441_Error_et BQ27441_WriteExtendedData(BQ27441_SENSOR sensor, uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len){
	if (len > 32)
		return BQ27441_ERROR;
	
	if (!_BQ27441_UserConfigControl[sensor]) BQ27441_EnterConfig(sensor, false);
	
	if (BQ27441_BlockDataControl(sensor) != BQ27441_OK) // // enable block data memory control
		return BQ27441_ERROR; // Return false if enable fails
	if (BQ27441_BlockDataClass(sensor, classID) != BQ27441_OK) // Write class ID using DataBlockClass()
		return BQ27441_ERROR;
	
	BQ27441_BlockDataOffset(sensor, offset / 32); // Write 32-bit block offset (usually 0)
	uint8_t oldCsum = 0;
  BQ27441_ComputeBlockChecksum(sensor,  &oldCsum); // Compute checksum going in
	BQ27441_BlockDataChecksum(sensor, &oldCsum);

	// Write data bytes:
	for (int i = 0; i < len; i++){
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		BQ27441_WriteBlockData(sensor, (offset % 32) + i, data[i]);
	}
	
	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = 0;
	BQ27441_ComputeBlockChecksum(sensor, &newCsum); // Compute the new checksum
	BQ27441_WriteBlockChecksum(sensor, newCsum);

	if (!_BQ27441_UserConfigControl[sensor]) BQ27441_ExitConfig(sensor, false);
	
	return BQ27441_OK;
}


/******************************* I2C Read/Write *********************************/

BQ27441_Error_et BQ27441_ReadBytes(BQ27441_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint8_t *Data, uint16_t NumByteToRead){
	uint8_t result = 0;
	if(sensor == BQ27441_LRWAN){
		result = BQ27441_I2C_ReadData(&BQ27441_LRWAN_Handle, addr<<1, RegAddr, Data, NumByteToRead );
	}else if(sensor == BQ27441_NBIOT){
		result = BQ27441_I2C_ReadData(&BQ27441_NBIOT_Handle, addr<<1, RegAddr, Data, NumByteToRead );
	}
  if ( result > 0)
    return BQ27441_ERROR;
  else
    return BQ27441_OK;
}

BQ27441_Error_et BQ27441_WriteBytes(BQ27441_SENSOR sensor, uint8_t addr, uint8_t RegAddr, uint8_t *Data, uint16_t NumByteToWrite ){
	
  //if ( NumByteToWrite > 1 ) RegAddr |= 0x80; // TODO: don't know if this still applies

	uint8_t result = 0;
	if(sensor == BQ27441_LRWAN){
		result = BQ27441_I2C_WriteData(&BQ27441_LRWAN_Handle,  addr<<1, RegAddr, Data, NumByteToWrite );
	}else if(sensor == BQ27441_NBIOT){
		result = BQ27441_I2C_WriteData(&BQ27441_NBIOT_Handle, addr<<1, RegAddr, Data, NumByteToWrite );
	}
	
  if ( result > 0)
    return BQ27441_ERROR;
  else
    return BQ27441_OK;
}

/******************************* I2C Routines *********************************/


static uint8_t BQ27441_I2C_Init( BQ27441_SENSOR sensorType ){
	
	if(sensorType == BQ27441_LRWAN){
		if(HAL_I2C_GetState( &BQ27441_LRWAN_Handle) == HAL_I2C_STATE_RESET ){

			/* I2C_EXPBD peripheral configuration */

			BQ27441_LRWAN_Handle.Init.Timing 				 = BQ27441_TIMING_400KHZ;    /* 400KHz */
			BQ27441_LRWAN_Handle.Init.OwnAddress1    = 0x33;
			BQ27441_LRWAN_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			
			BQ27441_LRWAN_Handle.Instance            = BQ27441_LRWAN_INSTANCE;

			/* Init the I2C */
			BQ27441_I2C_MspInit(BQ27441_LRWAN);
			HAL_I2C_Init( &BQ27441_LRWAN_Handle );
		}

		if( HAL_I2C_GetState( &BQ27441_LRWAN_Handle) == HAL_I2C_STATE_READY ){
			return 0;
		}
		else{
			return 1;
		}
	}else if(sensorType == BQ27441_NBIOT){
		if(HAL_I2C_GetState( &BQ27441_NBIOT_Handle) == HAL_I2C_STATE_RESET ){

			/* I2C_EXPBD peripheral configuration */

			BQ27441_NBIOT_Handle.Init.Timing 				 = BQ27441_TIMING_400KHZ;    /* 400KHz */
			BQ27441_NBIOT_Handle.Init.OwnAddress1    = 0x33;
			BQ27441_NBIOT_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
			
			BQ27441_NBIOT_Handle.Instance            = BQ27441_NBIOT_INSTANCE;

			/* Init the I2C */
			BQ27441_I2C_MspInit(BQ27441_NBIOT);
			HAL_I2C_Init( &BQ27441_NBIOT_Handle );
		}

		if( HAL_I2C_GetState( &BQ27441_NBIOT_Handle) == HAL_I2C_STATE_READY ){
			return 0;
		}
		else{
			return 1;
		}
	}else{
		return 0;
	}
	
}

static uint8_t BQ27441_I2C_WriteData(I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( handler, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    BQ27441_I2C_Error(handler, Addr );
    return 1;
  }
  else{
    return 0;
  }
}

static uint8_t BQ27441_I2C_ReadData( I2C_HandleTypeDef* handler, uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size ){

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( handler, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK ){

    /* Execute user timeout callback */
    BQ27441_I2C_Error(handler, Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}

static void BQ27441_I2C_Error( I2C_HandleTypeDef* handler, uint8_t Addr ){
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit( handler );

  /* Re-Initiaize the I2C comunication bus */
	if(handler->Instance == BQ27441_LRWAN_INSTANCE){
		BQ27441_I2C_Init(BQ27441_LRWAN);
	}else if(handler->Instance == BQ27441_NBIOT_INSTANCE){
		BQ27441_I2C_Init(BQ27441_NBIOT);
	}
  
}


static void BQ27441_I2C_MspInit(BQ27441_SENSOR sensorType){
  GPIO_InitTypeDef  GPIO_InitStruct;

	if(sensorType == BQ27441_LRWAN){
		BQ27441_LRWAN_SCL_SDA_GPIO_CLK_ENABLE(); // Enable I2C GPIO clocks; both are on port b

		// I2C_EXPBD SCL and SDA pins configuration
		GPIO_InitStruct.Pin        = BQ27441_LRWAN_SCL_PIN | BQ27441_LRWAN_SDA_PIN;
		GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed 		 = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pull       = GPIO_NOPULL;
		GPIO_InitStruct.Alternate  = BQ27441_LRWAN_SCL_SDA_AF;

		HAL_GPIO_Init( BQ27441_LRWAN_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

		BQ27441_LRWAN_CLK_ENABLE();			// Enable the I2C_EXPBD peripheral clock
		BQ27441_LRWAN_FORCE_RESET(); 		//Force the I2C peripheral clock reset
		BQ27441_LRWAN_RELEASE_RESET(); 	// Release the I2C peripheral clock reset

		/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(BQ27441_LRWAN_EV_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(BQ27441_LRWAN_EV_IRQn);
		
	}else if(sensorType == BQ27441_NBIOT){
		BQ27441_NBIOT_SCL_SDA_GPIO_CLK_ENABLE(); // Enable I2C GPIO clocks; both are on port b

		// I2C_EXPBD SCL and SDA pins configuration
		GPIO_InitStruct.Pin        = BQ27441_NBIOT_SCL_PIN | BQ27441_NBIOT_SDA_PIN;
		GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed 		 = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pull       = GPIO_NOPULL;
		GPIO_InitStruct.Alternate  = BQ27441_NBIOT_SCL_SDA_AF;

		HAL_GPIO_Init( BQ27441_NBIOT_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

		BQ27441_NBIOT_CLK_ENABLE();			// Enable the I2C_EXPBD peripheral clock
		BQ27441_NBIOT_FORCE_RESET(); 		//Force the I2C peripheral clock reset
		BQ27441_NBIOT_RELEASE_RESET(); 	// Release the I2C peripheral clock reset

		/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(BQ27441_NBIOT_EV_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(BQ27441_NBIOT_EV_IRQn);
	}
}
