// -------------------------------- NBIOT FUNCTIONS -------------------------------------

#include "nbiot.h"

BG96_Powerdown_t powerStatus = BG96_POWERDOWN;

void initNBIoT(void){

	BG96_Init();
	BG96_PowerOn();
	
	powerStatus = BG96_ACTIVE; 
	
	char buffer[30];
	memset(buffer, '\0', 30);
	
	// ---------- Configure NB-IoT ---------- 
	BG96_SendATCommandGetReply("ATX1\r\n", buffer, 300); // Important for edrx?
	BG96_SendATCommandGetReply("AT+QCFG=\"psm/urc\",1\r\n", buffer, 300);
	BG96_SetErrorFormat(BG96_ERROR_VERBOSE);
	BG96_ConfigureURCIndication(BG96_URCINDICATION_USBAT);
	BG96_SetNetworkReporting(BG96_NETWORK_REPORTING_STAT);
	BG96_SetModemOptimization();
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00001010\""); // set tau timer to 1h, active timer to 30s (seems to work best in network)//20s
	BG96_SetEDRXConfiguration(1,5,"\"0010\"");
	BG96_SetMode(BG96_NBIOT_MODE);
	
	#ifdef DEBUG
	PRINTF_LN("- Initialised");
	#endif
	
}

void registerNBIoT(void){

	// ---------- Configure/select network ---------- 
	BG96_ConfigureContext();
	BG96_SetPDPContext("\"m2minternet.proximus.be\"");
	BG96_SelectNetwork(20601, BG96_NETWORK_NBIOT);
	
	char pinbuffer[30];
	memset(pinbuffer, '\0', 30);
	BG96_SendATCommandGetReply("AT+CPIN?\r\n", pinbuffer, 1000);
	
	#ifdef DEBUG
	PRINTF_LN("- Network selected");
	#endif
	
	// ---------- Connect to network ---------- 
	BG96_ConnectToOperator(90000);
	#ifdef DEBUG
	PRINTF_LN("- Connected to operator");
	#endif
	
	// ---------- Set power saving settings ---------- 
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00001010\""); // set tau timer to 1h, active timer to 30s (seems to work best in network)//20s
	BG96_SetEDRXConfiguration(1,5,"\"0010\"");
	BG96_SetPowerSavingModeSettings(20,12);
	BG96_SetModemOptimization();
	#ifdef DEBUG
	PRINTF_LN("- Set power saving settings");
	#endif
	
	// ---------- Get various modem parameters ---------- 
	uint8_t celevel; 
	BG96_GetCELevel(&celevel);
	#ifdef DEBUG
	PRINTF_LN("- CE Level: %d", celevel);
	#endif
	
	char* seperator = "|\0";
	sprintf(energyStruct.nbiot_conditions, "%d|", celevel);
	
	char buffer[70];

	BG96_GetNetworkStatus(buffer);
	strcat(energyStruct.nbiot_conditions, buffer);
	strcat(energyStruct.nbiot_conditions, seperator);
	#ifdef DEBUG
	PRINTF_LN("- Network status: %s", buffer);
	#endif
	
	BG96_GetPowerSavingMode(buffer);
	strcat(energyStruct.nbiot_conditions, buffer);
	strcat(energyStruct.nbiot_conditions, seperator);
	#ifdef DEBUG
	PRINTF_LN("- Power saving mode settings: %s\r\n", buffer);
	#endif
	
	BG96_GetEDRXConfiguration(buffer);
	strcat(energyStruct.nbiot_conditions, buffer);
	strcat(energyStruct.nbiot_conditions, seperator);
	#ifdef DEBUG
	PRINTF_LN("- Edrx mode settings: %s\r\n", buffer);
	#endif
	
	PRINTF_LN("- Total conditions: %s", energyStruct.nbiot_conditions);
	
	#ifdef DEBUG
	PRINTF_LN("- Register procedure complete");
	#endif

}

int8_t _sendNBIoT(bool sendingMeasuredEnergy, char * payload){
	
	PRINTF_LN("- Start sending NB-IoT");
	
	// ---------- Register power status for energystruct ---------- 
	if(!sendingMeasuredEnergy){	
		PRINTF_LN("- Power status: %d", powerStatus);
		energyStruct.nbiot_initStatus = powerStatus;
	}
	
	// ---------- Start energy measurement ----------
  if(!sendingMeasuredEnergy){	
		startEnergyMeasurement(LTC2942_NBIOT);
	}
	
	if(powerStatus == BG96_POWERDOWN){
		initNBIoT();
		registerNBIoT();
	}else{
		BG96_Init();
	}
	
	
	// ---------- Wake from psm and connect to network ---------- 
	if(BG96_WakeFromPSMToSend() == BG96_OK){
		// ---------- Get network info ---------- 
		#ifdef DEBUG
		char bufferNetworkInfo[100];
		BG96_GetNetworkInfo(bufferNetworkInfo);
		PRINTF_LN("- Network info: %s", bufferNetworkInfo);
		#endif 
		
		// ---------- Connect to server ---------- 
		HAL_Delay(200);
		if(BG96_UDP_Start("62.235.63.122",8891) != BG96_OK){
			PRINTF_LN("- UDP Start failed, going back to psm");
		}else{		

			BG96_UDP_SendData(payload, 30000);
			BG96_UDP_Stop();
			BG96_DeactivateContext();
			
			PRINTF_LN("- Done sending NB-IoT");
			BG96_SetPowerSavingModeImmediately();
			PRINTF_LN("- Waiting for PSM");
		}
	}
	
	// ---------- Wait for PSM ---------- 
	uint8_t counter = 0;
	while(!BG96_IsPoweredDown() && counter < 3){ 
		BG96_WaitForPowerDown(10000);
		PRINTF_LN("- PSM failed again, try again (%i).", counter);
		if(!BG96_IsPoweredDown())
			BG96_SetPowerSavingModeImmediately();
	  counter ++; 
	}
	if(!BG96_IsPoweredDown() && counter >= 3){
		PRINTF_LN("- Shutting down completely.");
		BG96_Powerdown_t pd = BG96_PowerDown();
		if (pd == BG96_POWERDOWN_ERROR){
			powerStatus = BG96_ACTIVE; 
			PRINTF_LN("- (PSM) Shutdown fail");
		}else{
			powerStatus = pd;
		}
	}
	
	// ---------- Stop everything ----------
	if(!sendingMeasuredEnergy){	
		stopEnergyMeasurementNBIoT();
	}
	BG96_DeInit();
	BG96_IoDeInit();
	PRINTF_LN("- Send done.");
	
	// ---------- Increase packet number ----------
	if(!sendingMeasuredEnergy){	
		energyStruct.nbiot_packetNumber++;
	}
	return 0; 
}

int8_t sendNBIoT(){
	char buffer[] = "hallo"; 
	return _sendNBIoT(false, buffer);
}

void sendEnergyStruct( void ){
	char buffer[300];
	memset(buffer, '\0', sizeof(buffer));
	sprintf(	buffer, 
					"%d,%d,"
					"%d,%d,%s,%d,"
					"%d,%d,%s,%d,"
					"%d,%d,%s,%d,",
					energyStruct.general_deviceID, energyStruct.general_bootID,
					energyStruct.nbiot_packetNumber, energyStruct.nbiot_energy, energyStruct.nbiot_conditions, energyStruct.nbiot_initStatus, 
					energyStruct.sigfox_packetNumber, energyStruct.sigfox_energy, energyStruct.sigfox_conditions, energyStruct.sigfox_initStatus,
					energyStruct.lorawan_packetNumber, energyStruct.lorawan_energy, energyStruct.lorawan_conditions, energyStruct.lorawan_initStatus);
	
	PRINTF_LN("Sending report: %s", buffer);
	PRINTF_LN("Now sending");
	_sendNBIoT(true, buffer);
}

void stopEnergyMeasurementNBIoT( void ){
	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	energyStruct.nbiot_energy = uwh;
	uint32_t mj = uwh * 3.6f;
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
	PRINTF("\r\n||NB-IoT packet energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, mj);
}

