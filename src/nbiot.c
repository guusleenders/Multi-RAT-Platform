// -------------------------------- NBIOT FUNCTIONS -------------------------------------

#include "nbiot.h"

void initNBIoT(void){
	startEnergyMeasurement(LTC2942_NBIOT);

	BG96_Init();
	BG96_PowerOn();
	
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
	
	// These are previous settings
	// BG96_CheckSIMPIN(buffer);
	// BG96_EnablePSMIndication();
	// BG96_SendATCommandGetReply("AT+CFUN=0\r\n", buffer, 300);
	// BG96_SetPowerSavingModeImmediately(); // Not available in current firmware
	
	PRINTF_LN("- Initialised");

	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	initEnergyStruct.packetType = INIT;
	initEnergyStruct.nbiotEnergy = uwh;
	#ifdef DEBUG
	PRINTF("\r\n||NB-IoT init energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
	#endif
}

void registerNBIoT(void){
	startEnergyMeasurement(LTC2942_NBIOT);
	
	// ---------- Configure/select network ---------- 
	BG96_ConfigureContext();
	BG96_SetPDPContext("\"m2minternet.proximus.be\"");
	BG96_SelectNetwork(20601, BG96_NETWORK_NBIOT);
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
	
	char buffertest[70];

	BG96_GetNetworkStatus(buffertest);
	#ifdef DEBUG
	PRINTF_LN("- Network status: %s", buffertest);
	#endif
	
	BG96_GetPowerSavingMode(buffertest);
	#ifdef DEBUG
	PRINTF_LN("- Power saving mode settings: %s\r\n", buffertest);
	#endif
	
	BG96_GetEDRXConfiguration(buffertest);
	#ifdef DEBUG
	PRINTF_LN("- Edrx mode settings: %s\r\n", buffertest);
	#endif
	
	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	energyStruct.nbiotPacketType = REGISTRATION;
	energyStruct.nbiotEnergy = uwh;
	#ifdef DEBUG
	PRINTF("\r\n||NB-IoT registration energy used: %d uAh/10 (%d uWh/10, %d/10 mJ )||\r\n", energy, uwh, uwh*3.6f);
	#endif
	
	// ---------- Go to PSM ---------- 
	//BG96_Sleep();
	BG96_SetPowerSavingModeImmediately();
	
	sendNBIoT();
	#ifdef DEBUG
	PRINTF_LN("- Register procedure complete");
	#endif

}

int8_t sendNBIoT(void){
	BG96_Init();
	
	startEnergyMeasurement(LTC2942_NBIOT);
	
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
			char buffer[] = "hallo"; 
			
			BG96_UDP_SendData(buffer, 30000);
			BG96_UDP_Stop();
			BG96_DeactivateContext();
			
			PRINTF_LN("- Done sending NB-IoT");
			BG96_SetPowerSavingModeImmediately();
			PRINTF_LN("- Waiting for PSM");
		}
	}
	
	// ---------- Wait for PSM ---------- 
	while(!BG96_IsPoweredDown()){ // TODO: add timer for shutdown; BG96 needs to be shut down before Sigfox
		BG96_WaitForPowerDown(60000);
		PRINTF_LN("- PSM failed again, try again.");
		if(!BG96_IsPoweredDown())
			BG96_SetPowerSavingModeImmediately();
	}
	if(BG96_IsPoweredDown()){
		PRINTF_LN("- Entered PSM after 1 retries");
		// Maybe reset controller after power down not happened for timeout?
	}else{
		PRINTF_LN("- PSM failed");
		BG96_SetPowerSavingModeImmediately();
	}
	
	stopEnergyMeasurementNBIoT();
	
	BG96_DeInit();
	BG96_IoDeInit();
	
	return 0; 
}

void stopEnergyMeasurementNBIoT( void ){
	uint16_t uwh = stopEnergyMeasurement(LTC2942_NBIOT);
	energyStruct.nbiotPacketType = SEND;
	energyStruct.nbiotEnergy = uwh;
	uint32_t mj = uwh * 3.6f;
	LTC2942_SetShutdown(LTC2942_NBIOT, 1);
	PRINTF("\r\n||NB-IoT packet energy used: %d/10 uAh (%d/10 uWh, %d/10 mJ )||\r\n", energy, uwh, mj);
}
