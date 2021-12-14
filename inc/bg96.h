#ifndef __BG96
#define __BG96

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "hw.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_uart.h"
#include "stm32l0xx_hal_gpio.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_dma.h"
#include "low_power_manager.h"
#include "tiny_vsnprintf.h"
#include "scheduler.h"
#include "util.h"

#include "vcom.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define BG96_FIRSTBOOT 1

#define BG96_RX_PORT GPIOA
#define BG96_RX_PIN GPIO_PIN_13
#define BG96_TX_PORT GPIOA
#define BG96_TX_PIN GPIO_PIN_14
#define BG96_SERIAL_AF GPIO_AF6_LPUART1

#define BG96_POWERKEY_PORT GPIOB
#define BG96_POWERKEY_PIN GPIO_PIN_15														
#define BG96_RESETKEY_PORT GPIOA
#define BG96_RESETKEY_PIN GPIO_PIN_8	

#define BG96_DTR_PORT GPIOB
#define BG96_DTR_PIN GPIO_PIN_12			

#define BG96_STATUS_PIN GPIO_PIN_9
#define BG96_STATUS_PORT GPIOA

#define BG96_REPLY_SIZE 120
#define BG96_IP_ADDRESS_SIZE 30
#define BG96_DOMAIN_NAME_SIZE 50
#define BG96_PORT_SIZE 8  

#define BG96_MAX_SEND_RETRIES 10
#define BG96_WAIT_FOR_POWERDOWN_MAX 240000

enum {
    BG96_AUTO_MODE = 0,
    BG96_GSM_MODE,
    BG96_CATM1_MODE,
    BG96_NBIOT_MODE,
};

// LTE Bands
#define BG96_LTE_B1 "1"
#define BG96_LTE_B2 "2"
#define BG96_LTE_B3 "4"
#define BG96_LTE_B4 "8"
#define BG96_LTE_B5 "10"
#define BG96_LTE_B8 "80"
#define BG96_LTE_B12 "800"
#define BG96_LTE_B13 "1000"
#define BG96_LTE_B18 "20000"
#define BG96_LTE_B19 "40000"
#define BG96_LTE_B20 "80000"
#define BG96_LTE_B26 "2000000"
#define BG96_LTE_B28 "8000000"
#define BG96_LTE_B39 "4000000000" // catm1 only
#define BG96_LTE_CATM1_ANY "400A0E189F"
#define BG96_LTE_CATNB1_ANY "A0E189F"
#define BG96_LTE_NO_CHANGE "0"

// GSM Bands
#define BG96_GSM_NO_CHANGE "0"
#define BG96_GSM_900 "1"
#define BG96_GSM_1800 "2"
#define BG96_GSM_850 "4"
#define BG96_GSM_1900 "8"
#define BG96_GSM_ANY "f"

#define BG96_SCRAMBLE_ON "0"
#define BG96_SCRAMBLE_OFF "1"

// EDRX configuration
#define BG96_EDRX_DISABLE 0
#define BG96_EDRX_ENABLE 1
#define BG96_EDRX_ENABLE_RESULT 2
#define BG96_EDRX_DISABLE_RESET 3
#define BG96_EDRX_GSM 2
#define BG96_EDRX_UTRAN 3
#define BG96_EDRX_CATM1 4
#define BG96_EDRX_NBIOT 5
#define BG96_EDRX_5_12 "0000"
#define BG96_EDRX_10_24 "0001"
#define BG96_EDRX_20_48 "0010"
#define BG96_EDRX_10485_76 "1111"

// Network configuration
#define BG96_NETWORK_GSM 0
#define BG96_NETWORK_CATM1 8
#define BG96_NETWORK_NBIOT 9

// Error format
#define BG96_ERROR_DISABLE 0
#define BG96_ERROR_CODE 1
#define BG96_ERROR_VERBOSE 2

// Network reporting (CREG)
#define BG96_NETWORK_REPORTING_DISABLE 0
#define BG96_NETWORK_REPORTING_STAT 1
#define BG96_NETWORK_REPORTING_INFO 2

// PSM 
#define BG96_PSM_ENABLE 1
#define BG96_PSM_DISABLE 0
#define BG96_PSM_RAU_10MIN "0000001" // Customization possible, usage: bits 8-6: unit; bits 5-1: number
#define BG96_PSM_RAU_1HR "0010001"
#define BG96_PSM_RAU_10HR "0100001"
#define BG96_PSM_RAU_2SEC "0110001"
#define BG96_PSM_RAU_30SEC "1000001"
#define BG96_PSM_RAU_1MIN "1010001"
#define BG96_PSM_GPSREADY_2SEC "0000001" // Customization possible, usage: bits 8-6: unit; bits 5-1: number
#define BG96_PSM_GPSREADY_1MIN "0010001"
#define BG96_PSM_GPSREADY_05HR "0100001"
#define BG96_PSM_GPSREADY_DEACTIVATED "1110001"
#define BG96_PSM_TAU_10MIN "0000001" // T3412 timer. Customization possible, usage: bits 8-6: unit; bits 5-1: number
#define BG96_PSM_TAU_1HR "0010001"
#define BG96_PSM_TAU_10HR "0100001"
#define BG96_PSM_TAU_2SEC "0110001"
#define BG96_PSM_TAU_30SEC "1000001"
#define BG96_PSM_TAU_1MIN "1010001"
#define BG96_PSM_ACTIVE_2SEC "0000001" // T3324 timer. Customization possible, usage: bits 8-6: unit; bits 5-1: number
#define BG96_PSM_ACTIVE_1MIN "0010001"
#define BG96_PSM_ACTIVE_05HR "0100001"
#define BG96_PSM_ACTIVE_DEACTIVATED "1110001"
#define BG96_PSM_VERSION_NO_NETWORK_COORDINATION 0b0001
#define BG96_PSM_VERSION_REL12_NO_NETWORK_RETENTION 0b0010
#define BG96_PSM_VERSION_REL12_WITH_NETWORK_RETENTION 0b0100
#define BG96_PSM_VERSION_PSM_BETWEEN_EDRX 0b1000

// GPS
#define BG96_GNSS_MODE_STANDALONE 1
#define BG96_GNSS_MODE_MS_BASED 2
#define BG96_GNSS_MODE_MS_ASSISTED 3
#define BG96_GNSS_MODE_SPEED 4
#define BG96_GNSS_FIXTIME 30 // Max positioning time
#define BG96_GNSS_ACCURACY 50 // Accuracy of positioning in meter
#define BG96_GNSS_FIXCOUNT 0 // Number of attempts for fix
#define BG96_GNSS_FIXDELAY 1 // Delay between first and second positioning in seconds

// TCP
#define BG96_TCP_CONTEXT_TYPE_IPV4 1
#define BG96_TCP_CONTEXT_TYPE_IPV6 2
#define BG96_TCP_CONTEXT_TYPE_IPV4V6 3
#define BG96_TCP_AUTH_TYPE_NONE 0
#define BG96_TCP_AUTH_TYPE_PAP 1
#define BG96_TCP_AUTH_TYPE_CHAP 2
#define BG96_TCP_AUTH_TYPE_PAPCHAP 3

// HTTP
#define BG96_HTTP_DISABLE 0
#define BG96_HTTP_ENABLE 1
#define BG96_HTTP_CONTENT_TYPE_APPLICATION 0
#define BG96_HTTP_CONTENT_TYPE_TEXT 1
#define BG96_HTTP_CONTENT_TYPE_APPLICATION_STREAM 2
#define BG96_HTTP_CONTENT_TYPE_MULTIPART 3
#define BG96_HTTP_RESPONSE_SIZE 100

#define BG96_URCINDICATION_UART1 "\"uart1\""
#define BG96_URCINDICATION_USBAT "\"usbat\""
#define BG96_URCINDICATION_USBMODEM "\"usbmodem\""

typedef enum BG96_statuses{
	BG96_ERROR, 
	BG96_OK,
	BG96_TIMEOUT
}BG96_Status_t;

typedef enum BG96_powerdown{
	BG96_POWERDOWN, 
	BG96_ACTIVE,
	BG96_PSM,
	BG96_POWERDOWN_ERROR
}BG96_Powerdown_t;

// Inits
void BG96_Init( void );
void BG96_DeInit(void);
void BG96_IoDeInit(void);

// Serial functions
void BG96_Send( const char *format, ... );
void BG96_Dma_IRQHandler( void );
static void BG96_PrintDMA( void );
static void BG96_StartDMA(char* buf, uint16_t buffLen);
void BG96_IRQHandler(void);
FlagStatus BG96_IsNewCharReceived(void);
uint8_t BG96_GetNewChar(void);
void BG96_ReceiveToBuffer( void );
void BG96_ParseResult( char *buffer );

// AT-supporting functions
void BG96_SendATCommand( char *buffer );
BG96_Status_t BG96_SendATCommandCheckReply( char *buffer , char *replyBuffer, uint32_t timeout);
BG96_Status_t BG96_SendATCommandGetReply( char *buffer , char *replyBuffer, uint32_t timeout);
BG96_Status_t _BG96_SendATCommandGetReply( char *buffer , char *replyBuffer, uint32_t timeout, bool run);

// General AT commands
BG96_Status_t BG96_PowerOn( void );
BG96_Status_t BG96_SetBaudRate( uint16_t baud );
BG96_Powerdown_t BG96_PowerDown( void );
BG96_Status_t BG96_SaveConfiguration( void );
BG96_Status_t BG96_ResetConfiguration( void );
BG96_Status_t BG96_ResetParameters( void );
BG96_Status_t BG96_GetIMEI( char* buffer );
BG96_Status_t BG96_GetFirmwareInfo( char* buffer );
BG96_Status_t BG96_GetHardwareInfo( char* buffer );
BG96_Status_t BG96_SetErrorFormat( uint8_t error );
BG96_Status_t BG96_CheckSIMPIN( char* reply );
BG96_Status_t BG96_SetSIMPIN( char* pin );
BG96_Status_t BG96_SetNetworkReporting( uint8_t creg );
BG96_Status_t BG96_SetGSMBand( char* gsmband );
BG96_Status_t BG96_SetCatM1Band( char* band );
BG96_Status_t BG96_SetNBIoTBand( char* band );
BG96_Status_t BG96_GetBandConfiguration( char* buffer );
BG96_Status_t BG96_SetScrambleConfiguration( bool scramble );
BG96_Status_t BG96_SetMode( uint8_t mode);
BG96_Status_t BG96_GetSignalQuality(char * buffer);
BG96_Status_t BG96_GetSignalStength(char * buffer);
BG96_Status_t BG96_GetNetworkInfo(char * buffer);
BG96_Status_t BG96_GetNetworkStatus(char * buffer);
BG96_Status_t BG96_GetCELevel(uint8_t * celevel);
BG96_Status_t BG96_DisableNetworkStatus( void );
BG96_Status_t BG96_GetAvailableNetworks(char * buffer);
BG96_Status_t BG96_SelectNetwork(uint16_t networkId, uint8_t mode);
BG96_Status_t BG96_SetPDPContext(char * url);
BG96_Status_t BG96_ConnectToOperator( uint32_t timeout );
BG96_Status_t BG96_SetEDRXConfiguration(uint8_t enable, uint8_t mode, char * edrx);
BG96_Status_t BG96_GetEDRXConfiguration(char * buffer);
BG96_Status_t BG96_GetPacketCounter( char* buffer );
BG96_Status_t BG96_ConfigureURCIndication( char* indication );
BG96_Status_t BG96_SetModemOptimization( void );
BG96_Status_t BG96_Sleep(void );
BG96_Status_t BG96_Wake( void );
BG96_Status_t BG96_WakeFromPSM( uint32_t timeout );
BG96_Status_t BG96_WakeFromPSMToSend( void );
BG96_Status_t BG96_EnablePSMIndication( void );
BG96_Status_t BG96_DisablePSMIndication( void );
BG96_Status_t BG96_SetPowerSavingMode(uint8_t mode, char * requestedRAU, char * requestedGPRSREADY, char * requestedTAU, char * requestedActiveTimer);
BG96_Status_t BG96_GetPowerSavingMode(char * buffer);
BG96_Status_t BG96_SetPowerSavingModeSettings(uint32_t threshold, uint8_t version);
BG96_Status_t BG96_SetPowerSavingModeImmediately( void );
BG96_Status_t BG96_WaitForPowerDown( uint32_t timeout );
bool BG96_IsPoweredDown( void );
BG96_Status_t BG96_WaitForConnection(uint32_t timeout);

// GNSS AT commands
BG96_Status_t BG96_GNSS_Enable( uint8_t mode, uint8_t fixTime,  uint8_t accuracy, uint16_t fixCount, uint16_t fixDelay);
BG96_Status_t BG96_GNSS_Disable( void );
BG96_Status_t BG96_GNSS_GetLocation(float * latitudePointer, float * longitudePointer, uint8_t timeout);

// TCP/UDP/IP commands
void BG96_SelectContextID(uint8_t context);
void BG96_SelectConnectID(uint8_t connect);
BG96_Status_t BG96_GetPacketCounters(uint16_t * downlink, uint16_t * uplink);
BG96_Status_t BG96_ResetPacketCounters( void );
BG96_Status_t BG96_ConfigureContext( void );
BG96_Status_t BG96_ConfigureContextAPN(uint8_t contextType, char * apn, char * username, char * password, uint8_t authentication );
BG96_Status_t BG96_ActivateContext( void );
BG96_Status_t BG96_DeactivateContext( void );
bool BG96_CheckIfContextActivated( void );
BG96_Status_t BG96_UDP_Start(char * ipaddress, uint32_t port);
BG96_Status_t BG96_UDP_Stop( void );
BG96_Status_t BG96_UDP_StartService(char * ipaddress, uint32_t port);
BG96_Status_t BG96_UDP_GetStatus(char * replyBuffer);
BG96_Status_t BG96_UDP_SendData( char * data, uint32_t timeout);
BG96_Status_t BG96_UDP_SendDataTo(char * ipaddress, uint32_t port, char * data);
	
// HTTP Commands
BG96_Status_t BG96_HTTP_SetContextID(uint8_t id);
BG96_Status_t BG96_HTTP_SetResponseHeaderID(uint8_t id);
BG96_Status_t BG96_HTTP_SetRequestHeaderID(uint8_t id);
BG96_Status_t BG96_HTTP_SetSSLContextID(uint8_t id);
BG96_Status_t BG96_HTTP_SetContentType(uint8_t id);
BG96_Status_t BG96_HTTP_SetURL(char * url);
BG96_Status_t BG96_HTTP_Get(uint16_t timeout);
BG96_Status_t BG96_HTTP_ReadResponse(uint16_t wait_time, char * buffer, uint16_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __BG96 */
