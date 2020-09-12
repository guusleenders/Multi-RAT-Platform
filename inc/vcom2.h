
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCOM2_H__
#define __VCOM2_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hw_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Character added when a RX error has been detected */
#define AT_ERROR_RX_CHAR 0x01
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init the VCOM.
 * @param  None
 * @retval None
 */
void vcom2_Init(void (*TxCb)(void));

/**
 * @brief  DeInit the VCOM.
 * @param  None
 * @retval None
 */
void vcom2_DeInit(void);

/**
 * @brief  Init the VCOM IOs.
 * @param  None
 * @retval None
 */ 
void vcom2_IoInit(void);

/**
 * @brief  DeInit the VCOM IOs.
 * @param  None
 * @retval None
 */
void vcom2_IoDeInit(void);

/**
 * @brief  Sends string on com port
 * @param  String
 * @retval None
 */
void vcom2_Send(const char *format, ...);



#ifdef __cplusplus
}
#endif

#endif /* __VCOM2_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
