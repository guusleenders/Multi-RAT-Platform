/*!
 * \file      Commissioning.h
 *
 * \brief     End device commissioning parameters
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  * @file    commissioning.h
  * @author  MCD Application Team
  * @brief   End device commissioning parameters
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__

#ifdef __cplusplus
extern "C" {
#endif
/*!
 ******************************************************************************
 ********************************** WARNING ***********************************
 ******************************************************************************
  The crypto-element implementation supports both 1.0.x and 1.1.x LoRaWAN
  versions of the specification.
  Thus it has been decided to use the 1.1.x keys and EUI name definitions.
  The below table shows the names equivalence between versions:
               +---------------------+-------------------------+
               |       1.0.x         |          1.1.x          |
               +=====================+=========================+
               | LORAWAN_DEVICE_EUI  | LORAWAN_DEVICE_EUI      |
               +---------------------+-------------------------+
               | LORAWAN_APP_EUI     | LORAWAN_JOIN_EUI        |
               +---------------------+-------------------------+
               | LORAWAN_GEN_APP_KEY | LORAWAN_APP_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_APP_KEY     | LORAWAN_NWK_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_F_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_S_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_NWK_S_ENC_KEY   |
               +---------------------+-------------------------+
               | LORAWAN_APP_S_KEY   | LORAWAN_APP_S_KEY       |
               +---------------------+-------------------------+
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 */

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                            0

/*!
 * When using ABP activation the MAC layer must know in advance to which server
 * version it will be connected.
 */
#define ABP_ACTIVATION_LRWAN_VERSION_V10x                  0x01000300 // 1.0.3.0

#define ABP_ACTIVATION_LRWAN_VERSION                       ABP_ACTIVATION_LRWAN_VERSION_V10x

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                             true

/*!
 * IEEE Organizationally Unique Identifier ( OUI ) (big endian)
 * \remark This is unique to a company or organization
 */
#define IEEE_OUI                                           0x01, 0x01, 0x01

/*!
 * When set to 1 DevEui is LORAWAN_DEVICE_EUI
 * When set to 0 DevEui is automatically generated by calling
 *         BoardGetUniqueId function
 */
#define STATIC_DEVICE_EUI                                  1

/*!
 * Mote device IEEE EUI (big endian)
 *
 * \remark see STATIC_DEVICE_EUI comments
 */
#define LORAWAN_DEVICE_EUI                                 { 0x00, 0x18, 0xC7, 0x51, 0x84, 0x49, 0x5A, 0xBE }

/*!
 * App/Join server IEEE EUI (big endian)
 */
#define LORAWAN_JOIN_EUI                                   { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0x01, 0x1D }

/*!
 * Application root key
 * WARNING: NOT USED FOR 1.0.x DEVICES
 */
#define LORAWAN_APP_KEY                                    { 0x86, 0x94, 0x33, 0x2C, 0x61, 0xD2, 0xD6, 0x78, 0x5D, 0xBB, 0x18, 0x6A, 0x54, 0xEC, 0x21, 0xD5 }//{ 0xA4, 0xC1, 0xC2, 0xC1, 0xFE, 0x89, 0xEB, 0xB5, 0xB3, 0x09, 0x49, 0x10, 0x38, 0xB1, 0xD8, 0xC0 }

/*!
 * Application root key - Used to derive Multicast keys on 1.0.x devices.
 * WARNING: USED only FOR 1.0.x DEVICES
 */
#define LORAWAN_GEN_APP_KEY                                { 0x86, 0x94, 0x33, 0x2C, 0x61, 0xD2, 0xD6, 0x78, 0x5D, 0xBB, 0x18, 0x6A, 0x54, 0xEC, 0x21, 0xD5 }//{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }

/*!
 * Network root key
 * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_APP_KEY
 */
#define LORAWAN_NWK_KEY                                    { 0x86, 0x94, 0x33, 0x2C, 0x61, 0xD2, 0xD6, 0x78, 0x5D, 0xBB, 0x18, 0x6A, 0x54, 0xEC, 0x21, 0xD5 }//{ 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                                 ( uint32_t )0x13

/*!
 * When set to 1 DevAdd is LORAWAN_DEVICE_ADDRESS
 * When set to 0 DevAdd is automatically generated using
 *         a pseudo random generator seeded with a value derived from
 *         BoardUniqueId value
 */
#define STATIC_DEVICE_ADDRESS                              1

/*!
 * Device address on the network (big endian)
 *
 * \remark see STATIC_DEVICE_ADDRESS comments
 */
#define LORAWAN_DEVICE_ADDRESS                             ( uint32_t )0x26011E5C

/*!
 * Forwarding Network session integrity key
 * WARNING: NWK_S_KEY FOR 1.0.x DEVICES
 */
#define LORAWAN_F_NWK_S_INT_KEY                            { 0xDC, 0x4B, 0xAA, 0x0B, 0x79, 0xFA, 0x18, 0x40, 0xE0, 0x33, 0x56, 0x87, 0x24, 0xCD, 0xA3, 0x65 }

/*!
 * Serving Network session integrity key
 * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY
 */
#define LORAWAN_S_NWK_S_INT_KEY                            { 0xDC, 0x4B, 0xAA, 0x0B, 0x79, 0xFA, 0x18, 0x40, 0xE0, 0x33, 0x56, 0x87, 0x24, 0xCD, 0xA3, 0x65 }

/*!
 * Network session encryption key
 * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY
 */
#define LORAWAN_NWK_S_ENC_KEY                              { 0xDC, 0x4B, 0xAA, 0x0B, 0x79, 0xFA, 0x18, 0x40, 0xE0, 0x33, 0x56, 0x87, 0x24, 0xCD, 0xA3, 0x65 }

/*!
 * Application session key
 */
#define LORAWAN_APP_S_KEY                                  { 0xAC, 0xE5, 0x46, 0x1C, 0xC1, 0x21, 0x12, 0xB8, 0xEC, 0xFF, 0xBD, 0xDF, 0xE8, 0x73, 0x39, 0x5B }

#ifdef __cplusplus
}
#endif

#endif /* __LORA_COMMISSIONING_H__ */
