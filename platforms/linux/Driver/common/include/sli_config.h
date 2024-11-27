/***************************************************************************//**
 * @file
 * @brief CONFIG INIT contains the default configurations used in the api's
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef SLI_CONFIG_H
#define SLI_CONFIG_H


#include "sli_api.h"
/*==============================================*/
/**
 * Global Defines
 */


#define SLI_INTERFACE SLI_SDIO            //@ SLI_SPI or SLI_UART or SLI_USB or SLI_SDIO host interface for communication with module

#if (SLI_INTERFACE == SLI_SPI)
#ifndef SLI_SPI_INTERFACE
#define SLI_SPI_INTERFACE
#endif
#undef  SLI_UART_INTERFACE
#undef  SLI_USB_INTERFACE
#undef  SLI_SDIO_INTERFACE
#elif (SLI_INTERFACE == SLI_UART)
#ifndef SLI_UART_INTERFACE
#define SLI_UART_INTERFACE
#endif
#undef  SLI_SPI_INTERFACE
#undef  SLI_USB_INTERFACE
#undef  SLI_SDIO_INTERFACE
#elif (SLI_INTERFACE == SLI_USB)
#ifndef SLI_USB_INTERFACE
#define SLI_USB_INTERFACE
#endif
#undef  SLI_SPI_INTERFACE
#undef  SLI_UART_INTERFACE
#undef  SLI_SDIO_INTERFACE
#elif (SLI_INTERFACE == SLI_SDIO)
#ifndef SLI_SDIO_INTERFACE
#define SLI_SDIO_INTERFACE
#endif
#undef  SLI_SPI_INTERFACE
#undef  SLI_UART_INTERFACE
#undef  SLI_USB_INTERFACE
#endif

#ifdef SLI_UART_INTERFACE
#ifndef SLI_UART_DEVICE
#ifdef LINUX_PLATFORM
#define SLI_UART_DEVICE                    "/dev/ttyUSB0"
#elif WINDOWS 
#define SLI_UART_DEVICE                    "\\\\.\\COM98"
#define BYPASS_CARD_READY 							0 //@ 0 - for Card Ready Bypass and 1 - Wait for Card Ready 
#endif

#endif
#define SLI_USE_HOST_WAKEUP_AS_INTERRUPT    ENABLE
#endif
#define SLI_SECURE_BOOT            DISABLE
#define HOST_INTERACTION_MODE      ENABLE                      //@ ENABLE or DISABLE host interaction for bootloader
#define SLI_TCP_IP_BYPASS          ENABLE                     //@ ENABLE or DISABLE TCP/IP bypass mode
#if SLI_TCP_IP_BYPASS
#define SLI_TCP_IP_FEATURE_BIT_MAP  TCP_IP_FEAT_BYPASS
#else
#define SLI_TCP_IP_FEATURE_BIT_MAP (TCP_IP_FEAT_DHCPV4_CLIENT | TCP_IP_FEAT_HTTP_CLIENT)
#endif
#endif
