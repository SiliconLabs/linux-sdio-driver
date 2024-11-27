/***************************************************************************//**
 * @file
 * @brief This file contains the function definition prototypes for HAL
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

#ifndef SLI_HAL_H
#define SLI_HAL_H


/**
 * INCLUDES
 */
#include "sli_global.h"

#ifdef SAM9_G20
#define SPI_INTR_GPIO_PIN AT91_PIN_PB8
#endif
#ifdef SAM9_G35
#define SPI_INTR_GPIO_PIN AT91_PIN_PA7
#endif
#ifdef SAM9_G45
#define SPI_INTR_GPIO_PIN AT91_PIN_PD0
#define SPI_WAKEUP_REQ_GPIO  AT91_PIN_PB10
#define SPI_WAKEUP_STAT_GPIO AT91_PIN_PB11

#endif
#ifdef X86
#define SPI_INTR_GPIO_PIN 0
#endif


/**
 * Function Prototypes
 */


void  sli_module_power(uint8 tf);
#endif
