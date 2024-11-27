/***************************************************************************//**
 * @file
 * @brief Contains prototypes of utils used in sli_lib_util.c
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


#ifndef SLI_LIB_UTIL_H
#define SLI_LIB_UTIL_H

#include "sli_global.h"

void sli_uint32_to_4bytes(uint8 *dBuf, uint32 val);
void sli_uint16_to_2bytes(uint8 *dBuf, uint16 val);
uint16 sli_bytes2R_to_uint16(uint8 *dBuf);
#endif
