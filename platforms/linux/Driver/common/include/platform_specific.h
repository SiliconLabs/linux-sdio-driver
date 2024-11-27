/***************************************************************************//**
 * @file
 * @brief Platform specific 
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

#ifndef PLATFORM_SPECIFIC_H
#define PLATFORM_SPECIFIC_H
#include "sli_global.h"
 
#include "sli_nic.h"

#define SLI_DPRINT(lvl, fmt, args...)              if (lvl & SLI_DEBUG_LVL) printk(fmt, ##args)

#ifndef NULL
#define NULL 0
#endif
#endif
