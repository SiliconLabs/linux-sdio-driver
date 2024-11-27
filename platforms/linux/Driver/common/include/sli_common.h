/***************************************************************************//**
 * @file
 * @brief Some common defines
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

#ifndef SLI_COMMON_H
#define SLI_COMMON_H

#ifndef UINT32
#define UINT32 unsigned int
#endif
#ifndef UINT16
#define UINT16 unsigned short
#endif
#ifndef UINT8
#define UINT8 unsigned char
#endif
#ifndef PVOID
#define PVOID void*
#endif

#ifndef INT32
#define INT32 int
#endif
#ifndef INT16
#define INT16 short
#endif
#ifndef INT8
#define INT8 char
#endif
#ifndef VOID
#define VOID void
#endif

extern UINT32 sli_zone_enabled;

#define SLI_STATIC   static
#define SLI_EXTERN   extern

#define SLI_STATUS_FAIL     -1
#define SLI_STATUS_SUCCESS   0
#define SLI_STATUS INT32

#define SLI_MAC_ADDR_LEN     6

#endif
