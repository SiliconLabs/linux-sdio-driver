/***************************************************************************//**
 * @file  sli_api.h
 * @brief API specific Defines
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


#ifndef SLI_API_H
#define SLI_API_H

/**
 * Include Files
 */
#include "sli_global.h"
#include "sli_app_util.h"
/**
 * Global Defines
 */

//! Host interfaces
#define SLI_SPI    0
#define SLI_UART   1
#define SLI_USB    2
#define SLI_SDIO   3

#ifdef RS9116
//! Upgrade Image types
#define BURN_NWP_FW                      'B'
#else
#define SLI_UPGRADE_IMAGE_I_FW           '2'
#endif
#define SLI_UPGRADE_BL                   '#'


//! Firmware Upgradation form host params
#define FW_UP_PL  0
#define FW_UP_REQ 1
#ifdef RS9116
#define SLI_RPS_HEADER 64
#else
#define SLI_RPS_HEADER 32
#endif
#define SLI_FW_UP_SUCCESS 0x0003

//!Load Image types
#ifdef RS9116
#define LOAD_NWP_FW                          'A'
#else
#define LOAD_NWP_FW                          '1'
#endif
#define LOAD_DEFAULT_NWP_FW_ACTIVE_LOW       0x71 

//!Select Default image
#define SELECT_DEFAULT_NWP_FW               '5'

// bypass bootup
#define ENABLE_GPIO_BASED_BYPASS            '7'
#define DISABLE_GPIO_BASED_BYPASS           '8'

//!load default
#define SLI_LOAD_DEFAULT                  '9'

//! Check CRC
#define CHECK_NWP_INTEGRITY                     'K'


#define HOST_INTF_REG_OUT                 0x4105003C
#define HOST_INTF_REG_IN                  0x41050034
#define BOARD_READY                       0xABCD  
#define REG_READ                          0xD1
#define REG_WRITE                         0xD2
#define PONG_WRITE                        0xD4
#define PING_WRITE                        0xD5
#define GPIO_RESET                        0xD7
#define LOAD_BOOTLOADER                   0xD8
#ifdef RS9116
#define HOST_INTERACT_REG_VALID           (0xA0 << 8)
#define HOST_INTERACT_REG_VALID_READ      (0xAB << 8)
#else
#define HOST_INTERACT_REG_VALID           (0xAB << 8)
#define HOST_INTERACT_REG_VALID_READ      (0xAB << 8)
#endif
#define PONG_AVAIL                        'O'
#define PING_AVAIL                        'I'
#define PONG_VALID                        'O'   
#define PING_VALID                        'I'   
#define LOADING_INITIATED                 '1'
#define SEND_RPS_FILE                     '2'
#define FWUP_SUCCESSFUL                   'S'
#define EOF_REACHED                       'E'
#define BOOTUP_OPTIONS_LAST_CONFIG_NOT_SAVED 0xF1
#define BOOTUP_OPTIONS_CHECKSUM_FAIL      0xF2
#define INVALID_OPTION                    0xF3
#define CHECKSUM_SUCCESS                  0xAA
#define CHECKSUM_FAILURE                  0xCC
#define CHECKSUM_INVALID_ADDRESS          0x4C

#define SLI_SUCCESS                        0         
#define SLI_BUSY                           -1
#define SLI_FAIL                           -2
#define SLI_BUFFER_FULL                    -3
#define SLI_IN_SLEEP                       -4


#define SLI_RESET_LOOP_COUNTER(X)              X = 0;
#define SLI_WHILE_LOOP(X, Y)                   while((X++) < (uint32)Y)
#define SLI_LOOP_COUNT_UPGRADE_IMAGE           0xFFFF
#define SLI_LOOP_COUNT_WAKEUP_REQ              0xFFFFFFFF       
#define SLI_LOOP_COUNT_WAKEUP_WAIT             0xFFFFFFFF       
#define SLI_LOOP_COUNT_UPGRADE_REQ             0xFFFF           
#define SLI_LOOP_COUNT_UPGRADE_CHUNK           0xFFFF           
#define SLI_LOOP_COUNT_UPGRADE_STATUS          0xFFFF           
#define SLI_LOOP_COUNT_SELECT_OPTION           0xFFFF           
#define SLI_CHECK_LOOP_COUNTER(X, Y)           { if(X >= Y)\
                                                  return -1;}

//!SPI Internal Register Offset
#define SLI_SPI_INT_REG_ADDR               0x00      //@ register access method     
#define SPI_SPI_MODE_REG_ADDR              0x08      //@ register access method     

//!Power Mode Constants
#define SLI_POWER_MODE_0                   0x0000       
#define SLI_POWER_MODE_1                   0x0001       
#define SLI_POWER_MODE_2                   0x0002       
#define SLI_POWER_MODE_3                   0x0003
#define SLI_POWER_MODE_8                   0x0008
#define SLI_POWER_MODE_9                   0x0009

#define RSI_RSP_SOFT_RESET                 0x1C

#define BIT(a) ((long int)1 << a)

/*=====================================================================================*/
/**
 *         This is platform dependent operation.Needs to be implemented 
 *         specific to the platform.This timer is mentioned in the following functions
 *             Application/TCPDemo/Source/main.c
 *             WLAN/SPI/Source/spi_functs.c
 *             WLAN/SPI/Source/spi_iface_init.c
 *     
 */



extern volatile sli_powerstate sli_pwstate;
/*
 * Function Prototype Definitions
 */


uint8* sli_fill_parameters(uint32 type, uint8 *buffer);
void config_gpio_output(uint8 value);
void config_gpio_input(void);
uint8 get_gpio_value();
uint8 get_spi_intr_gpio_value();
int16 sli_module_power_cycle(void);
int16 sli_execute_cmd(uint8 *descparam,uint8 *payloadparam,uint16 size_param);


#endif
