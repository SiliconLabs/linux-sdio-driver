/***************************************************************************/ /**
 * @file
 * @brief SEND BOOT INSN: send boot instructions to WiFi module
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

#include "sli_global.h"

int16 sli_secure_ping_pong_wr(uint32 ping_pong, uint8 *src_addr,
                              uint16 size_param);
/*==============================================*/
/**
 * @fn          int16 sli_boot_insn(uint8 type, uint16 *data)
 * @brief       Sends boot instructions to WiFi module
 * @param[in]   uint8 type, type of the insruction to perform
 * @param[in]   uint32 *data, pointer to data which is to be read/write
 * @param[out]  none
 * @return      errCode
 *              < 0  = Command issued failure/Invalid command
 *                0  = SUCCESS
 *              > 0  = Read value
 * @section description
 * This API is used to send boot instructions to WiFi module.
 */
extern int16 sli_mem_rd(uint32 reg_address, uint16 len, uint8 *value);
extern int16 sli_mem_wr(uint32 reg_address, uint16 len, uint8 *value);

int16 sli_boot_insn(uint8 type, uint16 *data)
{
  int16 retval = 0;
  uint16 local = 0;
  uint32 j = 0;
  uint32 cmd = 0;
  uint16 read_data = 0;
  volatile int32 loop_counter = 0;
#ifdef SLI_DEBUG_PRINT
  SLI_DPRINT(SLI_PL3, "\nBootInsn\n");
#endif

  switch (type) {
    case REG_READ:
      retval = sli_mem_rd(HOST_INTF_REG_OUT, 2, (uint8 *)&read_data);
      *data = read_data;
      break;

    case REG_WRITE:
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)data);
      break;

    case PING_WRITE:
#if SLI_SECURE_BOOT
#ifndef SLI_OFFSET_BASED
      retval = sli_secure_ping_pong_wr(0, (uint8 *)data, 4096);
#else
      retval = sli_secure_ping_pong_wr_offset(0, (uint8 *)data, 4096);
#endif
      local = 0xab49;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&local);
#else
      for (j = 0; j < 2048; j++) {
        retval =
          sli_mem_wr(0x18000 + (j * 2), 2, (uint8 *)((uint32)data + (j * 2)));
        if (retval < 0) {
          return retval;
        }
      }

      local = 0xab49;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&local);
#endif
      break;

    case PONG_WRITE:
#if SLI_SECURE_BOOT
#ifndef SLI_OFFSET_BASED
      retval = sli_secure_ping_pong_wr(1, (uint8 *)data, 4096);
#else
      retval = sli_secure_ping_pong_wr_offset(1, (uint8 *)data, 4096);
#endif
      local = 0xab4f;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&local);
#else
      for (j = 0; j < 2048; j++) {
        retval =
          sli_mem_wr(0x19000 + (j * 2), 2, (uint8 *)((uint32)data + (j * 2)));
        if (retval < 0) {
          return retval;
        }
      }
      // Perform the write operation
      local = 0xab4f;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&local);
#endif
      // Perform the write operation
      break;

    case BURN_NWP_FW:
      cmd = BURN_NWP_FW | HOST_INTERACT_REG_VALID;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&cmd);
      if (retval < 0) {
        return retval;
      }
      SLI_RESET_LOOP_COUNTER(loop_counter);
      SLI_WHILE_LOOP(loop_counter, SLI_LOOP_COUNT_UPGRADE_IMAGE) {
        retval = sli_mem_rd(HOST_INTF_REG_OUT, 2, (uint8 *)&read_data);
        if (retval < 0) {
          return retval;
        }
        if (read_data == (SEND_RPS_FILE | HOST_INTERACT_REG_VALID)) {
          break;
        }
      }
      SLI_CHECK_LOOP_COUNTER(loop_counter, SLI_LOOP_COUNT_UPGRADE_IMAGE);
      break;

    case LOAD_NWP_FW:
      cmd = LOAD_NWP_FW | HOST_INTERACT_REG_VALID;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&cmd);
      break;
    case LOAD_DEFAULT_NWP_FW_ACTIVE_LOW:
      cmd = LOAD_DEFAULT_NWP_FW_ACTIVE_LOW | HOST_INTERACT_REG_VALID;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&cmd);
      break;
    case SLI_UPGRADE_BL:
      cmd = SLI_UPGRADE_BL | HOST_INTERACT_REG_VALID;
      retval = sli_mem_wr(HOST_INTF_REG_IN, 2, (uint8 *)&cmd);
      if (retval < 0) {
        return retval;
      }
      SLI_RESET_LOOP_COUNTER(loop_counter);
      SLI_WHILE_LOOP(loop_counter, SLI_LOOP_COUNT_UPGRADE_IMAGE) {
        retval = sli_mem_rd(HOST_INTF_REG_OUT, 2, (uint8 *)&read_data);
        if (retval < 0) {
          return retval;
        }
        if (read_data == (SEND_RPS_FILE | HOST_INTERACT_REG_VALID)) {
          break;
        }
      }
      SLI_CHECK_LOOP_COUNTER(loop_counter, SLI_LOOP_COUNT_UPGRADE_IMAGE);
      break;
    default:
      retval = -2;
      break;
  }
  return retval;
}
/*==============================================*/
/**
 * @fn          int16_t sli_boot_req()
 * @brief       Sends boot instructions to WiFi module
 * @param[in]   uint32 *data
 * @param[in]   uint32 addr
 * @param[out]  none
 * @return      errCode
 *                0  = SUCCESS
 *              > 0  = Read value
 * @section description
 * This API is used to send boot instructions to WiFi module.
 */
int16 sli_boot_req(uint32 *data, uint32 addr)
{
  int16 retval = -1;
  if (data != NULL) {
    retval = sli_mem_rd(addr, 4, (uint32 *)data);
  }
  return retval;
}
