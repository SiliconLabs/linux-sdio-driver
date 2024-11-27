/***************************************************************************/ /**
 * @file
 * @brief This contains all the functions with netlink socket usage
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
#include "sli_linux.h"
#include <net/genetlink.h>

#if KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(33)
#include <net/sock.h>
#endif
#include "sli_config.h"
extern SLI_STATUS sli_submit_rx_urb(PSLI_ADAPTER Adapter, UINT8 endpoint);

INT32 sli_read_req_from_userspace(struct sk_buff *skb_2,struct genl_info *info);
/* attributes (variables): the index in this enum is used as a reference for the
 * type, userspace application has to indicate the corresponding type the policy
 * is used for security considerations
 */
enum {
  SLI_USER_A_UNSPEC,
  SLI_USER_A_MSG,
  __SLI_USER_A_MAX,
};
#define SLI_USER_A_MAX (__SLI_USER_A_MAX - 1)

/* attribute policy: defines which attribute has which type (e.g int, char *
 * etc) possible values defined in net/netlink.h
 */
static struct nla_policy sli_user_genl_policy[SLI_USER_A_MAX + 1] = {
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
  [SLI_USER_A_MSG] = { .type = NLA_STRING },
#else
  [SLI_USER_A_MSG] = { .type = NLA_NUL_STRING },
#endif
};
extern struct net_device *glbl_net_device;

#define SLI_VERSION_NR 1
/* family definition */
static struct genl_family sli_user_gnl_family = {
  .hdrsize = 0,
  .name = "CTRL_PKT_TXRX",   //! the name of this family, used by userspace
  .version = SLI_VERSION_NR,   //! version number
  .maxattr = SLI_USER_A_MAX,
};

/* commands: enumeration of all commands (functions),
 * used by userspace application to identify command to be executed
 */
enum {
  SLI_USER_C_UNSPEC,
  SLI_USER_C_CMD,
  __SLI_USER_C_MAX,
};


#define SLI_USER_C_MAX (__SLI_USER_C_MAX - 1)

/*==============================================*/
/**
 * @fn          INT16 sli_update_device(PSLI_ADAPTER Adapter,
 *                                     UINT8 *data)
 * @brief       updates the device
 * @param[in]   PSLI_ADAPTER Adapter, Pointer to Adapter
 * @param[in]   UINT8 *data
 * @param[out]  none
 * @return      Reurns 0
 * @section description
 * This function used to parse the data to be updated.
 * This is used when user sends some info to update.
 * Used to update MAC address and FSM state
 */
INT16 sli_update_device(PSLI_ADAPTER Adapter, UINT8 *data)
{
  UINT8 type = *(data + SLI_DESC_LEN);

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_update_device:\n");
#endif
  switch (type) {
    case UPDATE_JOIN_DONE:
      sli_memcpy(Adapter->net_device0->dev_addr,
                 data + SLI_DESC_LEN + 1 /* type */, 6);
      Adapter->FSM_STATE = FSM_OPEN;
      break;

    case UPDATE_CONCURRENT_AP_JOIN_DONE:
      sli_memcpy(Adapter->net_device1->dev_addr,
                 data + SLI_DESC_LEN + 1 /* type */, 6);
      Adapter->FSM_STATE_AP1 = FSM_OPEN;
      break;

    case SOFT_RESET:
      Adapter->FSM_STATE = FSM_RESET;
      break;
#ifdef RSI_AUTOMATION_ENABLE
    case TCP_IP_BYPASS_MACRO:
      Adapter->tcp_ip_bypass = *(data + SLI_DESC_LEN + 1);
      break;
#endif

    default:
      break;
  }

  return 0;
}

/*==============================================*/
/**
 * @fn          INT32 sli_read_req_from_userspace(
 *                          struct genl_info *info)
 * @brief       Gets the command request from user space
 *              over netlink socket
 * @param[in]   struct sk_buff *skb_2, pointer to sk_buff structure
 * @param[in]   struct genl_info *info, read command info pointer
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to read the command from user over netlink
 * socket.
 */
INT32 sli_read_req_from_userspace(struct sk_buff *skb_2,struct genl_info *info)
{
  PSLI_ADAPTER Adapter = sli_getpriv(glbl_net_device);
  struct nlattr *na;
  INT32 rc;
  void *msg_head;
  UINT8 *mydata;
  INT16 retval = 0;
  UINT16 i = 0;
  uint16 send_len;
  SLI_STATUS Status = SLI_STATUS_SUCCESS;

#ifdef SLI_DEBUG_PRINT
#endif

  if (info == NULL) {
    goto out;
  }

  /*for each attribute there is an index in info->attrs which points to a nlattr
   * structure in this structure the data is given
   */
  na = info->attrs[SLI_USER_A_MSG];
  if (na) {
    mydata = (char *)sli_nla_data(na);
    if (mydata == NULL) {
      SLI_DEBUG(SLI_ZONE_SPI_DBG, "error while receiving data\n");
    }
  } else {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "no info->attrs %i\n", SLI_USER_A_MSG);
  }

  /* mydata points to the descriptor + payload of the command.
   * We need to send this to WSC module by writing it over SPI interface.
   */
  send_len = mydata[2] | (mydata[3] << 8);
  send_len = send_len & 0xfff;

  if (*(mydata + 1) == 0xEE) {
    Status = sli_init_interface(Adapter);

    if (Status == SLI_STATUS_FAIL) {
      SLI_DEBUG(SLI_ZONE_ERROR, "sli_probe: Failed to enable interface\n");
      return -1;
    } else {
      Adapter->irq_registered = 1;
    }

    sli_down_interruptible(&Adapter->int_check_sem);
    do {
      retval = sli_execute_cmd(mydata, mydata + 7, send_len);
    } while (retval == -3);
    sli_up_sem(&Adapter->int_check_sem);

  } else if (1) { 
    sli_down_interruptible(&Adapter->int_check_sem);
    send_len = send_len & 0xfff;
    do {
      retval = sli_execute_cmd(mydata, mydata + 7, send_len);
    } while (retval == -3);
    sli_up_sem(&Adapter->int_check_sem);
  } else {
    retval = sli_update_device(Adapter, mydata);
  }

  if (get_portid(info) == WLAN_PORT_ID) {
    Adapter->wlan_seq_num = info->snd_seq;
  } else if (get_portid(info) == ZB_PORT_ID) {
    Adapter->zb_seq_num = info->snd_seq;
  } else if (get_portid(info) == BT_PORT_ID) {
    Adapter->bt_seq_num = info->snd_seq;
  }
  return retval;

  out:
  SLI_DEBUG(SLI_ZONE_SPI_DBG,
            "an error occured in sli_read_pkt_from_userspace:\n");
  return -2;
}

/*==============================================*/
/**
 * @fn          INT32 sli_send_rsp_to_userspace(
 *                          PSLI_ADAPTER Adapter,
 *                          rspBuff)
 * @brief       Sends the response to user space over netlink
 *              socket
 * @param[in]   PSLI_ADAPTER Adapter, pointer to adapter
 * @param[in]   rspBuff, pointer to response buffer
 * @param[out]  none
 * @return      errCode
 *              -1 = FAIL
 *              0  = SUCCESS
 * @section description
 * This API is used to read the command from user over netlink
 * socket.
 */
INT32 sli_send_rsp_to_userspace(PSLI_ADAPTER Adapter, UINT8 *rspBuff)
{
  struct nlattr *na = NULL;
  struct sk_buff *skb = NULL;
  INT32 rc;
  void *msg_head = NULL;
  UINT16 rspBufLen, total_len;
  INT16 retval;
  UINT8 i = 0;
  UINT8 queue_no;
  UINT32 seq_num, snd_pid = 0;
  /* send the response to user space */
  /* allocate some memory, since the size is not yet known use NLMSG_GOODSIZE*/
  rspBufLen = rspBuff[2] | (rspBuff[3] << 8);
  rspBufLen &= 0x0FFF;

  total_len = CPC_HEADER_LENGTH + rspBufLen;
  skb = sli_genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if (skb == NULL) {
    retval = 1;
    goto out;
  }

  if (!Adapter) {
    goto out;
  }

#if KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(33)
  struct net *net = &init_net;
#else
#if KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(32)
  struct net *net = &init_net;
#endif
#endif

#if KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(32)
  if (!net) {
    goto out;
  }
#endif
  /* create the message headers */
  /* arguments of genlmsg_put:
     struct sk_buff *,
     INT32 (sending) pid,
     INT32 sequence number,
     struct genl_family *,
     INT32 flags,
     u8 command index (why do we need this?)
   */
  seq_num = Adapter->wlan_seq_num;
  snd_pid = WLAN_PORT_ID;
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
  msg_head = sli_genlmsg_put(skb, 0, seq_num + 1, sli_user_gnl_family.id, 0, 0,
                             SLI_USER_C_CMD, sli_user_gnl_family.version);
#else
  msg_head = sli_genlmsg_put(skb, 0, seq_num + 1, &sli_user_gnl_family, 0,
                             SLI_USER_C_CMD);
#endif
  if (msg_head == NULL) {
    retval = 2;
    rc = -ENOMEM;
    goto out;
  }
  /* add a SLI_USER_A_MSG attribute (actual value to be sent) */
  rc = sli_nla_put(skb, SLI_USER_A_MSG, total_len, rspBuff);
  if (rc != 0) {
    retval = 3;
    goto out;
  }
  /* finalize the message */
  sli_genlmsg_end(skb, msg_head);
#if KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(32)
  /* send the message back */
  rc = sli_genlmsg_unicast(net, skb, snd_pid);
#else
  /* send the message back */
  rc = sli_genlmsg_unicast(skb, snd_pid);
#endif
  if (rc != 0) {
    retval = 4;
    goto out;
  }
  return 0;
  out:
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG,
            "an error occured in sli_send_rsp_to_userspace : %d, rc : %d\n",
            retval, rc);
#endif
  return -1;
}

/* commands: mapping between the command enumeration and the actual function*/
struct genl_ops sli_user_gnl_ops = {
  .cmd = SLI_USER_C_CMD,
  .flags = 0,
  .policy = sli_user_genl_policy,
  .doit = sli_read_req_from_userspace,
  .dumpit = NULL,
};

/*==============================================*/
/**
 * @fn          INT32 sli_register_genl(void)
 * @brief       Registers genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to register genl ops and family.
 */
INT32 sli_register_genl(void)
{
  INT32 rc;

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_register_genl:\n");
#endif
  /*register new family*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 12, 34)
  sli_user_gnl_family.ops = &sli_user_gnl_ops;
  sli_user_gnl_family.n_ops = 1;
#endif
  rc = sli_genl_register_family(&sli_user_gnl_family);
  if (rc != 0) {
    return rc;
  }

  /*register functions (commands) of the new family*/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
  rc = sli_genl_register_ops(&sli_user_gnl_family, &sli_user_gnl_ops);
  if (rc != 0) {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "register ops: %i\n", rc);
    sli_genl_unregister_family(&sli_user_gnl_family);
    return rc;
  }
#endif
  return rc;
}

/*==============================================*/
/**
 * @fn          INT32 sli_unregister_genl(void)
 * @brief       Unregisters genl family and operations
 * @param[in]   none
 * @param[out]  none
 * @return      errCode
 *              0  = SUCCESS
 *              else FAIL
 * @section description
 * This API is used to unregister genl related ops.
 */
INT32 sli_unregister_genl(void)
{
  INT32 ret;

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_unregister_genl:\n");
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
  /*unregister the functions*/
  ret = sli_genl_unregister_ops(&sli_user_gnl_family, &sli_user_gnl_ops);
  if (ret != 0) {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "unregister ops: %i\n", ret);
    return ret;
  }
#endif

  /*unregister the family*/
  ret = sli_genl_unregister_family(&sli_user_gnl_family);
  if (ret != 0) {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "unregister family %i\n", ret);
    return ret;
  }

  return ret;
}

/* $EOF */
/* Log */
