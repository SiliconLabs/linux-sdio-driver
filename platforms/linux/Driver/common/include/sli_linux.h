/***************************************************************************//**
 * @file
 * @brief Operating system specific definitions can be found here
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

#ifndef SLI_LINUX_H
#define SLI_LINUX_H_
#include "sli_common.h"
#ifdef SLI_SDIO_INTERFACE
#include "sli_sdio.h"
#endif

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

typedef struct _SLI_ADAPTER *PSLI_ADAPTER;

#define BIT(n) (1 << (n))
#define TEXT(arg...) arg
#define SLI_DEBUG(zone, fmt, arg...) if(zone & sli_zone_enabled) printk(fmt, ##arg)
#define SLI_ZONE_INFO           BIT(0)
#define SLI_ZONE_ERROR          BIT(1)
#define SLI_ZONE_INIT           BIT(2)
#define SLI_ZONE_OID            BIT(3)
#define SLI_ZONE_MGMT_SEND      BIT(4)
#define SLI_ZONE_MGMT_RCV       BIT(5)
#define SLI_ZONE_DATA_SEND      BIT(6)
#define SLI_ZONE_DATA_RCV       BIT(7)
#define SLI_ZONE_FSM            BIT(8)
#define SLI_ZONE_ISR            BIT(9)
#define SLI_ZONE_MGMT_DUMP      BIT(10)
#define SLI_ZONE_DATA_DUMP      BIT(11)
#define SLI_ZONE_PARSER         BIT(12)
#define SLI_ZONE_SPI_DBG        BIT(13)
#ifdef RS9116
#define SLI_ZONE_RS9116_DBG     BIT(14)
#endif
#define SLI_ZONE_SDIO_DBG       BIT(15)


#define sli_carrier_on(dev)        netif_carrier_on(dev)
#define sli_carrier_off(dev)       netif_carrier_off(dev)
#define sli_vmalloc(temp)          vmalloc(temp)
#define sli_vfree(ptr)             vfree(ptr)

#define sli_queue_init(a)          skb_queue_head_init(a)
#define sli_queue_purge(a)         skb_queue_purge(a)
#define sli_queue_tail(a,b)        skb_queue_tail(a,b)
#define sli_queue_head(a,b)        skb_queue_head(a,b)
#define sli_queue_len(a)           skb_queue_len(a)
#define sli_dequeue(a)             skb_dequeue(a)
#define sli_skb_put(a,b)           skb_put(a,b)
#define sli_kfree_skb(a)           dev_kfree_skb(a)

#define sli_spinlock_init(a)       spin_lock_init(a)
#define sli_lock_bh(a)             spin_lock_bh(a)
#define sli_unlock_bh(a)           spin_unlock_bh(a)

#define sli_memcpy(a,b,c)          memcpy(a,b,c)
#define sli_memset(a,b,c)          memset(a,b,c)
#define sli_memcmp(a,b,c)          memcmp(a,b,c)
#define sli_strcpy(a,b)            strcpy(a,b)
#define sli_strcpy(a,b)            strcpy(a,b)
#define sli_strncpy(a,b,c)         strncpy(a,b,c)
#define sli_equal_string(a,b)      strcmp(a,b)
#define sli_simple_strtol(a,b,c)   simple_strtol(a,b,c)

#define sli_Signal_Pending()       signal_pending(current)
#define sli_netif_rx(a)            netif_rx_ni(a)
#define sli_netif_start_queue(a)   netif_start_queue(a)
#define sli_netif_queue_stopped(a) netif_queue_stopped(a)
#define sli_netif_stop_queue(a)    netif_stop_queue(a)
#define sli_eth_type_trans(a,b)    eth_type_trans(a,b)

#define sli_schedule()             schedule()
#define sli_sprintf                sprintf

#define sli_down_interruptible(a)  down_interruptible(a)
#define sli_up_sem(a)              up(a)
#define sli_down_sem(a)            down(a)
#ifdef init_MUTEX
#define sli_init_mutex(a)          init_MUTEX(a)
#else
#define sli_sema_init(a,b)         sema_init(a,b)
#endif

#define sli_mem_free(ptr)          kfree(ptr)
#define sli_mem_alloc(a,b)         kmalloc(a,b)

#define sli_copy_to_user(a,b,c)    copy_to_user(a,b,c)
#define sli_copy_from_user(a,b,c)  copy_from_user(a,b,c)

#define sli_nla_data(a)                 nla_data(a)
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,18)
#define sli_genlmsg_new(a,b)            nlmsg_new(a)
#else
#define sli_genlmsg_new(a,b)            genlmsg_new(a,b)
#endif
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,18)
#define sli_genlmsg_put(a,b,c,d,e,f,g,h) genlmsg_put(a,b,c,d,e,f,g,h)
#else
#define sli_genlmsg_put(a,b,c,d,e,f)    genlmsg_put(a,b,c,d,e,f)
#endif
#define sli_nla_put(a,b,c,d)            nla_put(a,b,c,d)
#define sli_genlmsg_end(a,b)            genlmsg_end(a,b)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#define sli_genlmsg_unicast(a,b,c)      genlmsg_unicast(a,b,c)
#else
#define sli_genlmsg_unicast(a,b)        genlmsg_unicast(a,b)
#endif
#define sli_genl_register_ops(a,b)      genl_register_ops(a,b)
#define sli_genl_unregister_ops(a,b)    genl_unregister_ops(a,b)
#define sli_genl_register_family(a)     genl_register_family(a)
#define sli_genl_unregister_family(a)   genl_unregister_family(a)

#define EVENT_WAIT_FOREVER       (0x00)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 11)
# define get_portid(_info) (_info)->snd_pid
#else
# define get_portid(_info) (_info)->snd_portid
#endif

#define BIN_FILE 0
#define HEX_FILE 1

#define SLI_THREAD_NAME_LEN      15
#define SLI_THREAD_PRIORITY      0
#define SLI_INT_THREAD_PRIORITY  0

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
typedef  struct work_struct         SLI_WORK_QUEUE;
#define  sli_Init_Work_Queue(a,b)  INIT_WORK(a,b)
#define  sli_Schedule_Work(a)      schedule_work(a)
#else
typedef  struct tq_struct           SLI_WORK_QUEUE;
#define  sli_Init_Work_Queue(a,b,c)  INIT_TQUEUE(a,b,c)
#define  sli_Schedule_Work(a)        schedule_task(a)
#endif
#define SLI_THREAD_NAME_LEN 15

#define SLI_ASSERT(exp)                                                     \
    do {                                                                    \
        if (!(exp)) {                                                       \
            SLI_DEBUG(SLI_ZONE_ERROR,"Assertion Failed! %s, %s(), %s:%d\n", \
                    #exp, __FUNCTION__, __FILE__, __LINE__);                \
        }                                                                   \
    } while (0)

typedef struct semaphore SLI_SEMAPHORE, *PRSI_SEMAPHORE;


/*Function prototypes*/
/* net device */
struct net_device* sli_allocdev(INT32 sizeof_priv, UINT8 vap_id);
INT32  sli_registerdev( struct net_device *dev );
VOID   sli_unregisterdev(struct net_device *dev);
struct net_device_stats *sli_get_stats( struct net_device *dev);
INT32  sli_open(struct net_device *dev);
INT32  sli_stop(struct net_device *dev);
struct net_device* sli_netdevice_op(UINT8 vap_id);

INT32  sli_xmit(struct sk_buff *skb, struct net_device *dev);
INT32 sli_transmit_thread (PVOID pContext);
INT32  sli_indicate_packet(PSLI_ADAPTER Adapter, UINT8 *DataRcvPacket,UINT32 pktLen, UINT8 sta_id);

struct sk_buff *sli_alloc_skb(UINT32 len);
VOID   sli_kfree_skb(struct sk_buff *skb);
VOID   sli_dump(INT32 zone, PVOID vdata, INT32   len);
INT32  sli_Kill_Thread( PSLI_ADAPTER Adapter);
INT32 sli_Init_Thread ( PSLI_ADAPTER Adapter);
INT32 sli_Init_Event(SLI_EVENT *pEvent);
INT32 sli_Delete_Event (SLI_EVENT *pEvent);
INT32  sli_Wait_Event(SLI_EVENT *pEvent,UINT32 timeOut);
VOID sli_Set_Event(SLI_EVENT *pEvent);
INT32 sli_Reset_Event(SLI_EVENT *pEvent);
PSLI_ADAPTER sli_getpriv(struct net_device *glbl_net_device);
  


/* netlink */
#ifdef SLI_UART_INTERFACE
INT32 sli_send_rsp_to_userspace(PSLI_ADAPTER Adapter, UINT8 *rspBuff, UINT8 to_uart);
#else
INT32 sli_send_rsp_to_userspace(PSLI_ADAPTER Adapter, UINT8 *rspBuff);
#endif
INT32 sli_register_genl(void);
INT32 sli_unregister_genl(void);
#endif //SLI_LINUX_H_
