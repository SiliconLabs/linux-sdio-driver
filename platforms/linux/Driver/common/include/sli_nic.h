/***************************************************************************//**
 * @file
 * @brief LINUX SPI driver
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

#ifndef SLI_NIC_H
#define SLI_NIC_H

#include "linux/netdevice.h"
#include <linux/version.h>
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,18)
#include <asm/semaphore.h>
#else
#include "linux/semaphore.h"
#endif
#include "sli_common.h"
#include "sli_global.h"

/* sli ioctl */
#define SLI_WSC_PRIV_IOCTLS      SIOCDEVPRIVATE
#define OID_WSC_GET_STATUS       SLI_WSC_PRIV_IOCTLS + 0x1
#define OID_WSC_BOOT_READ        SLI_WSC_PRIV_IOCTLS + 0x2
#define OID_WSC_BOOT_WRITE       SLI_WSC_PRIV_IOCTLS + 0x3
#define OID_WSC_BOOT_PING_WRITE  SLI_WSC_PRIV_IOCTLS + 0x4
#define OID_WSC_BOOT_PONG_WRITE  SLI_WSC_PRIV_IOCTLS + 0x5
#define OID_WSC_POWER_SAVE_ENABLE  SLI_WSC_PRIV_IOCTLS + 0x6
#define OID_WSC_WAKEUP             SLI_WSC_PRIV_IOCTLS + 0x7
#define OID_MASTER_READ            SLI_WSC_PRIV_IOCTLS + 0x8 

#define USB_MAX_PACKET_SIZE													2048

#define BIT(n)                                  (1 << (n))
#define MAX_SPI_PKT_SIZE                        1600

#define SLI_LITTLE_ENDIAN                       1 //! Comment this for Big Endian

#define SLI_DESC_LEN                            16
#define CPC_HEADER_LENGTH                       7
#define SLI_LENGTH_OF_ADDRESS                   6
#define MAX_BUFF_SIZE                           4900
#define SLI_HEADER_SIZE                         14

#define MGMT_QUEUE                              0
#define DATA_QUEUE                              1

#define SND_DATA_Q                              0x5

#define WLAN_PORT_ID 0x1111
#define ZB_PORT_ID   0x2222
#define BT_PORT_ID   0x3333
#ifdef SLI_UART_INTERFACE
#define UART_PORT_ID 0x1122
#endif



#define FSM_OPEN                                1
#define FSM_RESET                               2

/* Status indications */
#define BUFFER_FULL                             0x01
#define BUFFER_EMPTY                            0x02
#define RX_PKT_PENDING                          0x04 
#define POWER_SAVE                              0x08

/* Kernel version between and including a & b */
#define KERNEL_VERSION_BTWN_2_6_(a,b) \
 ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) && \
  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,b)))

#define KERNEL_VERSION_EQUALS_2_6_(a) \
 (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,a))

/* Kernel version greater than equals */
#define KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(a)  \
 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a))

/* Kernel version Less than equals */
#define KERNEL_VERSION_LESS_THAN_EQUALS_2_6_(a)  \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,a))


#define KERNEL_VERSION_BTWN_2_6_(a,b) \
 ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) && \
  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,b)))

#define KERNEL_VERSION_EQUALS_2_6_(a) \
 (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,a))

/* Kernel version greater than equals */
#define KERNEL_VERSION_GREATER_THAN_2_6_(a)  \
 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a))

#define DEVINIT                       __devinit
#define DEVEXIT                       __devexit
#define INIT                          __init
#define EXIT                          __e
/* Update types from user to kernel */
enum {
  MODULE_POWER_CYCLE               = 0x01,
  UPDATE_JOIN_DONE                 = 0x02,
  PS_CONTINUE                      = 0x03,
  WKP_FROM_HOST                    = 0x04,
  UPDATE_CONCURRENT_AP_JOIN_DONE   = 0x05,
  SOFT_RESET                       = 0X06,
  TCP_IP_BYPASS_MACRO              = 0X10,
  GPIO_POWERSAVE                   = 0x08,
  ULP_ENABLED                      = 0X09,
  WMM_SUPPORT                      = 0x11,
};

typedef struct _SLI_EVENT {
    atomic_t   eventCondition;
    wait_queue_head_t  eventQueue;
}SLI_EVENT, *PSLI_EVENT;

#ifdef SLI_USB_INTERFACE
#define USB_VENDOR_REGISTER_READ                    0x15
#define USB_VENDOR_REGISTER_WRITE                   0x16
#ifdef RS9116
#define USB_TX_HEAD_ROOM_WLAN                       256
#else
#define USB_TX_HEAD_ROOM_WLAN                       236
#endif
#define USB_TX_HEAD_ROOM_ZB                         0
#ifdef RS9116
#define USB_TX_HEAD_ROOM_BT                         128
#else
#define USB_TX_HEAD_ROOM_BT                         0
#endif
#define USB_RPINE_VENDER_ID                         0x1618
#define USB_RPINE_VENDER_ID_2                       0x41B
#ifdef RS9116
#define USB_RPINE_PRODUCT_ID                        0x9116
#else
#define USB_RPINE_PRODUCT_ID                        0x9113
#endif
#define USB_WLAN                                    0x0
#define USB_BT_ZB                                   0x1
#ifdef RS9116
#define USB_ZB_ONLY                                 0x2
#endif
#endif

typedef struct _SLI_ADAPTER
{
#ifdef SLI_SPI_INTERFACE
  struct spi_device                               *spi;
#endif
#ifdef SLI_USB_INTERFACE
  struct usb_device                               *usbdev;
  struct usb_interface                            *interface;
#ifdef RS9116
#define MAX_BULK_END_POINTS     3
#else
#define MAX_BULK_END_POINTS     2
#endif
  UINT8                                           *rx_buf[MAX_BULK_END_POINTS];
  UINT32                                          rx_len[MAX_BULK_END_POINTS];
  UINT32                                          tx_len[MAX_BULK_END_POINTS];
  UINT32                                          max_rx_len[MAX_BULK_END_POINTS];
  UINT8                                           rx_endpoint_address[MAX_BULK_END_POINTS];
  UINT8                                           tx_endpoint_address[MAX_BULK_END_POINTS];
 #ifdef RS9116
#define MAX_RX_URBS 3 
#else
#define MAX_RX_URBS 2 
#endif
  struct urb                                      *rx_usb_urb[MAX_RX_URBS];
 #ifdef RS9116
#define MAX_TX_URBS 3
#else
#define MAX_TX_URBS 2
#endif
  struct urb                                      *tx_usb_urb[MAX_TX_URBS];

#endif
  struct workqueue_struct                         *workqueue;
  struct net_device                               *net_device0;
  struct net_device                               *net_device1;
  UINT8                                            rx_buffer[MAX_BUFF_SIZE];
  volatile UINT8                                   sli_status;
  UINT32                                           FSM_STATE;
  UINT32                                           FSM_STATE_AP1;
  struct _SLI_EVENT                                Event;
#ifdef SLI_UART_INTERFACE
  struct _SLI_EVENT                                DATA_ACKEvent;
  uint8                                           data_ack_missed;
#endif
  struct _SLI_EVENT                                PwrSaveEvent;
  INT32                                            txPkt_count;
  volatile UINT8                                   BufferFull;
  UINT8                                            bufferfull_indicated;
  sli_powerstate                                   sli_pwstate;
  struct net_device_stats                         stats;
  struct sk_buff_head                             list[4];
  struct work_struct                              handler;
  spinlock_t                                      lockqueue;
  spinlock_t                                      locks[4];
#ifdef SLI_UART_INTERFACE
  spinlock_t                                     uart_data_ack_flag_sem;
#endif
  UINT8                                           PermanentAddress[SLI_LENGTH_OF_ADDRESS];
  UINT32                                          PktLen; /*Transmit pkt length*/
  UINT8                                           DataRcvPacket[MAX_BUFF_SIZE];
  UINT8                                           DataSndPacket[MAX_BUFF_SIZE];
  struct semaphore                                int_check_sem;
  struct semaphore                                sleep_ack_sem;
  struct semaphore                                tx_data_sem;
  UINT32                                          RcvPktLen;
  UINT32                                          InterruptStatus;
  UINT32                                          handler_intr_cnt;
  UINT32                                          int_ctr;
  UINT32                                          isr_intr_cnt;
  UINT32                                          once_full;
  UINT32                                          once_empty;
  UINT32                                          pdca_int_cnt;
  UINT8                                           qos_enable;
  UINT8                                           about_to_send;
  UINT8                                           halt_flag;
  volatile INT32                                  total_pkts_qd;
  UINT16                                          save_flsh_status;
  UINT32                                          wlan_seq_num;
  UINT32                                          zb_seq_num;
  UINT32                                          bt_seq_num;
#ifdef SLI_UART_INTERFACE
  UINT32                                          uart_seq_num;
#endif
  INT32                                           txThreadId;
  int                                             transmit_thread_exit;
  struct completion                               txThreadComplete;
  uint8                                           irq_registered;
  uint8                                           power_save_enable;

#ifdef ENABLE_WMM_FEATURE
  /* WMM specific variables */
	int32                                             per_q_wt[NUM_EDCA_QUEUES];
	int32                                             wme_org_q[NUM_EDCA_QUEUES];
	int32                                             pkt_contended[NUM_EDCA_QUEUES];
 	//global varibles from onebox_wmm.c
	uint8                                           selected_qnum;
	uint8                                           pkt_cnt;
	uint8                                           tid;
	uint8                                           priority;
  struct chanAccParams                            wme_wmeChanParams;
  uint8                                           dbg_test_values_loaded;
#endif
#ifdef  RSI_AUTOMATION_ENABLE
  uint8                                           tcp_ip_bypass;
#endif
#if SLI_SDIO_INTERFACE
 #if ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
   (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
  PSDDEVICE pDevice;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
  struct  sdio_func *pfunction;
  uint32                                          TransmitBlockSize;
  uint32                                          ReceiveBlockSize;
#endif
  uint8                                           next_read_delay;
  uint8                                           sdio_high_speed_enable;
  uint8                                           stop_card_writing;
  uint8                                           io_enable;
  uint8                                           io_ready;
  uint8                                           int_enable;
  uint8                                           bus_interface_control;
#endif
} SLI_ADAPTER, *PSLI_ADAPTER;

typedef struct _SLI_NET_DEV
{
  PSLI_ADAPTER       sli_adapter_ptr;
} SLI_NET_DEV, *PSLI_NET_DEV;


#define SLI_MAX_PKTS_QUEUED  6400





#endif  //SLI_NIC_H
