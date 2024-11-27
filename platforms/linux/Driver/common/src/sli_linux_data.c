/***************************************************************************//**
 * @file
 * @brief File contains functions related to data tranfer over net device.
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
#include "sli_linux.h"
#include "sli_nic.h"
#include "sli_api.h"
#include <linux/sched.h>

/*==============================================*/
/**
 * @fn          INT32 sli_xmit (struct sk_buff *skb,
 *                           struct net_device *dev)
 * @brief       To transmit data
 * @param[in]   struct sk_buff *skb,
 *              Pointer to the socket buffer structure
 * @param[in]   struct net_device *dev,
 *              Pointer to our network device structure
 * @param[out]  none
 * @return      errCode
 *              -ve = FAIL
 *              0  = SUCCESS
 * @section description
 * When a packet is sent by the upper layers, the transmit entry
 * point registered at the time of initialization will be called.
 * This function will que the skb and set the event.
 */
#ifdef ENABLE_WMM_FEATURE
void onebox_set_contention_vals(struct chanAccParams wme_wmeChanParams, PSLI_ADAPTER adapter);
uint8 core_determine_hal_queue(PSLI_ADAPTER adapter);
#endif

INT32 sli_xmit (struct sk_buff *skb, struct net_device *dev)
{
  PSLI_ADAPTER Adapter = sli_getpriv (dev);
#ifdef ENABLE_WMM_FEATURE
 	UINT16 pkt_priority = BE_Q_STA;
	UINT8  tos;
#endif
  UINT32 qId;

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG,"\nsli_xmit:\n");
#endif
  SLI_DEBUG (SLI_ZONE_DATA_SEND, "+sli_xmit\n");

  /*Checking if data pkt is rcvd in open state or not */
  if (Adapter->FSM_STATE != FSM_OPEN)
  {
    goto fail;
  }

  /* Checking the size of the packet */
  if (skb->len > 1528)
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_xmit: Big packet ");
    goto fail;
  }

#ifdef ENABLE_WMM_FEATURE

  if(!Adapter->dbg_test_values_loaded)
  {
    Adapter->dbg_test_values_loaded = 1;
    onebox_set_contention_vals(Adapter->wme_wmeChanParams,Adapter);
  }

  if (Adapter->total_pkts_qd > SLI_MAX_PKTS_QUEUED){
    SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_xmit: failed due to total pkt qd crossed SLI_MAX_PKTS_QUEUED\n");

  }

#ifdef SLI_WMM_PS_SUPPORT
  /* Incase of device encap pick the tos from the IP hdr */
  tos = skb->data[15] >> 5;
  Adapter->tid = tos; 
  Adapter->priority = TID_TO_WME_AC(tos);
#else
  Adapter->priority = 1;
#endif
  switch (Adapter->priority) 
  {
    case WME_AC_VO:
      pkt_priority = VO_Q_STA;
      break;
    case WME_AC_VI:
      pkt_priority = VI_Q_STA;
      break;
    case WME_AC_BE:
      pkt_priority = BE_Q_STA;
      break;
    case WME_AC_BK:
      pkt_priority = BK_Q_STA;
      break;
    default:
      /* Should not come here */
      pkt_priority = BE_Q_STA;
  }
  qId = pkt_priority;
  SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_xmit: qId of packet %d and tid %d\n", qId,Adapter->tid);

  sli_lock_bh(&Adapter->lockqueue);
  Adapter->total_pkts_qd++;
  sli_unlock_bh(&Adapter->lockqueue);

  /*Acquiring the lock on the skb queues */
  sli_queue_tail (&Adapter->list[qId], skb);


  sli_Set_Event(&Adapter->Event);

  SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_xmit: Signalled\n");
  return NETDEV_TX_OK;
#else
  qId = DATA_QUEUE;

  if (Adapter->total_pkts_qd > SLI_MAX_PKTS_QUEUED)
  {
    {

      sli_Set_Event(&Adapter->Event);
    }

    goto fail;   
  }

  sli_lock_bh(&Adapter->lockqueue);
  Adapter->total_pkts_qd++;
  sli_unlock_bh(&Adapter->lockqueue);

  /*Acquiring the lock on the skb queues */
  sli_queue_tail (&Adapter->list[qId - 1], skb);
  {
    
    sli_Set_Event(&Adapter->Event);
  }
  SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_xmit: Signalled\n");
  return NETDEV_TX_OK;
#endif
fail:
  SLI_DEBUG (SLI_ZONE_DATA_SEND, "-Dropped\n");
  Adapter->stats.tx_dropped++;
  sli_kfree_skb (skb);
  return 0;
}


/*==============================================*/
/**
 * @fn          VOID sli_transmit_thread (PVOID pContext)
 * @brief       Tranmit thread to write data onto card
 * @param[in]   PVOID pContext
 * @param[out]  none
 * @return      none
 * @section description
 * This thread dequeues the skb from the list and adds the
 * descriptor. Finally, it writes onto the card.
 */
INT32 sli_transmit_thread (PVOID pContext)
{
  PSLI_ADAPTER Adapter = (PSLI_ADAPTER) pContext;
  UINT32 Len = 0, i;
  UINT32 q_num, que_end;
  UINT32 que_start = 0;
  INT32 status;
  UINT8 cmd_qnum = SND_DATA_Q;
  UINT8 desc[SLI_DESC_LEN];
  UINT8           int_status = 0;
  UINT32 num_pkts = 0;

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG,"\nsli_transmit_thread:\n");
#endif
  SLI_DEBUG (SLI_ZONE_DATA_SEND,
             "sli_transmit_thread: XMIT thread started\n");

  set_user_nice(current,-5);
  
  #if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 11)
  daemonize("XMIT-Thread");
  #endif

#ifdef ENABLE_WMM_FEATURE 
  do
  {
    UINT8 InterruptStatus = 0;
    UINT8 tid = 0;
    struct sk_buff *rcvskb;
    struct sk_buff *rcvskb_free;


    if (Adapter->BufferFull)
    {
      status = sli_Wait_Event (&Adapter->Event, 10);
    }
    else
    {
      status = sli_Wait_Event (&Adapter->Event, EVENT_WAIT_FOREVER);
    }

    sli_Reset_Event (&Adapter->Event); 
    while(1)
    {
      q_num = core_determine_hal_queue(Adapter);
      if (Adapter->FSM_STATE != FSM_OPEN)
      {
        SLI_DEBUG(SLI_ZONE_DATA_SEND,(TEXT("transmit_thread: FSM is not in OPEN State\n")));
        break;
      }

      if (q_num == INVALID_QUEUE) 
      {
        SLI_DEBUG(SLI_ZONE_DATA_SEND,TEXT("\nqos_pro: No More Pkt Due to invalid queueno\n"));
        break;
      }
      sli_lock_bh(&Adapter->lockqueue);
      Adapter->total_pkts_qd--;
      sli_unlock_bh(&Adapter->lockqueue);

      rcvskb = sli_dequeue (&Adapter->list[q_num]);

      SLI_DEBUG (SLI_ZONE_DATA_SEND, "Deque frm %d\n", q_num);
      if (rcvskb == NULL)
      {
        SLI_DEBUG (SLI_ZONE_ERROR,
            "sli_transmit_thread: ERROR!!!! skb NULL %d\n",
            q_num);
        SLI_DEBUG(SLI_ZONE_DATA_SEND,(TEXT("No packet queued and trying to deque packet\n")));
        break;
      }
      
      tid = WME_AC_TO_TID(q_num);
      sli_memset (desc, 0, 16);
#ifdef SLI_LITTLE_ENDIAN
      *(UINT16 *)&desc[0] = (((rcvskb->len) | (SND_DATA_Q << 12)) & 0x7fff);
#else
      desc[0] = (UINT8)((UINT16)((rcvskb->len ) & 0x00ff));
      desc[1] = (UINT8)((UINT16)((((rcvskb->len )| (SND_DATA_Q << 4)) >> 8) & 0x00ff));
#endif
      /*Keep Tid at 14th byte upper nibble*/
      desc[14] |= (((tid & 0xf) << 4) | (q_num & 0xf));
      Len = rcvskb->len;

      SLI_DEBUG (SLI_ZONE_DATA_SEND, "Queue %d and Tid of the queue %d\n", q_num,tid);
      //need to send the data from application
      Adapter->txPkt_count++;
      do
      {
        //need to send the data from application
        sli_down_interruptible(&Adapter->int_check_sem);
        status = sli_execute_cmd(desc, rcvskb->data, Len);

        sli_up_sem(&Adapter->int_check_sem);
        if (status == -2)
        {
          SLI_DEBUG (SLI_ZONE_ERROR, "\nstatus : %d\n",status);
          SLI_DEBUG (SLI_ZONE_ERROR,
              "trying data %d bytes, Pkt_Count:%d\n",
              Len, Adapter->txPkt_count);

        }
        if (status == 0)
        {
          break;
        }
      }while(1);

      if(status == -1)
        break;
      if (status != SLI_STATUS_SUCCESS)
      {
#ifdef SLI_DEBUG_PRINT
        SLI_DEBUG (SLI_ZONE_ERROR,
                  "desc Pkt Count: %d\n",
                  Adapter->txPkt_count);
#endif
        sli_kfree_skb (rcvskb);
        Adapter->stats.tx_dropped++;
        break;
      }


      SLI_DEBUG (SLI_ZONE_DATA_SEND,
          "sli_transmit_thread: Pkt of %d bytes written on card\n",
          Len);
      sli_kfree_skb (rcvskb);
      Adapter->stats.tx_packets++;
      Adapter->stats.tx_bytes += Len;

      if(Adapter->transmit_thread_exit)
      {
        sli_lock_bh(&Adapter->lockqueue);
        while(Adapter->total_pkts_qd > 0)
        {
          rcvskb_free= sli_dequeue (&Adapter->list[q_num]);
          Adapter->total_pkts_qd--;
          sli_kfree_skb (rcvskb_free);
        }
        sli_unlock_bh(&Adapter->lockqueue);
        break;
      }
    }
  } while (Adapter->transmit_thread_exit == 0);
#else

  do
  {
    UINT8 InterruptStatus = 0;
    struct sk_buff *rcvskb;
    struct sk_buff *rcvskb_free;
    if (Adapter->BufferFull)
    {
      status = sli_Wait_Event (&Adapter->Event, 10);
    }
    else
    {
      status = sli_Wait_Event (&Adapter->Event, EVENT_WAIT_FOREVER);
    }

    sli_Reset_Event (&Adapter->Event); 

    SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_transmit_thread:event released\n");
	que_end = 1;
      if (Adapter->transmit_thread_exit )
    {
      SLI_DEBUG (SLI_ZONE_DATA_SEND,
                 "sli_transmit_thread: Halt flag set\n");
      break;
    }
  que_loop:
    for (q_num = que_start; q_num < que_end; ++q_num)
    {
      UINT32 pkts_given = 0;
      SLI_DEBUG (SLI_ZONE_DATA_SEND, "sli_transmit_thread: Q_NUM:%d\n",
                 q_num);

      if (Adapter->FSM_STATE != FSM_OPEN)
      {
        SLI_DEBUG(SLI_ZONE_DATA_SEND,(TEXT("ganges_transmit_thread: FSM is not in OPEN State\n")));
        break;
      }

      do
      {

        ++pkts_given;

        sli_lock_bh(&Adapter->lockqueue);

        if (Adapter->total_pkts_qd > 0)
        {
          Adapter->total_pkts_qd--;
        }
        else 
        {
          sli_unlock_bh(&Adapter->lockqueue);
          break;
        }
        sli_unlock_bh(&Adapter->lockqueue);
        rcvskb = sli_dequeue (&Adapter->list[q_num]);

        SLI_DEBUG (SLI_ZONE_DATA_SEND, "Deque frm%d\n", q_num);
        if (rcvskb == NULL)
        {
          SLI_DEBUG (SLI_ZONE_ERROR,
                     "sli_transmit_thread: ERROR!!!! skb NULL %d\n",
                     q_num);
          break;
        }

        sli_memset (desc, 0, 16);
        sli_uint16_to_2bytes(desc, (((rcvskb->len) & 0xFFF) | (SND_DATA_Q << 12)));
        Len = rcvskb->len;

        //need to send the data from application
        Adapter->txPkt_count++;
        do
        {
          //need to send the data from application
          sli_down_interruptible(&Adapter->int_check_sem);
          status = sli_execute_cmd(desc, rcvskb->data, Len);
          sli_up_sem(&Adapter->int_check_sem);

          if (status == -2)
          {
            SLI_DEBUG (SLI_ZONE_ERROR, "\nstatus : %d\n",status);
            SLI_DEBUG (SLI_ZONE_ERROR,
                "trying data %d bytes, Pkt_Count:%d\n",
                Len, Adapter->txPkt_count);

          }
          if (status == 0)
          {
            break;
          }
        } while (1); 
        if(status == -1)
          break;
        if (status != SLI_STATUS_SUCCESS)
        {
#ifdef SLI_DEBUG_PRINT
          SLI_DEBUG (SLI_ZONE_ERROR,
                    "desc Pkt Count: %d\n",
                    Adapter->txPkt_count);
#endif
          sli_kfree_skb (rcvskb);
          Adapter->stats.tx_dropped++;
          break;
        }


        SLI_DEBUG (SLI_ZONE_DATA_SEND,
            "sli_transmit_thread: Pkt of %d bytes written on card\n",
            Len);
        sli_kfree_skb (rcvskb);
        Adapter->stats.tx_packets++;
        Adapter->stats.tx_bytes += Len;
        if(Adapter->transmit_thread_exit)
        {
          sli_lock_bh(&Adapter->lockqueue);
          while(Adapter->total_pkts_qd > 0)
          {
            rcvskb_free= sli_dequeue (&Adapter->list[q_num]);
            Adapter->total_pkts_qd--;
            sli_kfree_skb (rcvskb_free);
          }
          sli_unlock_bh(&Adapter->lockqueue);
          break;
        }       
      } while (Adapter->total_pkts_qd > 0);
    }
  } while (Adapter->transmit_thread_exit == 0);

#endif

  complete_and_exit(&Adapter->txThreadComplete, 0);

}

/*==============================================*/
/**
 * @fn          INT32 sli_indicate_packet(PSLI_ADAPTER Adapter,
 *                                     UINT8 * DataRcvPacket,
 *                                     UINT32 pktLen)
 * @brief       Indicated the packet to upper layer
 * @param[in]   PSLI_ADAPTER Adapter,
 *              Pointer to the private data of the device
 * @param[in]   UINT8 *DataRcvPacket, Pointer to a packet
 * @param[in]   UINT32 pktLen, Length of the packet
 * @param[out]  none
 * @return      On success 0, else a negative number
 * @section description
 * This function copies the packet into a skb structure and
 * indicates to the upper layer
 */
INT32 sli_indicate_packet(PSLI_ADAPTER Adapter, UINT8 * DataRcvPacket, UINT32 pktLen, UINT8 sta_id)
{
  struct sk_buff *rxskb;
  INT32 i = 0;

#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG,"\nsli_indicate_packet:\n");
#endif
#if SLI_CONCURRENT_MODE
  if ((sta_id == 0) && (Adapter->FSM_STATE != FSM_OPEN))
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_indicate_packet: Station is not in open state\n");
    goto fail;
  }
  else if ((sta_id != 0) && (Adapter->FSM_STATE_AP1 != FSM_OPEN))
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_indicate_packet: AP1 is not in open state\n");
    goto fail;
  }

#else
  if (Adapter->FSM_STATE != FSM_OPEN)
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_indicate_packet: Not in open state\n");
    goto fail;
  }
#endif

  if (pktLen > 1536)
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_indicate_packet: Big packet revd\n");
    goto fail;
  }

  if (pktLen < SLI_HEADER_SIZE)
  {
    SLI_DEBUG (SLI_ZONE_ERROR, "sli_indicate_packet: Runt packet recvd\n");
    goto fail;
  }
  /*Allocate skb and copy the data into it */
  rxskb = sli_alloc_skb (pktLen);
  if (!rxskb)
  {
    SLI_DEBUG (SLI_ZONE_ERROR,
               "sli_indicate_packet: Low on memory, packet dropped\n");
    goto fail;
  }

  sli_memcpy (sli_skb_put (rxskb, pktLen), DataRcvPacket, pktLen);
  /*Filling the various other fields of skb */
#if SLI_CONCURRENT_MODE
  if(sta_id == 0)
  {
   rxskb->dev = Adapter->net_device0;
   rxskb->protocol = sli_eth_type_trans (rxskb, Adapter->net_device0);
  }
  else
  {
    //! These station belongs to VAP 1 (which is for AP1)
   rxskb->dev = Adapter->net_device1;
   rxskb->protocol = rsi_eth_type_trans (rxskb, Adapter->net_device1);
  }
#else
   rxskb->dev = Adapter->net_device0;
   rxskb->protocol = sli_eth_type_trans (rxskb, Adapter->net_device0);
#endif

  rxskb->ip_summed = CHECKSUM_NONE;

  sli_netif_rx (rxskb);

  SLI_DEBUG (SLI_ZONE_DATA_RCV,
             "sli_indicate_packet: Pkt of %d bytes indicated\n", pktLen);

  Adapter->stats.rx_packets++;
  Adapter->stats.rx_bytes += pktLen;
  return 0;
fail:
  Adapter->stats.rx_dropped++;
  return -1;
}

/* $EOF */
/* Log */
