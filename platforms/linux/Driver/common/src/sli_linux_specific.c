/***************************************************************************/ /**
 * @file
 * @brief This contains all the linux specific code
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

#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <net/genetlink.h>

#include "sli_linux.h"
#include "sli_nic.h"
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
#include <linux/kthread.h>
#endif

/* Globals */
UINT32 sli_zone_enabled = SLI_ZONE_ERROR | SLI_ZONE_INIT | SLI_ZONE_SPI_DBG
#ifdef RS9116
                          | SLI_ZONE_RS9116_DBG
#endif
                          | SLI_ZONE_SDIO_DBG;
/*==============================================*/
/**
 * @fn          struct sk_buff* sli_alloc_skb(UINT32 Len)
 * @brief       allocates skb
 * @param[in]   UINT32 Len, length to be allocated
 * @param[out]  none
 * @return      struct sk_buff*, pointer to allocated socket buffer
 * @section description
 * This is used to allocate skb.
 */
struct sk_buff *sli_alloc_skb(UINT32 Len)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_alloc_skb:\n");
#endif
  return dev_alloc_skb(Len + 2); /*FIXME - CHECK DIS*/
}

/*==============================================*/
/**
 * @fn          VOID sli_dump(INT32 zone, PVOID vdata, INT32 len)
 * @brief       Dumps the data through debugger
 * @param[in]   INT32 zone, Pointer to Adapter
 * @param[in]   PVOID vdata, Data to dump
 * @param[in]   INT32 len, length of the data to dump
 * @param[out]  none
 * @return      none
 * @section description
 * This function dumps the given data through the debugger
 */
VOID sli_dump(INT32 zone, PVOID vdata, INT32 len)
{
  INT32 ii;
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_dump\n");
#endif
  UINT8 *data = vdata;
  for (ii = 0; ii < len; ii++) {
    if (!(ii % 16)) {
      SLI_DEBUG(zone, "\n");
    }
    SLI_DEBUG(zone, "%02x ", data[ii]);
  }
  SLI_DEBUG(zone, "\n");
}

/*==============================================*/
/**
 * @fn          INT32 sli_Init_Event(SLI_EVENT *pEvent)
 * @brief       Initializes event
 * @param[in]   SLI_EVENT *pEvent, Pointer to event
 * @return      0
 * @section description
 * This function initializes event.
 */
INT32 sli_Init_Event(SLI_EVENT *pEvent)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Init_Event:\n");
#endif
  SLI_DEBUG(SLI_ZONE_INFO, (TEXT("Initialising the event %p addr\n"), pEvent));

  atomic_set(&pEvent->eventCondition, 1);
  init_waitqueue_head(&pEvent->eventQueue);
  return 0;
}

/*==============================================*/
/**
 * @fn          INT32 sli_Delete_Event(SLI_EVENT *pEvent)
 * @brief       Deletes event
 * @param[in]   SLI_EVENT *pEvent, Pointer to event
 * @return      0
 * @section description
 * This function Deletes event.
 */
INT32 sli_Delete_Event(SLI_EVENT *pEvent)
{
  /**Dummy for Linux*/
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Delete_Event:\n");
#endif
  return 0;
}

/*==============================================*/
/**
 * @fn          INT32 sli_Wait_Event(SLI_EVENT *pEvent, UINT32 timeOut)
 * @brief       Wait event
 * @param[in]   SLI_EVENT *pEvent, Pointer to event
 * @param[in]   UINT32 timeOut, Wait timeout
 * @return      0
 * @section description
 * This function handles the wait event functionality.
 */
INT32 sli_Wait_Event(SLI_EVENT *pEvent, UINT32 timeOut)
{
  INT32 Status = 0;
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Wait_Event:\n");
#endif
  if (!timeOut) {
    wait_event_interruptible(pEvent->eventQueue,
                             (atomic_read(&pEvent->eventCondition) == 0));
  } else {
    Status = wait_event_interruptible_timeout(
      pEvent->eventQueue, (atomic_read(&pEvent->eventCondition) == 0),
      timeOut);
  } /* End if <condition> */

  return Status;
}

/*==============================================*/
/**
 * @fn          INT32 sli_Set_Event(SLI_EVENT *pEvent)
 * @brief       Set event
 * @param[in]   SLI_EVENT *pEvent, Pointer to event
 * @return      none
 * @section description
 * This function handles the set event functionality.
 */
VOID sli_Set_Event(SLI_EVENT *pEvent)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Set_Event:\n");
#endif
  atomic_set(&pEvent->eventCondition, 0);
  wake_up_interruptible(&pEvent->eventQueue);
}

/*==============================================*/
/**
 * @fn          INT32 sli_Reset_Event(SLI_EVENT *pEvent)
 * @brief       Reset event
 * @param[in]   SLI_EVENT *pEvent, Pointer to event
 * @return      VOID
 * @section description
 * This function handles the reset event functionality.
 */
INT32 sli_Reset_Event(SLI_EVENT *pEvent)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Reset_Event:\n");
#endif
  atomic_set(&pEvent->eventCondition, 1);
}

/*==============================================*/
/**
 * @fn          INT32 sli_Kill_Thread(PSLI_ADAPTER Adapter)
 * @brief       Kills the thread
 * @param[in]   PSLI_ADAPTER Adapter, Pointer to Adapter
 * @return      0
 * @section description
 * This function kills the thread
 */
INT32 sli_Kill_Thread(PSLI_ADAPTER Adapter)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Kill_Thread:\n");
#endif
  Adapter->transmit_thread_exit = 1;
  sli_Set_Event(&Adapter->Event);
  wait_for_completion(&Adapter->txThreadComplete);
  Adapter->txThreadId = 0;
  return 0;
}

/*==============================================*/
/**
 * @fn          INT32 sli_Init_Thread(PSLI_ADAPTER Adapter)
 * @brief       Initializes thread
 * @param[in]   PSLI_ADAPTER Adapter, Pointer to Adapter
 * @return      0 on success, else failure
 * @section description
 * This function initializes event.
 */
uint8 name_p[20] = "XMIT-Thread";
INT32 sli_Init_Thread(PSLI_ADAPTER Adapter)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_Init_Thread:\n");
#endif
  INT32 (*function)(PVOID pContext) = NULL;

  function = sli_transmit_thread;

  Adapter->txThreadId = 0;
  Adapter->transmit_thread_exit = 0;
  init_completion(&Adapter->txThreadComplete);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
  Adapter->txThreadId =
    kernel_thread(function, Adapter, CLONE_FS | CLONE_FILES);
#else
  Adapter->txThreadId = (int)kthread_run(function, Adapter, name_p);
#endif

  if (Adapter->txThreadId < 0) {
    SLI_DEBUG(SLI_ZONE_ERROR, (TEXT("sli_probe: Unable to initialize thrd\n")));
    return SLI_STATUS_FAIL;
  } else {
    return SLI_STATUS_SUCCESS;
  } /* End if <condition> */
}

/* $EOF */
/* Log */
