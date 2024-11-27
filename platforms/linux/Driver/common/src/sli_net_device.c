/***************************************************************************/ /**
 * @file
 * @brief This contains all the LINUX network device specific code
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <net/genetlink.h>
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif
#include "sli_api.h"
#include "sli_global.h"
#include "sli_linux.h"
#include "sli_nic.h"
#ifdef SLI_SPI_INTERFACE
#include <linux/irq.h>
#include <linux/spi/spi.h>
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18))
#include <asm/arch/gpio.h>
#include <linux/hardirq.h>
#include <linux/irqreturn.h>
#else
#include <linux/gpio.h>
#endif
#include "sli_hal.h"
#endif
#include "sli_sdio.h"

#include "sli_api.h"
#include "sli_global.h"
#include "sli_linux.h"
#include "sli_nic.h"

/* globals */
struct net_device *glbl_net_device = NULL;
PSLI_ADAPTER Adapter = NULL;

#ifdef ENABLE_WMM_FEATURE
void dbg_test_values(PSLI_ADAPTER adapter);
#endif

VOID sli_interrupt_handler(struct work_struct *work);

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
static SD_PNP_INFO sli_IdTable[] __devinitdata = {
  { .SDIO_ManufacturerID = 0x101,
    .SDIO_ManufacturerCode = 0x041b,
    .SDIO_FunctionNo = 1,
    .SDIO_FunctionClass = 0 },
  { .SDIO_ManufacturerID = 0x201,
    .SDIO_ManufacturerCode = 0x041b,
    .SDIO_FunctionNo = 1,
    .SDIO_FunctionClass = 0 },
  { .SDIO_ManufacturerID = 0x301,
    .SDIO_ManufacturerCode = 0x041b,
    .SDIO_FunctionNo = 1,
    .SDIO_FunctionClass = 0 },
  { .SDIO_ManufacturerID = 0x100,
    .SDIO_ManufacturerCode = 0x0303,
    .SDIO_FunctionNo = 1,
    .SDIO_FunctionClass = 0 },
  {}
};

static SDFUNCTION sli_driver = {
  .pName = "sli-SDIO WLAN",
  .Version = CT_SDIO_STACK_VERSION_CODE,   /* FIXME */
  .MaxDevices = 1,
  .NumDevices = 0,
  .pIds = sli_IdTable,
  .pProbe = (PVOID)sli_sdio_probe,
  .pRemove = sli_sdio_disconnect,
  .pSuspend = NULL,
  .pResume = NULL,
  .pWake = NULL,
};

#elif KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(26)
static const struct sdio_device_id sli_IdTable[] = {
  { SDIO_DEVICE(0x303, 0x100) }, { SDIO_DEVICE(0x041B, 0x0301) },
  { SDIO_DEVICE(0x041B, 0x0201) }, { SDIO_DEVICE(0x041B, 0x9330) },
  { SDIO_DEVICE(0x041B, 0x9116) }, { SDIO_DEVICE(0x041B, 0x9117) },
  { SDIO_DEVICE(0x041B, 0x917) }, { /* Blank */ },
};

static struct sdio_driver sli_driver = {
  .name = "sli-SDIO WLAN",
  .id_table = sli_IdTable,
  .probe = sli_sdio_probe,
  .remove = sli_sdio_disconnect,
};
#endif

/*==============================================*/
/**
 * @fn          VOID sli_deinit_interface(PVOID pContext)
 * @brief       Interrupt handler
 * @param[in]   pContext, Pointer to our adapter
 * @param[out]  none
 * @return      none
 * @section description:
 * This function de-intializes the bus driver
 */
VOID sli_deinit_interface(PVOID pContext)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_deinit_interface:\n");
#endif
  PSLI_ADAPTER Adapter = (PSLI_ADAPTER)pContext;
  return;
}

/* statics */

/*==============================================*/
/**
 * @fn          struct net_device* sli_allocdev(
 *                             INT32 sizeof_priv)
 * @brief       Allocate & initializes the network device
 * @param[in]   sizeof_priv, size of the priv area to be allocated
 * @param[out]  none
 * @return      Pointer to the network device structure is returned
 * @section description
 * Allocate & initialize the network device.This function
 * allocates memory for the network device & initializes it
 * with ethernet generic values
 */
struct net_device *sli_allocdev(INT32 sizeof_priv, uint8 vap_id)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_allocdev:\n");
#endif
  struct net_device *dev = NULL;
  if (vap_id == 0) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
    dev = alloc_netdev(sizeof_priv, "sli_wlan0", ether_setup);
#else
    dev = alloc_netdev(sizeof_priv, "sli_wlan0", NET_NAME_UNKNOWN, ether_setup);
#endif
#ifdef SLI_DEBUG_PRINT
    SLI_DEBUG(SLI_ZONE_INIT,
              "sli_allocdev: device allocated for sli_wlan0 %p\n", dev);
#endif
  } else if (vap_id == 1) {
    //! AP interface in concurrent mode
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
    dev = alloc_netdev(sizeof_priv, "sli_wlan1", ether_setup);
#else
    dev = alloc_netdev(sizeof_priv, "sli_wlan1", NET_NAME_UNKNOWN, ether_setup);
#endif
#ifdef SLI_DEBUG_PRINT
    SLI_DEBUG(SLI_ZONE_INIT, "sli_allocdev: device allocated for sli_wlan1\n");
#endif
  }
  return dev;
}

/*==============================================*/
/**
 * @fn          INT32 sli_registerdev(struct net_device *dev)
 * @brief       Register the network device
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      On success 0 is returned else a negative value signifying
 *              failure
 * @section description
 * This function is used to register the network device.
 */
INT32 sli_registerdev(struct net_device *dev)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_registerdev:\n");
#endif
  return register_netdev(dev);
}

/*==============================================*/
/**
 * @fn          INT32 sli_unregisterdev(struct net_device *dev)
 * @brief       Unregister the network device
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      On success 0 is returned else a negative value signifying
 *              failure
 * @section description
 * This function is used to unregisters the network device
 * & returns it back to the kernel
 */
VOID sli_unregisterdev(struct net_device *dev)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_unregisterdev:\n");
#endif
  unregister_netdev(dev);
  free_netdev(dev);
  return;
}

/*==============================================*/
/**
 * @fn          struct net_device_stats *
 *              sli_get_stats(struct net_device *dev)
 * @brief       Gives the stats info
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      Pointer to the net_device_stats structure
 * @section description
 * This function gives the statistical information regarding
 * the interface
 */
struct net_device_stats *sli_get_stats(struct net_device *dev)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_get_stats:\n");
#endif
  PSLI_ADAPTER Adapter = sli_getpriv(dev);
  return &Adapter->stats;
}

/*==============================================*/
/**
 * @fn          PSLI_ADAPTER sli_getpriv(struct net_device *dev)
 * @brief       Get the pointer for the adapter
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      Pointer to the adapter On success 0, negative value signifying
 *              failure
 * @section description
 * This function get the pointer for the adapter
 */
PSLI_ADAPTER sli_getpriv(struct net_device *dev)
{
  PSLI_NET_DEV sli_net_device = netdev_priv(dev);
  return sli_net_device->sli_adapter_ptr;
}

/*==============================================*/
/**
 * @fn          PSLI_NET_DEV sli_getdevpriv(struct net_device *dev)
 * @brief       Get the pointer for the adapter
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      Pointer to the adapter On success 0, negative value signifying
 *              failure
 * @section description
 * This function get the pointer for the adapter
 */
PSLI_NET_DEV sli_getdevpriv(struct net_device *dev)
{
  PSLI_NET_DEV sli_net_device = netdev_priv(dev);
  return sli_net_device;
}

/*==============================================*/
/**
 * @fn          INT32 sli_ioctl(struct net_device *dev,
 *                              struct ifreq *ifr,
 *                              INT32 cmd)
 * @brief       Gives the stats info
 * @param[in]   dev, pointer to our network device structure
 * @param[in]   ifr, pointer to ifr structure
 * @param[in]   cmd, type of command or request
 * @param[out]  none
 * @return      Pointer to the net_device_stats structure
 * @section description
 * This function is registered to driver and will be called when
 * ioctl issued from user space.
 */
INT32 sli_ioctl(struct net_device *dev, struct ifreq *ifr, INT32 cmd)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_ioctl:\n");
#endif
  PSLI_ADAPTER Adapter = sli_getpriv(dev);
  struct iwreq *wrq = (struct iwreq *)ifr;
  int16 retval = 0;
  uint8 local_buffer[4096];
  uint16 data = 0;
  uint32 addr, len, i, len_dwords;
  uint8 get_status = 0;

  switch (cmd) {
    case OID_WSC_GET_STATUS: {
      sli_down_interruptible(&Adapter->int_check_sem);
      sli_up_sem(&Adapter->int_check_sem);

      if (retval && copy_to_user(wrq->u.data.pointer, &retval, sizeof(UINT8))) {
        SLI_DEBUG(SLI_ZONE_SPI_DBG, "sli_ioctl : Failed to perform operation\n");
        return -EFAULT;
      } else if (copy_to_user(wrq->u.data.pointer, &get_status, sizeof(UINT8))) {
        SLI_DEBUG(SLI_ZONE_SPI_DBG, "sli_ioctl : Failed to perform operation\n");
        return -EFAULT;
      }
    } break;
    case OID_WSC_BOOT_READ: {
      sli_down_interruptible(&Adapter->int_check_sem);
      retval = sli_boot_insn(REG_READ, &data);
      sli_up_sem(&Adapter->int_check_sem);
      if (retval < 0) {
        return -EFAULT;
      }
      if (copy_to_user(wrq->u.data.pointer, &data, sizeof(uint16))) {
        SLI_DEBUG(SLI_ZONE_SPI_DBG, "sli_ioctl : Failed to perform operation\n");
        return -EFAULT;
      }
    } break;
    case OID_WSC_BOOT_WRITE: {
      sli_down_interruptible(&Adapter->int_check_sem);
      copy_from_user(local_buffer, wrq->u.data.pointer, 2);
      retval = sli_boot_insn(REG_WRITE, (uint16 *)local_buffer);
      sli_up_sem(&Adapter->int_check_sem);
      if (retval < 0) {
        return -EFAULT;
      }
    } break;
    case OID_WSC_BOOT_PING_WRITE: {
      sli_down_interruptible(&Adapter->int_check_sem);
      copy_from_user(local_buffer, wrq->u.data.pointer, 4096);
      retval = sli_boot_insn(PING_WRITE, (uint16 *)local_buffer);
      sli_up_sem(&Adapter->int_check_sem);
      if (retval < 0) {
        return -EFAULT;
      }
    } break;
    case OID_WSC_BOOT_PONG_WRITE: {
      sli_down_interruptible(&Adapter->int_check_sem);
      copy_from_user(local_buffer, wrq->u.data.pointer, 4096);
      retval = sli_boot_insn(PONG_WRITE, (uint16 *)local_buffer);
      sli_up_sem(&Adapter->int_check_sem);
      if (retval < 0) {
        return -EFAULT;
      }
    } break;
    case OID_WSC_POWER_SAVE_ENABLE: {
      uint8 currentPowerMode = 0;
      sli_down_interruptible(&Adapter->int_check_sem);
      copy_from_user(&currentPowerMode, wrq->u.data.pointer, 1);
      Adapter->power_save_enable = !!currentPowerMode;
      sli_up_sem(&Adapter->int_check_sem);
    } break;
    case OID_MASTER_READ: {
      addr = *(uint32_t *)wrq->u.data.pointer;
      len = wrq->u.data.length;
      len_dwords = (len & 3) ? ((len / 4) + 1) : (len / 4);
      memset(local_buffer, 0, sizeof(local_buffer));
      sli_down_interruptible(&Adapter->int_check_sem);
      for (i = 0; i < len_dwords; i++) {
        retval = sli_boot_req(local_buffer + (i * 4), addr + (i * 4));
      }
      sli_up_sem(&Adapter->int_check_sem);
      if (retval < 0) {
        return -EFAULT;
      }
      if (copy_to_user((wrq->u.data.pointer), local_buffer, len)) {
        SLI_DEBUG(SLI_ZONE_SPI_DBG, "sli_ioctl : failed to perform\n");
        return -EFAULT;
      }
    } break;
  }
  return retval;
}

/*==============================================*/
/**
 * @fn          INT32 sli_open(struct net_device *dev)
 * @brief       Opens the interface
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      On success 0 is returned else a negative value signifying
 *              failure
 * @section description
 * This function opens the interface
 */
INT32
sli_open(struct net_device *dev)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_open:\n");
#endif
  return 0;
}

/*==============================================*/
/**
 * @fn          INT32 sli_stop(struct net_device *dev)
 * @brief       Stops/brings down the interface
 * @param[in]   dev, pointer to our network device structure
 * @param[out]  none
 * @return      On success 0 is returned else a negative value signifying
 *              failure
 * @section description
 * This function stops/brings down the interface
 */
INT32
sli_stop(struct net_device *dev)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_stop:\n");
#endif
  SLI_DEBUG(SLI_ZONE_INFO, "sli_stop\n");
  return 0;
}

/*==============================================*/
/**
 * @fn          struct net_device* sli_netdevice_op(vap_id)
 * @brief       netdevice related operations
 * @param[in]   vap_id device vap id
 * @param[out]  none
 * @return      dev, Pointer to net device structure
 * @section description
 * This function performs all net device related operations like
 * allocating,initializing and registering the netdevice.
 */
struct net_device *sli_netdevice_op(UINT8 vap_id)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_netdevice_op:\n");
#endif
  struct net_device *dev;
  PSLI_NET_DEV net_dev;

  /*Allocate & initialize the network device structure*/
  dev = sli_allocdev(sizeof(SLI_NET_DEV), vap_id);

  if (dev == NULL) {
    SLI_DEBUG(SLI_ZONE_ERROR,
              "sli_netdevice_op: Failure in allocation of net-device\n");
    return dev;
  }

  //! Net device settings init
  net_dev = sli_getdevpriv(dev);
  net_dev->sli_adapter_ptr = Adapter;

#if KERNEL_VERSION_LESS_THAN_EQUALS_2_6_(28)
  dev->open = sli_open;
  dev->stop = sli_stop;
  dev->hard_start_xmit = sli_xmit;
  dev->get_stats = sli_get_stats;
  dev->do_ioctl = sli_ioctl;
  dev->hard_header_len = 30;
#else
  static struct net_device_ops dev_ops = {
    .ndo_open = sli_open,
    .ndo_stop = sli_stop,
    .ndo_start_xmit = sli_xmit,
    .ndo_do_ioctl = sli_ioctl,
    .ndo_get_stats = sli_get_stats,
  };

  dev->netdev_ops = &dev_ops;
  dev->hard_header_len = 30;
#endif
  if (sli_registerdev(dev) != 0) {
    SLI_DEBUG(
      SLI_ZONE_ERROR,
      "sli_netdevice_op: Registration of net-device failed for vap id %d\n",
      vap_id);
    free_netdev(dev);
    return NULL;
  }

  return dev;
}

/*==============================================*/
/**
 * @fn          SLI_STATUS sli_probe(VOID)
 * @brief       All the initializations at load time
 * @param[in]   none
 * @param[out]  none
 * @return      SLI_STATUS_SUCCESS in case of successful initialization
 *              or a negative error code signifying failure
 * @section description
 * This function is called at the module load time.
 * All the initialization work is done here.
 */
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
BOOLEAN __devinit sli_sdio_probe(PSDFUNCTION pfunction, PSDDEVICE pDevice)
#elif KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(26)
int sli_sdio_probe(struct sdio_func *pfunction, const struct sdio_device_id *id)
#endif
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_probe:\n");
#endif
  struct net_device *dev = NULL;
#if SLI_CONCURRENT_MODE
  struct net_device *dev_ap = NULL;
#endif
  SLI_STATUS status = SLI_STATUS_SUCCESS;
  UINT8 ii;
  UINT16 retval = 0;


  //! Freeing Adapter if it is already allocated.
  if (Adapter != NULL) {
    SLI_DEBUG(SLI_ZONE_INIT, "sli_probe: Freed Adapter\n");
    kfree(Adapter);
    Adapter = NULL;
  }

  //! Allocating memory for Adapter from kernel Heap memory pool
  Adapter = (PSLI_ADAPTER)kmalloc(sizeof(SLI_ADAPTER), GFP_ATOMIC);

  if (Adapter == NULL) {
    SLI_DEBUG(SLI_ZONE_INIT, "sli_probe: kmalloc failed\n");
  }

  //! Reseting the adapter
  sli_memset(Adapter, 0, sizeof(SLI_ADAPTER));

  dev = sli_netdevice_op(0);
  if (!dev) {
    SLI_DEBUG(SLI_ZONE_ERROR,
              "sli_probe: Failed to perform netdevice operations\n");
    return SLI_STATUS_FAIL;
  }

#if SLI_CONCURRENT_MODE
  dev_ap = sli_netdevice_op(1);
  if (!dev_ap) {
    SLI_DEBUG(SLI_ZONE_ERROR,
              "sli_probe: Failed to perform netdevice operations for AP1\n");
    return SLI_STATUS_FAIL;
  }
#endif

  SLI_DEBUG(SLI_ZONE_INIT, "sli_probe: Net device operations suceeded\n");

#ifdef ENABLE_WMM_FEATURE
  dbg_test_values(Adapter);
#endif

  Adapter->net_device0 = dev;
#if SLI_CONCURRENT_MODE
  Adapter->net_device1 = dev_ap;
#endif
  glbl_net_device = dev;

  for (ii = 0; ii < 4; ii++) {
    sli_queue_init(&Adapter->list[ii]);
  }

  Adapter->workqueue = create_singlethread_workqueue("gdvr_work");
  if (Adapter->workqueue == NULL) {
    SLI_DEBUG(SLI_ZONE_INIT, "Work queue Fail\n");
    goto fail;
  }
  SLI_DEBUG(SLI_ZONE_INIT, "\n SDIO: claiming host");
  sli_sdio_claim_host(pfunction);
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  status = sli_enable_interface(pDevice);

  if (!SDIO_SUCCESS((status))) {
    SLI_DEBUG(SLI_ZONE_ERROR, (TEXT("%s: Failed to enable interface for the "
                                    "kernels b/w 2.6.18 and 2.6.22\n"),
                               __func__));
    sli_sdio_release_host(pfunction);
    return status;
  }
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
  status = sli_enable_interface(pfunction);
  if (status != 0) {
    SLI_DEBUG(SLI_ZONE_SDIO_DBG, "%s: Failed to enable interface\n", __func__);
    sli_sdio_release_host(pfunction);
    return status;
  }
#endif

  sli_sdio_release_host(pfunction);

  /*Enable the SPI interface*/
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18))
  INIT_WORK(&Adapter->handler, (void *)sli_interrupt_handler,
            (void *)&Adapter->handler);
#else
  INIT_WORK(&Adapter->handler, (void *)sli_interrupt_handler);
#endif
  SLI_DEBUG(SLI_ZONE_INIT, "sli_probe: Enabled the interface\n");

  sli_spinlock_init(&Adapter->lockqueue);
#ifdef init_MUTEX
  sli_init_mutex(&Adapter->int_check_sem);
  sli_init_mutex(&Adapter->sleep_ack_sem); // default
  sli_init_mutex(&Adapter->tx_data_sem);   // default
#else
  sli_sema_init(&Adapter->int_check_sem, 1);
  sli_sema_init(&Adapter->sleep_ack_sem, 1);
  sli_sema_init(&Adapter->tx_data_sem, 1);
#endif

  /* Requesting thread */
  sli_Init_Event(&Adapter->Event);

  sli_Init_Event(&Adapter->PwrSaveEvent);
  sli_Init_Thread(Adapter);
  SLI_DEBUG(SLI_ZONE_INIT, "sli_probe: Initialized thread & Event\n");

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  Adapter->pDevice = pDevice;
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
  Adapter->pfunction = pfunction;
#endif

  sli_setcontext(pfunction, (void *)Adapter);
  /*Initalizing the hardware*/
  sli_sdio_claim_host(pfunction);

  status = sli_setupcard(Adapter);
  if (status != 0) {
    SLI_DEBUG(SLI_ZONE_SDIO_DBG, (TEXT("%s:Failed to setup card\n"), __func__));
    kfree((uint8 *)Adapter->DataRcvPacket[0]);
    sli_sdio_release_host(pfunction);
    goto fail;
  }

  SLI_DEBUG(SLI_ZONE_SDIO_DBG,
            (TEXT("%s: Setup card succesfully\n"), __func__));
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18)
  Adapter->sdio_high_speed_enable = 1;
#endif
  sli_sdio_release_host(pfunction);

  if (sli_init_host_interface(Adapter) != 0) {
    SLI_DEBUG(SLI_ZONE_SDIO_DBG,
              (TEXT("%s:Failed to init slave regs\n"), __func__));
    sli_sdio_release_host(pfunction);
    goto fail;
  }

  return status;
  /*Failure in one of the steps will cause the control to be transferred here*/
  fail_out:
  for (ii = 0; ii < 4; ii++) {
    sli_queue_purge(&Adapter->list[ii]);
  }
  sli_sdio_claim_host(pfunction);
  sli_sdio_release_irq(pfunction);

  /*Disable the interface*/
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  sli_disable_interface(pDevice);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
  sli_disable_interface(pfunction);
#endif
  /* Release the host. It should be called after calling sdio_disable_func() */
  sli_sdio_release_host(pfunction);

  SLI_DEBUG(SLI_ZONE_SDIO_DBG,
            (TEXT("%s: Failed to initialize...Exiting\n"), __func__));
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  return 1;
#elif KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(26)
  return 0;
#endif

  fail_out1:
  sli_deinit_interface(Adapter);

  fail:
  if (dev) {
    /*Unregisters & returns the network device to the kernel*/
    sli_unregisterdev(dev);
  }

#if SLI_CONCURRENT_MODE
  if (dev_ap) {
    /*Unregisters & returns the network device to the kernel*/
    sli_unregisterdev(dev_ap);
  }
#endif

  /*Disable the interface*/

  SLI_DEBUG(SLI_ZONE_ERROR, "sli_probe: Failure to initialize\n");
  return SLI_STATUS_FAIL;
}

/*==============================================*/
/**
 * @fn          VOID sli_interrupt_handler(struct work_struct *work)
 * @brief       Interrupt handler
 * @param[in]   struct work_struct *work, Pointer to work_struct
 *              structure
 * @param[out]  none
 * @return      none
 * @section description:
 * Upon the occurence of an interrupt, the interrupt handler will
 * be called
 */
uint8 interrupt_recv = 0;
VOID sli_interrupt_handler(struct work_struct *work)
{
#ifdef SLI_DEBUG_PRINT
#endif
  interrupt_recv++;
  PSLI_ADAPTER Adapter = container_of(work, SLI_ADAPTER, handler);
  uint32 InterruptType = 0;
  INT32 retval;
  uint8 temp;
  UINT8 InterruptStatus = 0;
  uint16 offset;
  uint32 length, status;
  UINT8 *rx_buff = NULL;

#ifdef DSLI_DEBUG_PRINT
  uint32 i = 0;
#endif

  uint32 i = 0;
  do {
    retval = read_register(Adapter, SDIO_FUN1_INT_REG, 0, &InterruptStatus);
    if (retval != 0) {
      schedule();
    }
    Adapter->InterruptStatus = InterruptStatus;
    InterruptStatus &= 0xE;
    if (InterruptStatus == 0) {
      return;
    }

    rx_buff = Adapter->rx_buffer;

    do {
      InterruptType = SLI_GET_INTERRUPT_TYPE(InterruptStatus);
      switch (InterruptType) {
        case SDIO_BUFFER_FULL: {
          Adapter->BufferFull = 1;
          ack_interrupt(Adapter, SD_PKT_BUFF_FULL);
          SLI_DEBUG(SLI_ZONE_SDIO_DBG, "BUFFER FULL \n");
          return;
        } break;

        case SDIO_BUFFER_FREE: {
          Adapter->BufferFull = 0;
          ack_interrupt(Adapter, SD_PKT_BUFF_EMPTY);
          SLI_DEBUG(SLI_ZONE_SDIO_DBG, "BUFFER EMPTY \n");
          return;
        } break;

        case FIRMWARE_STATUS: {
          Adapter->FSM_STATE = FSM_CARD_NOT_READY;
          SLI_DEBUG(SLI_ZONE_SPI_DBG, "FIRMWARE CARD NOT READY\n");
          return;
        } break;

        case SDIO_DATA_PENDING: {
          sli_up_sem(&Adapter->int_check_sem);

          if (sli_sdio_frame_read(Adapter)) {
            SLI_DEBUG(SLI_ZONE_ERROR,
                      (TEXT("ganges_interrupt_handler: Unable to recv pkt\n")));
            sli_down_sem(&Adapter->int_check_sem);
            return;
          }
          length =
            ((Adapter->DataRcvPacket[2] | (Adapter->DataRcvPacket[3] << 8))
             & 0xfff)
            + CPC_HEADER_LENGTH;
          memcpy(rx_buff, &Adapter->DataRcvPacket[0], length);
          sli_send_rsp_to_userspace(Adapter, rx_buff);
          sli_down_sem(&Adapter->int_check_sem);
        } break;

        default: {
          sli_up_sem(&Adapter->int_check_sem);
          ack_interrupt(Adapter, InterruptStatus);
          SLI_DEBUG(
            SLI_ZONE_SPI_DBG,
            (TEXT("ganges_interrupt_handler: No more pending interrupts\n")));
          sli_down_sem(&Adapter->int_check_sem);
          return;
        }
      }
      InterruptStatus ^= BIT(InterruptType - 1);
    } while (InterruptStatus);
  } while (1);
  return;
}

/*==============================================*/
/**
 * @fn          SLI_STATUS sli_linux_disconnect(VOID)
 * @brief       Reverse of probe
 * @param[in]   none
 * @param[out]  none
 * @return      SLI_STATUS_SUCCESS in case of successful initialization
 *              or a negative error code signifying failure
 * @section description
 * This function performs the reverse of the probe function.
 */
SLI_STATUS sli_linux_disconnect(VOID)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_linux_disconnect:\n");
#endif
  SLI_STATUS Status = SLI_STATUS_SUCCESS;
  UINT32 ii;
  PSLI_ADAPTER Adapter = sli_getpriv(glbl_net_device);

  SLI_DEBUG(SLI_ZONE_INFO, "sli_linux_disconnect: Deinitializing\n");

  Adapter->FSM_STATE = 0;

  sli_Delete_Event(&Adapter->PwrSaveEvent);

  sli_Delete_Event(&Adapter->Event);

  SLI_DEBUG(SLI_ZONE_INFO, "Killing thread \n");
  Adapter->halt_flag = 1;

  sli_Kill_Thread(Adapter);

  SLI_DEBUG(SLI_ZONE_INFO, "Purge queue \n");
  for (ii = 0; ii < 4; ii++) {
    sli_queue_purge(&Adapter->list[ii]);
  }

  /*Return the network device to the kernel*/
  SLI_DEBUG(SLI_ZONE_INFO, "Deinit interface \n");
  sli_deinit_interface(Adapter);
  SLI_DEBUG(SLI_ZONE_INFO, "Unregister netdev \n");
  sli_unregisterdev(Adapter->net_device0);
#if SLI_CONCURRENT_MODE
  sli_unregisterdev(Adapter->net_device1);
#endif

  /*Disable the interface*/
  SLI_DEBUG(SLI_ZONE_INFO, "sli_linux_disconnect: Disabling the interface\n");
  return Status;
}

/*==============================================*/
/**
 * @fn          SLI_STATUS sli_init_interface(PVOID pContext)
 * @brief       Interrupt handler
 * @param[in]   pContext, Pointer to our adapter
 * @param[out]  none
 * @return      Returns SLI_STATUS_SUCCESS or SLI_STATUS_FAIL
 * @section description:
 * This function intializes the bus driver
 */
SLI_STATUS
sli_init_interface(PVOID pContext)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_init_interface:\n");
#endif
  INT32 ret = 0;
  PSLI_ADAPTER Adapter = (PSLI_ADAPTER)pContext;

  uint32 status;
  sli_sdio_claim_host(Adapter->pfunction);
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  status = sli_request_interrupt_handler(Adapter->pDevice,
                                         sli_sdio_interrupt_handler, Adapter);
#elif KERNEL_VERSION_GREATER_THAN_EQUALS_2_6_(26)
  status = sli_request_interrupt_handler(Adapter->pfunction,
                                         sli_sdio_interrupt_handler, Adapter);
#endif
  if (status != 0) {
    SLI_DEBUG(SLI_ZONE_SDIO_DBG,
              (TEXT("%s:Failed to register interrupt\n"), __func__));
    sli_sdio_release_host(Adapter->pfunction);
    return -1;
  }
  SLI_DEBUG(SLI_ZONE_SDIO_DBG,
            (TEXT("%s: Registered Interrupt handler\n"), __func__));

  /*Initalizing the hardware*/
  sli_sdio_release_host(Adapter->pfunction);

  return SLI_STATUS_SUCCESS;
}

/*==============================================*/
/**
 * @fn          SLI_STATIC INT32 __init
 *              sli_module_init(VOID)
 * @brief       module init
 * @param[in]   pContext, Pointer to our adapter
 * @param[out]  none
 * @return      0 in case of success or a negative
 *              value signifying failure
 * @section description:
 * This function is invoked when the module is loaded into the
 * kernel. It registers the client driver.
 */
SLI_STATIC INT32 __init sli_module_init(VOID)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_module_init:\n");
#endif
  INT32 rc;

  SLI_STATUS status = SLI_STATUS_SUCCESS;
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "INIT MODULE\n");

  rc = sli_register_genl();
  if (rc != 0) {
    goto failure;
  }

  SLI_DEBUG(SLI_ZONE_SDIO_DBG, "SDIO REGISTER DRIVER \n");
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  status = SDIOErrorToOSError(SDIO_RegisterFunction(&onebox_driver));
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
  status = sdio_register_driver(&sli_driver);
#endif
  if (status) {
    goto failure;
  }
  return 0;

  failure:
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "an error occured while inserting the module\n");
  return -1;
}

/*==============================================*/
/**
 * @fn          SLI_STATIC VOID __exit
 *              sli_module_exit(VOID)
 * @brief       module exit
 * @param[in]   VOID
 * @param[out]  none
 * @return      none
 * @section description:
 * At the time of removing/unloading the module, this function is
 * called. It unregisters the client driver.
 */
SLI_STATIC VOID __exit sli_module_exit(VOID)
{
#ifdef SLI_DEBUG_PRINT
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "\nsli_module_exit:\n");
#endif
  INT32 ret;
  SLI_DEBUG(SLI_ZONE_SPI_DBG, "EXIT MODULE\n");
  int32 status;
  /*Unregistering the client driver*/
  ret = sli_unregister_genl();
  if (ret != 0) {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "genl unregister failed %i\n", ret);
    return;
  }

#if KERNEL_VERSION_BTWN_2_6_(18, 22)
  status = SDIO_UnregisterFunction(&sli_driver);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
  sdio_unregister_driver(&sli_driver);
#endif

  //! Free the Adapter when driver is removed
  if (Adapter != NULL) {
    SLI_DEBUG(SLI_ZONE_SPI_DBG, "Freed Adapter successfully\n");
    kfree(Adapter);
    Adapter = NULL;
  }

  if (status) {
    goto fail;
  }

  SLI_DEBUG(SLI_ZONE_SPI_DBG, "Module removed successfully\n");
  return;
  fail:
  SLI_DEBUG(SLI_ZONE_ERROR, "Error in removing the module\n");
  return;
}

module_init(sli_module_init);
module_exit(sli_module_exit);
MODULE_LICENSE("GPL");

/* $EOF */
/* Log */
