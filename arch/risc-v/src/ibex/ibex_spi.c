/****************************************************************************
 * arch/risc-v/src/ibex/ibex_spi.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <arch/board/board.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "riscv_internal.h"

#include "hardware/ibex_gpio.h"
#include "hardware/ibex_spi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device hardware configuration */
struct ibex_spi_config_s
{
  uint32_t        clk_freq; /* SPI clock frequency */
  enum spi_mode_e mode;     /* SPI mode */
  uint8_t         nbits;    /* SPI send/receive data size */
};

struct ibex_spi_dev_s
{
  struct spi_dev_s         spi_dev; /* Externally visible part of the
                                       SPI interface */
  struct ibex_spi_config_s config;  /* Port configuration */
  int                      refs;    /* Referernce count */
  mutex_t                  lock;    /* Held while chip is selected for
                                       mutual exclusion */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int ibex_spi_lock(struct spi_dev_s *dev, bool lock);

static void ibex_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected);

static uint32_t ibex_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void ibex_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void ibex_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int ibex_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t ibex_spi_status(struct spi_dev_s *dev,
                                uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int ibex_spi_cmddata(struct spi_dev_s *dev,
                             uint32_t devid, bool cmd);
#endif
static uint32_t ibex_spi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void ibex_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#else
static void ibex_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer, size_t nwords);
static void ibex_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int ibex_spi_trigger(struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct spi_ops_s g_ibex_spi_ops =
{
  .lock             = ibex_spi_lock,
  .select           = ibex_spi_select,
  .setfrequency     = ibex_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = ibex_spi_setdelay,
#endif
  .setmode          = ibex_spi_setmode,
  .setbits          = ibex_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = ibex_spi_hwfeatures,
#endif
  .status           = ibex_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = ibex_spi_cmddata,
#endif
  .send             = ibex_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = ibex_spi_exchange,
#else
  .sndblock         = ibex_spi_sndblock,
  .recvblock        = ibex_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = ibex_spi_trigger,
#endif
  .registercallback = NULL,
};

static struct ibex_spi_dev_s g_ibex_spi_dev =
{
  .spi_dev =
  {
    .ops      = &g_ibex_spi_ops
  },
  .config     = 
  {
    .clk_freq = IBEX_SPI_FREQ,
    .mode     = IBEX_SPI_MODE,
    .nbits    = IBEX_SPI_NBITS
  },
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ibex_spi_lock
 *
 * Description:
 *   Lock or unlock the SPI device
 *
 * Input Parameters:
 *   priv   - Private SPI device structure
 *   lock   - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   The result of lock or unlock the SPI device
 *
 ****************************************************************************/
static int ibex_spi_lock(struct spi_dev_s *dev, bool lock)
{
  int ret;
  struct ibex_spi_dev_s *ibex_dev = (struct ibex_spi_dev_s *)dev;

  if (lock)
    ret = nxmutex_lock(&ibex_dev->lock);
  else
    ret = nxmutex_unlock(&ibex_dev->lock);

  return ret;
}

/****************************************************************************
 * Name: ibex_spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 * Input Parameters:
 *   priv     - Private SPI device structure
 *   devid    - Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void ibex_spi_select(struct spi_dev_s *dev, uint32_t devid,
                            bool selected)
{
  spiinfo("ibex_spi_select(): devid: %lu, CS: %s\n",
    devid, selected ? "select" : "free");
}

/****************************************************************************
 * Name: ibex_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/
static uint32_t ibex_spi_setfrequency(struct spi_dev_s *dev,
                                      uint32_t frequency)
{
  struct ibex_spi_dev_s *ibex_dev = (struct ibex_spi_dev_s *)dev;

  if (ibex_dev->config.clk_freq != frequency)
  {
    spierr("ibex_spi_setfrequency(): Setting SPI frequency not supported,"
    " only %ld Hz available.\n", ibex_dev->config.clk_freq);
    PANIC();
  }

  return ibex_dev->config.clk_freq;
}

/****************************************************************************
 * Name: ibex_spi_setdelay
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *   ifdelay    - The delay between frames
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/
#ifdef CONFIG_SPI_DELAY_CONTROL
static int ibex_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                                uint32_t stopdelay, uint32_t csdelay,
                                uint32_t ifdelay)
{
  spierr("ibex_spi_setdelay(): not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: ibex_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/
static void ibex_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct ibex_spi_dev_s *ibex_dev = (struct ibex_spi_dev_s *)dev;

  if (ibex_dev->config.mode != mode)
  {
    spierr("ibex_spi_setmode(): mode %d not supported.\n", mode);
    PANIC();
  }
}

/****************************************************************************
 * Name: ibex_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/
static void ibex_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct ibex_spi_dev_s *ibex_dev = (struct ibex_spi_dev_s *)dev;

  if (ibex_dev->config.nbits != nbits)
  {
    spierr("ibex_spi_setbits(): %d bits not supported.\n", nbits);
    PANIC();
  }
}

/****************************************************************************
 * Name: ibex_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/
 #ifdef CONFIG_SPI_HWFEATURES
 static int ibex_spi_hwfeatures(struct spi_dev_s *dev,
                                 spi_hwfeatures_t features)
 {
  spierr("ibex_spi_hwfeatures(): not supported.\n");
  DEBUGPANIC();

  return -1;
 }
 #endif

/****************************************************************************
 * Name: ibex_spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/
static uint8_t ibex_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  return getreg32(IBEX_SPI_STATUS);
}

/****************************************************************************
 * Name: ibex_spi_cmddata
 *
 * Description:
 *   Some devices require an additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 *   This function reconfigures MISO from SPI Pin to GPIO Pin, and sets
 *   MISO to high (data) or low (command). ibex_spi_select() will revert
 *   MISO back from GPIO Pin to SPI Pin.  We must revert because the SPI Bus
 *   may be used by other drivers.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/
#ifdef CONFIG_SPI_CMDDATA
static int ibex_spi_cmddata(struct spi_dev_s *dev,
                            uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
  {
    modifyreg32(IBEX_GPIO_OUT, cmd ? BOARD_OLED_DATA_CMD_MASK : 0,
                cmd ? 0 : BOARD_OLED_DATA_CMD_MASK);
    return OK;
  }

  spierr("ibex_spi_cmddata(): devid=%ld not supported\n", devid);
  DEBUGPANIC();

  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: ibex_spi_send
 *
 * Description:
 *   Exchange one word on SPI.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/
static uint32_t ibex_spi_send(struct spi_dev_s *dev, uint32_t wd)
{ 
  spiinfo("ibex_spi_send(): SPI receive not implemented.\n");

  while (getreg32(IBEX_SPI_STATUS) & IBEX_SPI_STATUS_TX_FULL_MASK);
  putreg32(wd, IBEX_SPI_TX);

  return 0;
}

/****************************************************************************
 * Name: ibex_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#ifdef CONFIG_SPI_EXCHANGE
 static void ibex_spi_exchange(struct spi_dev_s *dev,
                            const void *txbuffer, void *rxbuffer,
                            size_t nwords)
{
  spiinfo("ibex_spi_exchange(): SPI receive not implemented.\n");

  uint8_t recv_data = 0;
  for (int i = 0; i < nwords; i++)
  {
    if (txbuffer)
    {
      recv_data = ibex_spi_send(dev, ((uint8_t*)txbuffer)[i]);
    }

    if (rxbuffer)
    {
      ((uint8_t*)rxbuffer)[i] = recv_data;
    }
  }
}

#else
/****************************************************************************
 * Name: ibex_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void ibex_spi_sndblock(struct spi_dev_s *dev,
                            const void *txbuffer, size_t nwords)
{
  if (!txbuffer)
  {
    spierr("ibex_spi_sndblock(): txbufer=NULL.");
    PANIC();
  }

  for (int i = 0; i < nwords; i++)
    ibex_spi_send(dev, ((uint8_t*)txbuffer)[i]);
}

/****************************************************************************
 * Name: ibex_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void ibex_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer, size_t nwords)
{
  spiinfo("ibex_spi_recvblock(): SPI receive not implemented.\n");
}
#endif

/****************************************************************************
 * Name: ibex_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/
#ifdef CONFIG_SPI_TRIGGER
static int ibex_spi_trigger(struct spi_dev_s *dev)
{
  spierr("ibex_spi_trigger(): not supported\n");
  DEBUGPANIC();

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ibex_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/
struct spi_dev_s *ibex_spibus_initialize(int port)
{
  struct ibex_spi_dev_s *ibex_dev = &g_ibex_spi_dev;

  if (port != 0)
  {
    spierr("ibex_spibus_initialize(): port=%d not supported.\n", port);
    return NULL;
  }

  nxmutex_lock(&ibex_dev->lock);
  if (ibex_dev->refs != 0)
  {
    ibex_dev->refs++;
    nxmutex_unlock(&ibex_dev->lock);

    return (struct spi_dev_s *)ibex_dev;
  }

  ibex_dev->refs++;
  nxmutex_unlock(&ibex_dev->lock);

  return (struct spi_dev_s *)ibex_dev;
}

/****************************************************************************
 * Name: ibex_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/
int ibex_spibus_uninitialize(struct spi_dev_s *dev)
{
  struct ibex_spi_dev_s *ibex_dev = (struct ibex_spi_dev_s *)dev;
  DEBUGASSERT(ibex_dev);

  if (!ibex_dev->refs)
  {
    spierr("ibex_spibus_uninitialize(): ibex_dev->refs=0.\n");
    return ERROR;
  }

  nxmutex_lock(&ibex_dev->lock);
  ibex_dev->refs--;
  nxmutex_unlock(&ibex_dev->lock);
  
  return OK;
}