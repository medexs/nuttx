/****************************************************************************
 * arch/risc-v/src/ibex-ds/ibex_ds_serial.c
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
#include <nuttx/serial/serial.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"

#include "hardware/ibex_ds_uart.h"

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CONSOLE_DEV g_uart0port

#define CONFIG_UART0_BAUD    115200
#define CONFIG_UART0_BITS    8
#define CONFIG_UART0_STOP    false
#define CONFIG_UART0_PARITY  0

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct ibex_ds_uart_config_s
{
  uint8_t  idx;       /* UART idx */
  uint32_t baud;      /* Configured baud */
  uint8_t  data_bits; /* Number of bits */
  bool     stop_bits; /* Stop bits: true = 2, false = 1 */
  uint8_t  parity;    /* Parity selection: 0 = none, 1 = odd, 2 = even */
};

struct ibex_ds_uart_s
{
  uint8_t                      rx_irq; /* IRQ from UARTs RX queue */
  uint8_t                      tx_irq; /* IRQ from UARTs TX queue */
  struct ibex_ds_uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Serial driver methods */
static int  ibex_ds_setup(struct uart_dev_s *dev);
static void ibex_ds_shutdown(struct uart_dev_s *dev);
static int  ibex_ds_attach(struct uart_dev_s *dev);
static void ibex_ds_detach(struct uart_dev_s *dev);
static int  ibex_ds_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  ibex_ds_receive(struct uart_dev_s *dev, unsigned int *status);
static void ibex_ds_rxint(struct uart_dev_s *dev, bool enable);
static bool ibex_ds_rxavailable(struct uart_dev_s *dev);
static void ibex_ds_send(struct uart_dev_s *dev, int ch);
static void ibex_ds_txint(struct uart_dev_s *dev, bool enable);
static bool ibex_ds_txready(struct uart_dev_s *dev);
static bool ibex_ds_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* UART0 I/O buffers */
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 operations */
static const struct uart_ops_s g_uart_ops =
{
  .setup       = ibex_ds_setup,
  .shutdown    = ibex_ds_shutdown,
  .attach      = ibex_ds_attach,
  .detach      = ibex_ds_detach,
  .ioctl       = ibex_ds_ioctl,
  .receive     = ibex_ds_receive,
  .rxint       = ibex_ds_rxint,
  .rxavailable = ibex_ds_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = ibex_ds_send,
  .txint       = ibex_ds_txint,
  .txready     = ibex_ds_txready,
  .txempty     = ibex_ds_txempty,
};

/* UART0 private info */
static struct ibex_ds_uart_s g_uart0priv =
{
  .rx_irq = IBEX_DS_UART0_RX_IRQ + RISCV_IRQ_ASYNC,
  .tx_irq = IBEX_DS_UART0_TX_IRQ + RISCV_IRQ_ASYNC,
  .config =
  {
    .idx       = 0,
    .baud      = CONFIG_UART0_BAUD,
    .data_bits = CONFIG_UART0_BITS,
    .stop_bits = CONFIG_UART0_STOP,
    .parity    = CONFIG_UART0_PARITY
  },
};

/* UART0 device structure */
static uart_dev_t g_uart0port =
{
  .isconsole = 1,
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart0priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* UART0 interrupt handlers */

/****************************************************************************
 * Name: __ibex_ds_rx_irq_handler
 *
 * Description:
 *   This is the UART RX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __ibex_ds_rx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_recvchars(dev);

  return OK;
}

/****************************************************************************
 * Name: __ibex_ds_tx_irq_handler
 *
 * Description:
 *   This is the UART TX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __ibex_ds_tx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_xmitchars(dev);

  return OK;
}

/* UART operations */

/****************************************************************************
 * Name: ibex_ds_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/
static int ibex_ds_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ibex_ds_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/
static void ibex_ds_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: ibex_ds_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/
static int ibex_ds_attach(struct uart_dev_s *dev)
{
  struct ibex_ds_uart_s *priv = (struct ibex_ds_uart_s *)dev->priv;

  int ret = irq_attach(priv->rx_irq, __ibex_ds_rx_irq_handler, (void *)dev);
  if (ret != OK)
  {
    irqerr("IRQERR: Failed to attach rx_irq handler\n");
    return ret;
  }
  irqinfo("IRQINFO: Attached rx_irq handler\n");

  ret = irq_attach(priv->tx_irq, __ibex_ds_tx_irq_handler, (void *)dev);
#ifdef CONFIG_DEBUG_IRQ_ERR
  if (ret != OK)
    irqerr("IRQERR: Failed to attach tx_irq handler\n");
  else
#endif
#ifdef CONFIG_DEBUG_IRQ_INFO
    irqinfo("IRQINFO: Attached tx_irq handler\n");
#endif

  return ret;
}

/****************************************************************************
 * Name: ibex_ds_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/
static void ibex_ds_detach(struct uart_dev_s *dev)
{
  struct ibex_ds_uart_s *priv = (struct ibex_ds_uart_s *)dev->priv;

  irq_detach(priv->rx_irq);
  irq_detach(priv->tx_irq);
}

/****************************************************************************
 * Name: ibex_ds_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/
static int ibex_ds_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: ibex_ds_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/
static int ibex_ds_receive(struct uart_dev_s *dev, unsigned int *status)
{
  int rx_data = -1;

  /* Check if UART0s RX fifo is not empty */
  if (!(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_RX_EMPTY_MASK))
    rx_data = getreg32(IBEX_DS_UART0_RX);

  *status = OK;
  return rx_data;
}

/****************************************************************************
 * Name: ibex_ds_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static void ibex_ds_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ibex_ds_uart_s *priv = (struct ibex_ds_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->rx_irq);
  else
    up_disable_irq(priv->rx_irq);
}

/****************************************************************************
 * Name: ibex_ds_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/
static bool ibex_ds_rxavailable(struct uart_dev_s *dev)
{
  return !(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_RX_EMPTY_MASK);
}

/****************************************************************************
 * Name: ibex_ds_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void ibex_ds_send(struct uart_dev_s *dev, int ch)
{
  /* Wait until there is space for a byte in TX fifo */
  while(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_TX_FULL_MASK);

  putreg32(ch, IBEX_DS_UART0_TX);
}

/****************************************************************************
 * Name: ibex_ds_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static void ibex_ds_txint(struct uart_dev_s *dev, bool enable)
{
  struct ibex_ds_uart_s *priv = (struct ibex_ds_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->tx_irq);
  else
    up_disable_irq(priv->tx_irq);
}

/****************************************************************************
 * Name: ibex_ds_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/
static bool ibex_ds_txready(struct uart_dev_s *dev)
{
  return !(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_TX_FULL_MASK);
}

/****************************************************************************
 * Name: ibex_ds_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/
static bool ibex_ds_txempty(struct uart_dev_s *dev)
{
  return (getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_TX_EMPTY_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ibex_ds_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/
void ibex_ds_earlyserialinit(void)
{
  /* Initialize CONSOLE_DEV */
  CONSOLE_DEV.isconsole = true;

  ibex_ds_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: ibex_ds_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/
void ibex_ds_serialinit(void)
{
  /* Initialize CONSOLE_DEV */
  CONSOLE_DEV.open_count = 0;
  CONSOLE_DEV.isconsole = true;

  /* Register the console */
  DEBUGVERIFY(uart_register("/dev/console", &CONSOLE_DEV));
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/
void riscv_earlyserialinit(void)
{
  ibex_ds_earlyserialinit();
}

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/
void riscv_serialinit(void)
{
  ibex_ds_serialinit();
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/
void up_putc(int ch)
{
  /* Check for LF */
  if (ch == '\n')
  {
    /* Add CR */
    
    /* Wait until there is space for a byte in TX fifo */
    while(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_TX_FULL_MASK);
    putreg32('\r', IBEX_DS_UART0_TX);
  }

  /* Wait until there is space for a byte in TX fifo */
  while(getreg32(IBEX_DS_UART0_STATUS) & IBEX_DS_UART_STATUS_TX_FULL_MASK);
  putreg32(ch, IBEX_DS_UART0_TX);
}