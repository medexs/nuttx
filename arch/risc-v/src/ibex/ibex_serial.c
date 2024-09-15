/****************************************************************************
 * arch/risc-v/src/ibex/ibex_serial.c
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

#include "hardware/ibex_uart.h"

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
struct uart_config_s
{
  uint8_t  idx;       /* UART idx */
  uint32_t baud;      /* Configured baud */
  uint8_t  data_bits; /* Number of bits */
  bool     stop_bits; /* Stop bits: true = 2, false = 1 */
  uint8_t  parity;    /* Parity selection: 0 = none, 1 = odd, 2 = even */
};

struct ibex_uart_s
{
  uint8_t              rx_irq; /* IRQ from UARTs RX queue */
  uint8_t              tx_irq; /* IRQ from UARTs TX queue */
  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Serial driver methods */
static int  ibex_setup(struct uart_dev_s *dev);
static void ibex_shutdown(struct uart_dev_s *dev);
static int  ibex_attach(struct uart_dev_s *dev);
static void ibex_detach(struct uart_dev_s *dev);
static int  ibex_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  ibex_receive(struct uart_dev_s *dev, unsigned int *status);
static void ibex_rxint(struct uart_dev_s *dev, bool enable);
static bool ibex_rxavailable(struct uart_dev_s *dev);
static void ibex_send(struct uart_dev_s *dev, int ch);
static void ibex_txint(struct uart_dev_s *dev, bool enable);
static bool ibex_txready(struct uart_dev_s *dev);
static bool ibex_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* UART0 I/O buffers */
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 operations */
static const struct uart_ops_s g_uart_ops =
{
  .setup       = ibex_setup,
  .shutdown    = ibex_shutdown,
  .attach      = ibex_attach,
  .detach      = ibex_detach,
  .ioctl       = ibex_ioctl,
  .receive     = ibex_receive,
  .rxint       = ibex_rxint,
  .rxavailable = ibex_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = ibex_send,
  .txint       = ibex_txint,
  .txready     = ibex_txready,
  .txempty     = ibex_txempty,
};

/* UART0 private info */
static struct ibex_uart_s g_uart0priv =
{
  .rx_irq = UART0_RX_IRQ,
  .tx_irq = UART0_TX_IRQ,
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
 * Name: __ibex_rx_irq_handler
 *
 * Description:
 *   This is the UART RX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __ibex_rx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_recvchars(dev);

  return OK;
}

/****************************************************************************
 * Name: __ibex_tx_irq_handler
 *
 * Description:
 *   This is the UART TX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __ibex_tx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_xmitchars(dev);

  return OK;
}

/* UART operations*/

/****************************************************************************
 * Name: ibex_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/
static int ibex_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ibex_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/
static void ibex_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: ibex_attach
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
static int ibex_attach(struct uart_dev_s *dev)
{
  struct ibex_uart_s *priv = (struct ibex_uart_s *)dev->priv;

  int ret = irq_attach(priv->rx_irq + RISCV_IRQ_ASYNC, __ibex_rx_irq_handler, (void *)dev);
  if (ret != OK)
    return ret;

  ret = irq_attach(priv->tx_irq + RISCV_IRQ_ASYNC, __ibex_tx_irq_handler, (void *)dev);

  return ret;
}

/****************************************************************************
 * Name: ibex_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/
static void ibex_detach(struct uart_dev_s *dev)
{
  struct ibex_uart_s *priv = (struct ibex_uart_s *)dev->priv;

  irq_detach(priv->rx_irq + RISCV_IRQ_ASYNC);
  irq_detach(priv->tx_irq + RISCV_IRQ_ASYNC);
}

/****************************************************************************
 * Name: ibex_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/
static int ibex_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: ibex_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/
static int ibex_receive(struct uart_dev_s *dev, unsigned int *status)
{
  int rx_data = -1;

  /* Check if UART0s RX fifo is not empty */
  if (!(getreg32(UART0_STATUS) & UART_STATUS_RX_EMPTY_MASK))
    rx_data = getreg32(UART0_RX);

  return rx_data;
}

/****************************************************************************
 * Name: ibex_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static void ibex_rxint(struct uart_dev_s *dev, bool enable)
{
  struct ibex_uart_s *priv = (struct ibex_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->rx_irq);
  else
    up_disable_irq(priv->rx_irq);
}

/****************************************************************************
 * Name: ibex_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/
static bool ibex_rxavailable(struct uart_dev_s *dev)
{
  return !(getreg32(UART0_STATUS) & UART_STATUS_RX_EMPTY_MASK);
}

/****************************************************************************
 * Name: ibex_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void ibex_send(struct uart_dev_s *dev, int ch)
{
  /* Wait until there is space for a byte in TX fifo */
  while(getreg32(UART0_STATUS) & UART_STATUS_TX_FULL_MASK);

  putreg32(ch, UART0_TX);
}

/****************************************************************************
 * Name: ibex_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static void ibex_txint(struct uart_dev_s *dev, bool enable)
{
  struct ibex_uart_s *priv = (struct ibex_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->tx_irq);
  else
    up_disable_irq(priv->tx_irq);
}

/****************************************************************************
 * Name: ibex_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/
static bool ibex_txready(struct uart_dev_s *dev)
{
  return !(getreg32(UART0_STATUS) & UART_STATUS_TX_FULL_MASK);
}

/****************************************************************************
 * Name: ibex_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/
static bool ibex_txempty(struct uart_dev_s *dev)
{
  return (getreg32(UART0_STATUS) & UART_STATUS_TX_EMPTY_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ibex_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/
void ibex_earlyserialinit(void)
{
  CONSOLE_DEV.isconsole = true;
  ibex_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: ibex_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/
void ibex_serialinit(void)
{
  /* Register the console */
  uart_register("/dev/console", &CONSOLE_DEV);
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
  ibex_earlyserialinit();
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
  ibex_serialinit();
}