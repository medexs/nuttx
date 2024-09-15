/****************************************************************************
 * arch/risc-v/include/ibex/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_IBEX_IRQ_H
#define __ARCH_RISCV_INCLUDE_IBEX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQs and exception counts */

/* IRQs: SW + timer + external + 15 fast local + non-maskable */
#define IBEX_NR_IRQS (1 + 1 + 1 + 15 + 1)
/* Number of available IRQ's - needed by NuttX */
#define NR_IRQS (IBEX_NR_IRQS + RISCV_IRQ_ASYNC)

/* IRQ numbers assignments */
#define UART0_RX_IRQ 16
#define UART0_TX_IRQ 17

#endif /* __ARCH_RISCV_INCLUDE_IBEX_IRQ_H */
