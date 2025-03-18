/****************************************************************************
 * arch/risc-v/src/ibex-ds/hardware/ibex_ds_uart.h
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

#ifndef __ARCH_RISCV_SRC_IBEX_DS_HARDWARE_IBEX_DS_UART_H
#define __ARCH_RISCV_SRC_IBEX_DS_HARDWARE_IBEX_DS_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/
#define IBEX_DS_UART0_BASE        0x80001000
#define IBEX_DS_UART0_RX_REG      0x0
#define IBEX_DS_UART0_TX_REG      0x4
#define IBEX_DS_UART0_STATUS_REG  0x8
#define IBEX_DS_UART0_RX          (IBEX_DS_UART0_BASE + IBEX_DS_UART0_RX_REG)
#define IBEX_DS_UART0_TX          (IBEX_DS_UART0_BASE + IBEX_DS_UART0_TX_REG)
#define IBEX_DS_UART0_STATUS      (IBEX_DS_UART0_BASE + IBEX_DS_UART0_STATUS_REG)

#define IBEX_DS_UART_STATUS_RX_EMPTY_MASK  0x1
#define IBEX_DS_UART_STATUS_TX_FULL_MASK   0x2
#define IBEX_DS_UART_STATUS_TX_EMPTY_MASK  0x4

#endif /* __ARCH_RISCV_SRC_IBEX_DS_HARDWARE_IBEX_DS_UART_H */