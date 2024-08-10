/****************************************************************************
 * arch/risc-v/src/ibex/hardware/ibex_gpio.h
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

#ifndef __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_GPIO_H
#define __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/
#define GPIO_BASE        0x80000000
#define GPIO_OUT_REG     0x0
#define GPIO_IN_REG      0x4
#define GPIO_IN_DBNC_REG 0x8
#define GPIO_OUT         (GPIO_BASE + GPIO_OUT_REG)
#define GPIO_IN          (GPIO_BASE + GPIO_IN_REG)
#define GPIO_IN_DBNC     (GPIO_BASE + GPIO_IN_DBNC_REG)

#endif /* __ARCH_RISCV_SRC_IBEX_HARDWARE_IBEX_GPIO_H */