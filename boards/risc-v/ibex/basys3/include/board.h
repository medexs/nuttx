/****************************************************************************
 * boards/risc-v/ibex/basys3/include/board.h
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

#ifndef __BOARDS_RISCV_IBEX_BASYS3_INCLUDE_BOARD_H
#define __BOARDS_RISCV_IBEX_BASYS3_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// Basys3 LEDs
#define BOARD_LED_CNT       16
#define BOARD_ALL_LED_MASK  ((BOARD_LED_CNT == 32) ? 0xFFFFFFFF : (((uint32_t)1 << BOARD_LED_CNT) - 1))
#define BOARD_LED0_MASK     0x1
#define BOARD_LED_MASK(n)   (BOARD_LED0_MASK << n)

// NuttX Event LEDs
#define LED_CPU             BOARD_LED_MASK(0)
#define LED_STARTED         BOARD_LED_MASK(1)
#define LED_HEAPALLOCATE    BOARD_LED_MASK(2)
#define LED_IRQSENABLED     BOARD_LED_MASK(3)
#define LED_STACKCREATED    BOARD_LED_MASK(4)
#define LED_INIRQ           BOARD_LED_MASK(5)
#define LED_SIGNAL          BOARD_LED_MASK(6)
#define LED_ASSERTION       BOARD_LED_MASK(7)
#define LED_PANIC           BOARD_LED_MASK(8)

#endif /* __BOARDS_RISCV_IBEX_BASYS3_INCLUDE_BOARD_H */

