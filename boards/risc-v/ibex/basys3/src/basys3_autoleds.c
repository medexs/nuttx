/****************************************************************************
 * boards/risc-v/ibex/basys3/src/basys3_autoleds.c
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
#include <arch/board/board.h>

#include "riscv_internal.h"

#include "hardware/ibex_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/
void board_autoled_initialize(void)
{
	// Turn all LEDs off
	// TODO: set only LED pins to 0, but the rest of GPIO_OUT has to be defined
	putreg32(GPIO_OUT, 0);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/
void board_autoled_on(int led)
{
	modifyreg32(GPIO_OUT, 0, led);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/
void board_autoled_off(int led)
{
	modifyreg32(GPIO_OUT, led, 0);
}