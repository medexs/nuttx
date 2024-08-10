/****************************************************************************
 * arch/risc-v/src/ibex/ibex_gpio.c
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
#include "riscv_internal.h"

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ibex_gpio_write
 *
 * Description:
 *   Write data to GPIO register at addr
 *
 ****************************************************************************/
void ibex_gpio_write(uint32_t addr, uint32_t data)
{
	putreg32(data, addr);
}

/****************************************************************************
 * Name: ibex_gpio_read
 *
 * Description:
 *   Return data from GPIO register at addr
 *
 ****************************************************************************/
uint32_t ibex_gpio_read(uint32_t addr)
{
	return getreg32(addr);
}