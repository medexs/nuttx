/****************************************************************************
 * boards/risc-v/ibex/nexys-video/src/ibex_boardinit.c
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

#include <syslog.h>

#include "riscv_internal.h"

#include "ibex.h"
#include "hardware/ibex_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
 
/****************************************************************************
 * Name: ibex_board_initialize
 *
 * Description:
 *   Initialize board
 *
 ****************************************************************************/
void ibex_board_initialize(void)
{
  /* Set OLED control signals (active-low) */
  uint32_t set_mask = BOARD_OLED_VDD_N_MASK | BOARD_OLED_RST_N_MASK |
                      BOARD_OLED_VBAT_N_MASK;
  /* Clear other signals */
  uint32_t clear_mask = UINT32_MAX - set_mask;
  modifyreg32(IBEX_GPIO_OUT, clear_mask, set_mask);
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/
int board_app_initialize(uintptr_t arg)
{
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */
  return OK;
#else
  /* Perform board-specific initialization */
  return ibex_bringup();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void board_late_initialize(void)
{
  /* Perform board-specific initialization */
  if(ibex_bringup() != OK)
  {
    syslog(LOG_ERR, "ERROR: Failed to bring up Ibex.\n");
    PANIC();
  }

}