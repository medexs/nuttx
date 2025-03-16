/****************************************************************************
 * boards/risc-v/ibex-ds/nexys-video/src/ibex_ds_ssd1306.c
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

#include <debug.h>

#include "riscv_internal.h"

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)
#  include <nuttx/video/fb.h>
#endif

#include "hardware/ibex_ds_gpio.h"
#include "ibex_ds_spi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct lcd_dev_s *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/
int board_lcd_initialize(void)
{
  /* Power up and reset OLED */
  modifyreg32(IBEX_DS_GPIO_OUT, BOARD_OLED_VDD_N_MASK, 0);
  usleep(1 * 1000);
  modifyreg32(IBEX_DS_GPIO_OUT, BOARD_OLED_RST_N_MASK, 0);
  usleep(3);
  modifyreg32(IBEX_DS_GPIO_OUT, 0, BOARD_OLED_RST_N_MASK);
  modifyreg32(IBEX_DS_GPIO_OUT, BOARD_OLED_VBAT_N_MASK, 0);
  usleep(100 * 1000);

  /* Initialize SPI */
  struct spi_dev_s *spi = ibex_ds_spibus_initialize(0);
  if (!spi)
  {
    lcderr("board_lcd_initialize(): Failed to initialize SPI port.\n");
    return -ENODEV;
  }

  /* Bind the SPI port to the OLED */
  g_lcddev = ssd1306_initialize(spi, NULL, 0);
  if (!g_lcddev)
  {
    lcderr("board_lcd_initialize(): Failed to bind SPI port to OLED.\n");
    return -ENODEV;
  }

  /* Turn the OLED on */
  lcdinfo("board_lcd_initialize(): Bound SPI port to OLED.\n");
  g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 ****************************************************************************/
struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  return lcddev == 0 ? g_lcddev : NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 ****************************************************************************/
void board_lcd_uninitialize(void)
{
  /* Power down OLED */
  modifyreg32(IBEX_DS_GPIO_OUT, 0, BOARD_OLED_VBAT_N_MASK);
  usleep(100 * 1000);
  modifyreg32(IBEX_DS_GPIO_OUT, 0, BOARD_OLED_VDD_N_MASK);
}