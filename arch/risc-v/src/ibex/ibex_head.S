/****************************************************************************
 * arch/risc-v/src/ibex/ibex_head.S
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
#include "ibex_vectors.S"

#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/
/* Imported symbols */
.extern __vector_table

.global __start

/****************************************************************************
 * Section: .text
 ****************************************************************************/
.section .text

/****************************************************************************
 * Name: __start
 ****************************************************************************/
__start:
	/* Set gp pointer */
  .option push /* Save current options which are later restored */
  .option norelax
  la gp, __global_pointer$
	.option pop  /* Restore saved options */

  /* Set stack pointer to the idle thread stack */
  la sp, _ebss
  li t0, SMP_STACK_SIZE
  add sp, sp, t0

  /* Disable all interrupts (i.e. timer, external) in mstatus */
  csrw CSR_MSTATUS, zero

  /* Initialize the Machine Trap Vector */
  la t0, __vector_table
  csrw CSR_MTVEC, t0

  /* Jump to __ibex_start */
  jal  x1, __ibex_start

  /* We shouldn't return from __ibex_start */
  ret