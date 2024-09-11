/****************************************************************************
 * arch/risc-v/src/ibex/ibex_vectors.S
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

.section .text
.balign 256
.global  __vector_table

/****************************************************************************
 * Name: __vector_table
 *
 * Description:
 *   Like in riscv_vectors.s, all M-mode exceptions and interrupts will be
 *   handled from here. If kernel is in S-mode delegated exceptions and
 *   interrupts are handled. Label '__vector_table' is aligned to 256 because
 *   Ibex needs contents of mtvec aligned to 256B. The vector table consists
 *   of 32 same vectors, because for now, Ibex can only have mtvec.mode set to
 *   vectored.
 ****************************************************************************/
__vector_table:
	.option push
  .option norvc

  .rept (32)
  j exception_common
  .endr

