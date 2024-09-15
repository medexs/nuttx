/****************************************************************************
 * arch/risc-v/src/ibex/ibex_irq.c
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

#include "ibex_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   Process interrupt and its callback function.
 *
 * Input Parameters:
 *   mcause - RISC-V "mcause" register.
 *   regs   - Saved registers reference.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void * riscv_dispatch_irq(uintptr_t mcause, uintreg_t * regs)
{
  /* Get exception code */
  int irq = mcause & RISCV_IRQ_MASK;

  /* If current is interrupt and not exception */
  if (mcause & RISCV_IRQ_BIT)
    /* In NuttX vector table, IRQ's are located at RISCV_IRQ_ASYNC and beyond */
    irq += RISCV_IRQ_ASYNC;

  /* Deliver the IRQ */
  regs = riscv_doirq(irq, regs);

  return regs;
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/
void up_irqinitialize(void)
{
  /* Disable Machine interrupts */
  up_irq_save();

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Colorize the interrupt stack for debug purposes */
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Attach the common interrupt handler */
  riscv_exception_attach();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/
irqstate_t up_irq_enable(void)
{
  /* Read mstatus & set machine interrupt enable (MIE) in mstatus */
  irqstate_t flags = READ_AND_SET_CSR(CSR_MSTATUS, MSTATUS_MIE);
  return flags;
}