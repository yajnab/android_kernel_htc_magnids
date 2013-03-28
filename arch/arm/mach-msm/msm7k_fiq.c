/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <mach/clk.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <asm/fiq.h>
#include <asm/hardware/gic.h>

#include "msm_watchdog.h"

#define MODULE_NAME "msm7k_fiq_handler"


struct msm_watchdog_dump msm_dump_cpu_ctx;

int fiq_counter = 0;
/* Called from the FIQ bark handler */
void msm_7k_bark_fin(void)
{
       fiq_counter++;
       panic("MSM FIQ HANDLER\n");
}

struct fiq_handler msm_7k_fh = {
       .name = MODULE_NAME,
};

extern unsigned int msm_7k_fiq_start, msm_7k_fiq_length;
extern void msm_7k_fiq_setup(void *stack);

static int msm_setup_fiq_handler(void)
{
       int ret = 0;
       void *stack;
       claim_fiq(&msm_7k_fh);
       set_fiq_handler(&msm_7k_fiq_start, msm_7k_fiq_length);
       stack = (void *)__get_free_pages(GFP_KERNEL, THREAD_SIZE_ORDER);
       if (!stack) {
              pr_info("No free pages available - %s fails\n",
                     __func__);
              return -ENOMEM;
       }
       msm_7k_fiq_setup(stack);
       gic_set_irq_secure(MSM8625_INT_A9_M2A_2);
       printk("%s : setup_fiq_handler --done \n", __func__);
       return ret;
}

static int init_7k_fiq(void)
{
       printk("MSM Init FIQ\n");
       msm_setup_fiq_handler();
       return 0;
}

late_initcall(init_7k_fiq);
