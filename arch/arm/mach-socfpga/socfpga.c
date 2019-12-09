// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2012-2015 Altera Corporation
 */
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/reboot.h>
#include <linux/reset/socfpga.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/pmu.h>
#include <asm/cacheflush.h>

#include "core.h"
#include "socfpga_cti.h"

void __iomem *sys_manager_base_addr;
void __iomem *rst_manager_base_addr;
void __iomem *sdr_ctl_base_addr;
unsigned long socfpga_cpu1start_addr;
void __iomem *clkmgr_base_addr;

#ifdef CONFIG_HW_PERF_EVENTS
static struct arm_pmu_platdata socfpga_pmu_platdata = {
	.handle_irq = socfpga_pmu_handler,
	.init = socfpga_init_cti,
	.start = socfpga_start_cti,
	.stop = socfpga_stop_cti,
};
#endif

static const struct of_dev_auxdata socfpga_auxdata_lookup[] __initconst = {
#ifdef CONFIG_HW_PERF_EVENTS
	OF_DEV_AUXDATA("arm,cortex-a9-pmu", 0, "arm-pmu", &socfpga_pmu_platdata),
#endif
	{ /* sentinel */ }
};

static void __init socfpga_sysmgr_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "altr,sys-mgr");

	if (of_property_read_u32(np, "cpu1-start-addr",
			(u32 *) &socfpga_cpu1start_addr))
		pr_err("SMP: Need cpu1-start-addr in device tree.\n");

	/* Ensure that socfpga_cpu1start_addr is visible to other CPUs */
	smp_wmb();
	sync_cache_w(&socfpga_cpu1start_addr);

	sys_manager_base_addr = of_iomap(np, 0);

	np = of_find_compatible_node(NULL, NULL, "altr,rst-mgr");
	rst_manager_base_addr = of_iomap(np, 0);

	np = of_find_compatible_node(NULL, NULL, "altr,clk-mgr");
	clkmgr_base_addr = of_iomap(np, 0);
	WARN_ON(!clkmgr_base_addr);

	np = of_find_compatible_node(NULL, NULL, "altr,sdr-ctl");
	sdr_ctl_base_addr = of_iomap(np, 0);
}

static void __init socfpga_init_irq(void)
{
	irqchip_init();
	socfpga_sysmgr_init();
	if (IS_ENABLED(CONFIG_EDAC_ALTERA_L2C))
		socfpga_init_l2_ecc();

	if (IS_ENABLED(CONFIG_EDAC_ALTERA_OCRAM))
		socfpga_init_ocram_ecc();
	socfpga_reset_init();
}

static void __init socfpga_arria10_init_irq(void)
{
	irqchip_init();
	socfpga_sysmgr_init();
	if (IS_ENABLED(CONFIG_EDAC_ALTERA_L2C))
		socfpga_init_arria10_l2_ecc();
	if (IS_ENABLED(CONFIG_EDAC_ALTERA_OCRAM))
		socfpga_init_arria10_ocram_ecc();
	socfpga_reset_init();
}

static void socfpga_cyclone5_restart(enum reboot_mode mode, const char *cmd)
{
	u32 temp;

	/* Turn on all periph PLL clocks */
	writel(0xffff, clkmgr_base_addr + SOCFPGA_ENABLE_PLL_REG);

	temp = readl(rst_manager_base_addr + SOCFPGA_RSTMGR_CTRL);

	if (mode == REBOOT_HARD)
		temp |= RSTMGR_CTRL_SWCOLDRSTREQ;
	else
		temp |= RSTMGR_CTRL_SWWARMRSTREQ;
	writel(temp, rst_manager_base_addr + SOCFPGA_RSTMGR_CTRL);
}

static void socfpga_arria10_restart(enum reboot_mode mode, const char *cmd)
{
	u32 temp;

	temp = readl(rst_manager_base_addr + SOCFPGA_A10_RSTMGR_CTRL);

	if (mode == REBOOT_HARD)
		temp |= RSTMGR_CTRL_SWCOLDRSTREQ;
	else
		temp |= RSTMGR_CTRL_SWWARMRSTREQ;
	writel(temp, rst_manager_base_addr + SOCFPGA_A10_RSTMGR_CTRL);
}

static void __init socfpga_cyclone5_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table,
			     socfpga_auxdata_lookup, NULL);
}

static const char *altera_dt_match[] = {
	"altr,socfpga",
	NULL
};

DT_MACHINE_START(SOCFPGA, "Altera SOCFPGA")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.init_irq	= socfpga_init_irq,
	.init_machine	= socfpga_cyclone5_init,
	.restart	= socfpga_cyclone5_restart,
	.dt_compat	= altera_dt_match,
MACHINE_END

static const char *altera_a10_dt_match[] = {
	"altr,socfpga-arria10",
	NULL
};

DT_MACHINE_START(SOCFPGA_A10, "Altera SOCFPGA Arria10")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.init_irq	= socfpga_arria10_init_irq,
	.restart	= socfpga_arria10_restart,
	.dt_compat	= altera_a10_dt_match,
MACHINE_END
