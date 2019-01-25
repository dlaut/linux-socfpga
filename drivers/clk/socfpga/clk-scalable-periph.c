#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_address.h>

#include "clk.h"

#define DL_SOCFPGA_MPUCLK_COMPATIBLE  "datalogic,socfpga-scalable-mpuclock"

#define to_socfpga_periph_clk(p) container_of(p, struct socfpga_periph_clk, hw.hw)

/*****************************************************************************/
/*                             MPU clock section                             */
/*****************************************************************************/

static unsigned long clk_mpuclk_recalc_rate(struct clk_hw *hwclk,
					unsigned long parent_rate)
{
	struct socfpga_periph_clk *socfpgaclk = to_socfpga_periph_clk(hwclk);
	u32 div, val;

	if (socfpgaclk->fixed_div) {
		div = socfpgaclk->fixed_div;
	} else {
		if (socfpgaclk->div_reg) {
			val = readl(socfpgaclk->div_reg) >> socfpgaclk->shift;
			val &= div_mask(socfpgaclk->width);
			parent_rate /= (val + 1);
		}
		div = ((readl(socfpgaclk->hw.reg) & 0x1ff) + 1);
	}

	return parent_rate / div;
}

static u8 clk_mpuclk_get_parent(struct clk_hw *hwclk)
{
	u32 clk_src;

	clk_src = readl(clk_mgr_base_addr + CLKMGR_DBCTRL);
	return clk_src & 0x1;
}

static int clk_mpuclk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	return 0;
}

static long clk_mpuclk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	u32 val = 1;
	u32 div = 1;
	struct socfpga_periph_clk *socfpgaclk = to_socfpga_periph_clk(hw);

	if (socfpgaclk->fixed_div) {
		div = socfpgaclk->fixed_div;
	}
	else {
		if (socfpgaclk->div_reg) {
			val = readl(socfpgaclk->div_reg) >> socfpgaclk->shift;
			val &= div_mask(socfpgaclk->width);
			val++;
		}
		div = ((readl(socfpgaclk->hw.reg) & 0x1ff) + 1);
	}

	div = *prate / rate;

	// Change divisor register to reach the new clock rate.
	writel(div - 1, socfpgaclk->hw.reg);

	return *prate / (val * div);
}

static const struct clk_ops mpuclk_ops = {
	.recalc_rate = clk_mpuclk_recalc_rate,
	.get_parent = clk_mpuclk_get_parent,
	.set_rate = clk_mpuclk_set_rate,
	.round_rate = clk_mpuclk_round_rate,
};

static __init void _clk_mpuclock_init(struct device_node *clk_np)
{
	u32 reg;
	struct clk *clk;
	struct socfpga_periph_clk *periph_clk;
	const char *clk_name = clk_np->name;
	const char *parent_name[SOCFPGA_MAX_PARENTS];
	struct clk_init_data init;
	int rc;
	u32 fixed_div;
	u32 div_reg[3];

	of_property_read_u32(clk_np, "reg", &reg);

	periph_clk = kzalloc(sizeof(*periph_clk), GFP_KERNEL);
	if (WARN_ON(!periph_clk))
		return;

	periph_clk->hw.reg = clk_mgr_base_addr + reg;

	rc = of_property_read_u32_array(clk_np, "div-reg", div_reg, 3);
	if (!rc) {
		periph_clk->div_reg = clk_mgr_base_addr + div_reg[0];
		periph_clk->shift = div_reg[1];
		periph_clk->width = div_reg[2];
	} else {
		periph_clk->div_reg = 0;
	}

	rc = of_property_read_u32(clk_np, "fixed-divider", &fixed_div);
	if (rc)
		periph_clk->fixed_div = 0;
	else
		periph_clk->fixed_div = fixed_div;

	of_property_read_string(clk_np, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &mpuclk_ops;
	init.flags = 0;

	init.num_parents = of_clk_parent_fill(clk_np, parent_name,
					      SOCFPGA_MAX_PARENTS);
	init.parent_names = parent_name;

	periph_clk->hw.hw.init = &init;

	clk = clk_register(NULL, &periph_clk->hw.hw);
	if (WARN_ON(IS_ERR(clk))) {
		kfree(periph_clk);
		return;
	}
	rc = of_clk_add_provider(clk_np, of_clk_src_simple_get, clk);
}

static void __init _socfpga_scalable_periph_init(struct device_node *node)
{
	_clk_mpuclock_init(node);
}

CLK_OF_DECLARE(socfpga_scalable_perip_clk, DL_SOCFPGA_MPUCLK_COMPATIBLE, _socfpga_scalable_periph_init);

MODULE_DESCRIPTION("Scalable clock driver");
MODULE_AUTHOR("Filippo Franzoni");
MODULE_LICENSE("GPL");
