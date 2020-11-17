/*
 * Microchip KSZ8863 series register access through UART
 *
 * Copyright (C) 2020 Datalogic
 */

#include "linux/device.h"
#include "linux/platform_device.h"
#include "linux/printk.h"

#include "ksz8.h"
#include "ksz_common.h"
#include "ksz8863_reg.h"

/* Dummy communication driver to set the required registers to default values.
 *
 */

static int ksz8863_dummy_read(void *ctx, const void *reg_buf, size_t reg_len,
			     void *val_buf, size_t val_len)
{
	struct ksz_device *dev = (struct ksz_device *)ctx;
	u8 reg = *(u8 *)reg_buf;
	u8 *val = val_buf;
	int ret = 0;
	int i;

	dev_err(dev->dev, "%s: reg=%d; reg_len=%zu; val_len=%zu", __FUNCTION__, reg, reg_len, val_len);
	/* dump_stack(); */
	/* dev_err(dev->dev, "------------------"); */

	if (reg == 0 && reg_len == 4 && val_len == 2) /* Chip ID0 and Chip ID1 */
	{
		val[0] = KSZ88_FAMILY_ID;
		val[1] = CHIP_ID_63;
	}
	else if ((reg == 30 || reg == 46) && reg_len == 4 && val_len == 1) /* Port 1 and 2 Status 0 */
	{
		val[0] = 0xEF;
	}
	else if ((reg == 31 || reg == 47) && reg_len == 4 && val_len == 1) /* Port 1 and 2 Status 1 */
	{
		val[0] = 0xA6;
	}
	else if ((reg == 28 || reg == 44) && reg_len == 4 && val_len == 1) /* Port 1 and 2 Control 12 */
	{
		val[0] = 0xBF;
	}
	else if (reg == 50 && reg_len == 4 && val_len == 1) /* Port 3 Control 2 */
	{
		val[0] = 0x06;
	}
	else
	{
		for (i = 0; i < val_len; i++) {
			val[i] = 0;
		}
	}

	ret = 0;
	return ret;
}

static int ksz8863_dummy_write(void *ctx, const void *data, size_t count)
{
	struct ksz_device *dev = (struct ksz_device *)ctx;
	u8 *val = (u8 *)(data + 4);
	u32 reg = *(u32 *)data;
	int ret = 0;

	dev_err(dev->dev, "%s: reg=%d; count=%zu", __FUNCTION__, reg, count);
	print_hex_dump_bytes("val: ", DUMP_PREFIX_NONE, val, count);
	/* dump_stack(); */
	/* dev_err(dev->dev, "------------------"); */

	return ret;
}

static const struct regmap_bus regmap_uart[] = {
	{
		.read = ksz8863_dummy_read,
		.write = ksz8863_dummy_write,
		.max_raw_read = 1,
		.max_raw_write = 1,
	},
	{
		.read = ksz8863_dummy_read,
		.write = ksz8863_dummy_write,
		.val_format_endian_default = REGMAP_ENDIAN_BIG,
		.max_raw_read = 2,
		.max_raw_write = 2,
	},
	{
		.read = ksz8863_dummy_read,
		.write = ksz8863_dummy_write,
		.val_format_endian_default = REGMAP_ENDIAN_BIG,
		.max_raw_read = 4,
		.max_raw_write = 4,
	}
};

static const struct regmap_config ksz8863_regmap_config[] = {
	{
		.name = "#8",
		.reg_bits = 8,
		.pad_bits = 24,
		.val_bits = 8,
		.cache_type = REGCACHE_NONE,
		.use_single_read = 1,
		.lock = ksz_regmap_lock,
		.unlock = ksz_regmap_unlock,
	},
	{
		.name = "#16",
		.reg_bits = 8,
		.pad_bits = 24,
		.val_bits = 16,
		.cache_type = REGCACHE_NONE,
		.use_single_read = 1,
		.lock = ksz_regmap_lock,
		.unlock = ksz_regmap_unlock,
	},
	{
		.name = "#32",
		.reg_bits = 8,
		.pad_bits = 24,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
		.use_single_read = 1,
		.lock = ksz_regmap_lock,
		.unlock = ksz_regmap_unlock,
	}
};

static int ksz8863_uart_probe(struct platform_device *op)
{
	struct regmap_config rc;
	struct ksz_device *dev;
	struct ksz8 *ksz8;
	int ret;
	int i;

	dev_err(&op->dev, "%s: start", __FUNCTION__);

	ksz8 = devm_kzalloc(&op->dev, sizeof(struct ksz8), GFP_KERNEL);
	ksz8->priv = op;

	dev = ksz_switch_alloc(&op->dev, ksz8);
	if (!dev)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ksz8863_regmap_config); i++) {
		rc = ksz8863_regmap_config[i];
		rc.lock_arg = &dev->regmap_mutex;
		dev->regmap[i] = devm_regmap_init(&op->dev,
						  &regmap_uart[i], dev,
						  &rc);
		if (IS_ERR(dev->regmap[i])) {
			ret = PTR_ERR(dev->regmap[i]);
			dev_err(&op->dev,
				"Failed to initialize regmap%i: %d\n",
				ksz8863_regmap_config[i].val_bits, ret);
			return ret;
		}
	}

	if (op->dev.platform_data)
		dev->pdata = op->dev.platform_data;

	ret = ksz8_switch_register(dev);

	/* Main DSA driver may not be started yet. */
	if (ret)
		return ret;

	platform_set_drvdata(op, dev);

	dev_err(&op->dev, "%s: end", __FUNCTION__);

	return 0;
}

static int ksz8863_uart_remove(struct platform_device *op)
{
	struct ksz_device *dev = platform_get_drvdata(op);

	if (dev)
		ksz_switch_remove(dev);

	return 0;
}

static const struct of_device_id ksz8863_dt_ids[] = {
	{ .compatible = "microchip,ksz8863" },
	{ .compatible = "microchip,ksz8873" },
	{ },
};
MODULE_DEVICE_TABLE(of, ksz8863_dt_ids);

static struct platform_driver _platform_driver = {
  .remove = __exit_p(ksz8863_uart_remove),
  .driver = {
    .name = "ksz8863-switch",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(ksz8863_dt_ids),
  },
};

module_platform_driver_probe(_platform_driver, ksz8863_uart_probe);

MODULE_AUTHOR("Filippo Franzoni <filippo.franzoni@datalogic.com>");
MODULE_DESCRIPTION("Microchip KSZ8863 UART Switch driver");
MODULE_LICENSE("GPL v2");
