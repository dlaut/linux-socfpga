/*
 * Microchip KSZ8863 series register access through PCP
 *
 * Copyright (C) 2020 Datalogic
 */

#include "linux/compiler.h"
#include "linux/kernel.h"
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/tty.h>
#include <linux/printk.h>
#include <linux/delay.h>

#include "ksz8.h"
#include "ksz_common.h"
#include "ksz8863_reg.h"


/*****************************************************************************/
/*                                 Constants                                 */
/*****************************************************************************/

#define KSZ8863_WRITE_SWITCH_COMMAND	0x2
#define KSZ8863_READ_SWITCH_COMMAND	0x3
#define KSZ8863_COMMAND_HEADER_SIZE	sizeof(u8) + sizeof(u8)	// command + reg address
#define KSZ8863_SEND_RETRIES		3


/*****************************************************************************/
/*                                  Typedefs                                 */
/*****************************************************************************/

struct ksz8863_pcp
{
	struct device           *dev;	// Associated device
	struct tty_struct	*tty;	// TTY used for PCP communication
};


/*****************************************************************************/
/*                                 Functions                                 */
/*****************************************************************************/

/* Utility functions *********************************************************/

static inline void get_termios(struct tty_struct *tty,
			       struct ktermios *out_termios)
{
	down_read(&tty->termios_rwsem);
	*out_termios = tty->termios;
	up_read(&tty->termios_rwsem);
}


/* Regmap functions **********************************************************/

/**
 * Write data with a locked line discipline
 */
static int ksz8863_pcp_do_write(struct ksz8863_pcp *pcp, struct tty_ldisc *ldisc,
				const u8 *buf, size_t count)
{
	struct device * dev = pcp->dev;
	int ret;

	ret = ldisc->ops->write(pcp->tty, NULL, buf, count);
	if (unlikely(ret != count))
	{
		dev_err(dev, "%s: Unable to send write request (%d != %d)", __FUNCTION__, ret, count);
		ret = -EIO;
	}
	else
		ret = 0;

	return ret;
}

/**
 * Read data with a locked line discipline
 */
static int ksz8863_pcp_do_read(struct ksz8863_pcp *pcp, struct tty_ldisc *ldisc,
			       u8 *buf, size_t count)
{
	struct device * dev = pcp->dev;
	int ret;

	ret = ldisc->ops->read(pcp->tty, NULL, buf, count);
	if (unlikely(ret != count))
	{
		dev_err(dev, "%s: Unable to read response (%d != %d)", __FUNCTION__, ret, count);
		ret = -EIO;
	}
	else
		ret = 0;

	return ret;
}

/**
 * Send a PCP command which expects a response with a locked line discipline
 */
static int ksz8863_pcp_send_command(struct ksz8863_pcp *pcp, struct tty_ldisc *ldisc,
				    u8 *cmd, size_t count)
{
	int ret, n_retries;
	bool done;
	u8 *resp = kmalloc(count, GFP_KERNEL);
	if (unlikely(!resp))
	{
		dev_err(pcp->dev, "%s: Unable to allocate buffer", __FUNCTION__);
		return -ENOMEM;
	}

	// Retransmit command if it fails
	for (n_retries = 0, done = false; !done && n_retries < KSZ8863_SEND_RETRIES; ++n_retries)
	{
		// Send cmd request
		ret = ksz8863_pcp_do_write(pcp, ldisc, cmd, count);
		if (ret)
			continue;

		// Read response
		ret = ksz8863_pcp_do_read(pcp, ldisc, resp, count);
		if (ret)
			continue;

		done = true;
	}

	if (likely(done))
	{
		// Copy response to cmd buffer
		memcpy(cmd, resp, count);
	}
	kfree(resp);

	return ret;
}

static int ksz8863_pcp_read(void *ctx, const void *reg_buf, size_t reg_len,
			    void *val_buf, size_t val_len)
{
	struct ksz_device *ksz_dev = (struct ksz_device *)ctx;
	struct ksz8863_pcp *pcp = ((struct ksz8*)ksz_dev->priv)->priv;
	struct device * dev = pcp->dev;
	struct tty_ldisc *ldisc;

	u8 *buff;
	size_t buffer_size = val_len + KSZ8863_COMMAND_HEADER_SIZE;

	u8 reg = *(u8 *)reg_buf;
	u8 *val = val_buf;
	int ret;

	// Allocate buffer for read request (data is set to 0)
	buff = kzalloc(buffer_size, GFP_KERNEL);
	if (unlikely(!buff))
	{
		dev_err(dev, "%s: Unable to allocate buffer", __FUNCTION__);
		return -ENOMEM;
	}

	// Format read request message
	buff[0] = KSZ8863_READ_SWITCH_COMMAND;
	buff[1] = reg;

	// Lock line discipline
	ldisc = tty_ldisc_ref(pcp->tty);
	if (unlikely(!ldisc))
	{
		dev_err(dev, "%s: Unable to lock ldisc", __FUNCTION__);
		ret = -ENODEV;
		goto free_buff;
	}

	ret = ksz8863_pcp_send_command(pcp, ldisc, buff, buffer_size);

	if (ret)
		goto ldisc_deref;

	// Copy to output buffer, excluding the header (0x0 0x0)
	memcpy(val, buff + KSZ8863_COMMAND_HEADER_SIZE, val_len);

  ldisc_deref:
	tty_ldisc_deref(ldisc);
  free_buff:
	kfree(buff);

	return ret;
}

static int ksz8863_pcp_write(void *ctx, const void *data, size_t count)
{
	struct ksz_device *ksz_dev = (struct ksz_device *)ctx;
	struct ksz8863_pcp *pcp = ((struct ksz8*)ksz_dev->priv)->priv;
	struct device * dev = pcp->dev;
	struct tty_ldisc *ldisc;

	// Data = reg + val[0] + ... val[count - sizeof(reg)]
	u32 reg = *(u32 *)data;
	u8 *val = (u8 *)(data + sizeof(reg));
	u8 *write_buf;
	size_t write_size;
	int ret;

	// Allocate a buffer for "count" bytes (without "reg") + command + reg address
	count -= sizeof(reg);
	write_size = count + KSZ8863_COMMAND_HEADER_SIZE;
	write_buf = kmalloc(write_size, GFP_KERNEL);
	if (unlikely(!write_buf))
	{
		dev_err(dev, "%s: Unable to allocate buffer", __FUNCTION__);
		return -ENOMEM;
	}

	// Format message
	write_buf[0] = KSZ8863_WRITE_SWITCH_COMMAND;
	write_buf[1] = reg;
	memcpy(&write_buf[2], val, count);

	// Lock line discipline
	ldisc = tty_ldisc_ref(pcp->tty);
	if (unlikely(!ldisc))
	{
		dev_err(dev, "%s: Unable to lock ldisc", __FUNCTION__);
		ret = -ENODEV;
		goto free_buff;
	}

	// Send write command and discard response
	ret = ksz8863_pcp_send_command(pcp, ldisc, write_buf, write_size);

	tty_ldisc_deref(ldisc);

  free_buff:
	kfree(write_buf);

	return ret;
}

static const struct regmap_bus regmap_pcp[] = {
	{
		.read = ksz8863_pcp_read,
		.write = ksz8863_pcp_write,
		.max_raw_read = 1,
		.max_raw_write = 1,
	},
	{
		.read = ksz8863_pcp_read,
		.write = ksz8863_pcp_write,
		.val_format_endian_default = REGMAP_ENDIAN_BIG,
		.max_raw_read = 2,
		.max_raw_write = 2,
	},
	{
		.read = ksz8863_pcp_read,
		.write = ksz8863_pcp_write,
		.val_format_endian_default = REGMAP_ENDIAN_BIG,
		.max_raw_read = 4,
		.max_raw_write = 4,
	}
};

static const struct regmap_config ksz8863_regmap_pcp_config[] = {
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


/* Platform device functions *************************************************/

static int __init ksz8863_pcp_init_regmap(struct ksz_device *ksz_dev)
{
	struct device *dev = ((struct ksz8863_pcp*)((struct ksz8*)ksz_dev->priv)->priv)->dev;
	struct regmap_config rc;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ksz8863_regmap_pcp_config); i++)
	{
		rc = ksz8863_regmap_pcp_config[i];
		rc.lock_arg = &ksz_dev->regmap_mutex;
		ksz_dev->regmap[i] = devm_regmap_init(dev,
						      &regmap_pcp[i], ksz_dev,
						      &rc);
		if (unlikely(IS_ERR(ksz_dev->regmap[i])))
		{
			ret = PTR_ERR(ksz_dev->regmap[i]);
			dev_err(dev,
				"Failed to initialize regmap%i: %d",
				ksz8863_regmap_pcp_config[i].val_bits, ret);
			return ret;
		}
	}

	return 0;
}

/**
 * Allocate device private data.
 */
static int __init ksz8863_pcp_init_private_data(struct platform_device *op)
{
	struct device *pdev = &op->dev;
	struct ksz_device *ksz_dev;
	struct ksz8 *ksz8;
	struct ksz8863_pcp *pcp;

	// Allocate device managed memory for driver private structure
	ksz8 = devm_kzalloc(pdev, sizeof(*ksz8), GFP_KERNEL);
	if (unlikely(!ksz8))
	{
		dev_err(pdev, "Unable to allocate ksz8 private structure");
		return -ENOMEM;
	}
	pcp = devm_kzalloc(pdev, sizeof(*pcp), GFP_KERNEL);
	if (unlikely(!pcp))
	{
		dev_err(pdev, "Unable to allocate pcp private structure");
		return -ENOMEM;
	}

	// Set the platform device as the device reference
	pcp->dev = pdev;
	// Set ksz8863_pcp as ksz8 private data
	ksz8->priv = pcp;

	// Allocate ksz device
	ksz_dev = ksz_switch_alloc(&op->dev, ksz8);
	if (unlikely(!ksz_dev))
	{
		dev_err(pdev, "Unable to allocate ksz device");
		return -EINVAL;
	}

	// Save ksz device as platform driver data
	platform_set_drvdata(op, ksz_dev);

	// Get ksz_platform_data if present
	if (op->dev.platform_data)
		ksz_dev->pdata = op->dev.platform_data;

	return 0;
}

/**
 * Open TTY and set PCP ldisc.
 */
static int __init ksz8863_pcp_init_tty(struct platform_device *op)
{
	struct device *pdev = &op->dev;
	struct ksz_device *ksz_dev = platform_get_drvdata(op);
	struct tty_struct *tty;
	struct ktermios tmp_termios;
	dev_t dev;
	int ret;

	dev_dbg(pdev, "%s: start", __FUNCTION__);

	// Retrieve TTY
	/* TODO: read from dtb */
	ret = tty_dev_name_to_number("ttyS0", &dev);
	if (unlikely(ret))
	{
		dev_err(pdev, "%s: Unable to retrieve TTY", __FUNCTION__);
		return ret;
	}

	// Open TTY
	tty = tty_kopen(dev);
	if (unlikely(IS_ERR(tty)))
	{
		dev_err(pdev, "%s: Unable to kopen TTY", __FUNCTION__);
		return PTR_ERR(tty);
	}

	if (tty->ops->open)
		ret = tty->ops->open(tty, NULL);
	else
		ret = -ENODEV;

	if (unlikely(ret))
	{
		dev_err(pdev, "%s: Unable to open TTY", __FUNCTION__);
		tty_unlock(tty);
		tty_kclose(tty);
		return ret;
	}

	clear_bit(TTY_HUPPED, &tty->flags);

	get_termios(tty, &tmp_termios);
	tty_termios_encode_baud_rate(&tmp_termios, 115200, 115200);
	tty_set_termios(tty, &tmp_termios);

	tty_unlock(tty);

	// Set ldisc on tty
	ret = tty_set_ldisc(tty, N_PCP);
	if (unlikely(ret))
	{
		dev_err(pdev, "%s: Failed to set N_PCP on TTY", __FUNCTION__);
		tty_lock(tty);
		if (tty->ops->close)
			tty->ops->close(tty, NULL);
		tty_unlock(tty);
		tty_kclose(tty);
		return ret;
	}

	// Save TTY reference in private struct.
	((struct ksz8863_pcp*)((struct ksz8*)ksz_dev->priv)->priv)->tty = tty;

	dev_dbg(pdev, "%s: end", __FUNCTION__);

	return ret;
}

/**
 * Lock and close TTY (both kernel and user side)
 */
static void ksz8863_pcp_close_tty(struct ksz_device *ksz_dev)
{
	struct tty_struct *tty = ((struct ksz8863_pcp*)((struct ksz8*)ksz_dev->priv)->priv)->tty;

	tty_lock(tty);

	if (tty->ops->close)
		tty->ops->close(tty, NULL);

	tty_ldisc_flush(tty);
	tty_unlock(tty);
	tty_kclose(tty);
}

/**
 * Disable the autodetection mode on the EBC Connection Box
 */
static int __init ksz8863_pcp_disable_autodetect(struct ksz_device *ksz_dev)
{
	int ret;
	struct ksz8863_pcp *pcp = ((struct ksz8*)ksz_dev->priv)->priv;
	const u8 message[] = {0x0E, 0x0B, 0x0C, 0xDA};
	struct tty_ldisc *ldisc = tty_ldisc_ref(pcp->tty);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));

	tty_ldisc_deref(ldisc);

	return ret;
}

/**
 * Send a reset command to the CPLD
 */
static int __init ksz8863_pcp_reset_cpld(struct ksz_device *ksz_dev)
{
	int ret;
	struct ksz8863_pcp *pcp = ((struct ksz8*)ksz_dev->priv)->priv;
	const u8 message = 0x08;
	struct tty_ldisc *ldisc = tty_ldisc_ref(pcp->tty);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	ret = ksz8863_pcp_do_write(pcp, ldisc, &message, sizeof(message));

	tty_ldisc_deref(ldisc);

	// Wait for reset completion
	msleep(1);

	return ret;
}

static int __init ksz8863_pcp_probe(struct platform_device *op)
{
	struct device *pdev = &op->dev;
	struct ksz_device *ksz_dev;
	int ret;

	dev_dbg(pdev, "%s: start", __FUNCTION__);

	ret = ksz8863_pcp_init_private_data(op);
	if (unlikely(ret))
		return ret;

	ksz_dev = platform_get_drvdata(op);

	ret = ksz8863_pcp_init_regmap(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_init_tty(op);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_disable_autodetect(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_reset_cpld(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8_switch_register(ksz_dev);

	if (ret)
	{
		dev_err(pdev, "%s: Unable to register switch device", __FUNCTION__);
		ksz8863_pcp_close_tty(ksz_dev);
		return ret;
	}

	dev_dbg(pdev, "%s: end", __FUNCTION__);

	return 0;
}

static int ksz8863_pcp_remove(struct platform_device *op)
{
	struct ksz_device *ksz_dev = platform_get_drvdata(op);

	ksz_switch_remove(ksz_dev);

	ksz8863_pcp_close_tty(ksz_dev);

	return 0;
}

static const struct of_device_id ksz8863_dt_ids[] = {
	{ .compatible = "microchip,ksz8863" },
	{ .compatible = "microchip,ksz8873" },
	{ },
};
MODULE_DEVICE_TABLE(of, ksz8863_dt_ids);

static struct platform_driver _platform_driver = {
  .remove = __exit_p(ksz8863_pcp_remove),
  .driver = {
    .name = "ksz8863-switch",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(ksz8863_dt_ids),
  },
};

module_platform_driver_probe(_platform_driver, ksz8863_pcp_probe);

MODULE_AUTHOR("Filippo Franzoni <filippo.franzoni@datalogic.com>");
MODULE_DESCRIPTION("Microchip KSZ8863 PCP Switch driver");
MODULE_LICENSE("GPL v2");
