/*
 * Microchip KSZ8863 series register access through PCP
 *
 * Copyright (C) 2020 Datalogic
 */

#include "linux/compiler.h"
#include "linux/kernel.h"
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/tty.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include "ksz8.h"
#include "ksz_common.h"
#include "ksz8863_reg.h"


/*****************************************************************************/
/*                                 Constants                                 */
/*****************************************************************************/

/* Commands ******************************************************************/
#define KSZ8863_WRITE_SWITCH_COMMAND	0x2
#define KSZ8863_READ_SWITCH_COMMAND	0x3
#define KSZ8863_QUERY_VERSION_COMMAND	0x4
#define KSZ8863_RESET_CPLD_COMMAND	0x8
#define KSZ8863_FLASH_RW_COMMAND	0x80
#define KSZ8863_FLASH_RESET_COMMAND	0x90

/* Messages ******************************************************************/
#define KSZ8863_ISC_ENABLE_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0x74, 0x8, 0, 0}
#define KSZ8863_LSC_INIT_ADDR_NVCM1_MESSAGE	{KSZ8863_FLASH_RW_COMMAND, 0x47, 0, 0, 0}
#define KSZ8863_LSC_READ_TAG_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0xCA, 0, 0, 0x1}
#define KSZ8863_ISC_DISABLE_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0x26, 0, 0}
#define KSZ8863_NOOP_MESSAGE			{KSZ8863_FLASH_RW_COMMAND, 0xFF}
#define KSZ8863_ISC_ERASE_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0x0E, 0x08, 0x00, 0x00}
#define KSZ8863_LSC_CHECK_BUSY_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0xF0, 0x00, 0x00, 0x00}
#define KSZ8863_LSC_PROG_TAG_MESSAGE		{KSZ8863_FLASH_RW_COMMAND, 0xC9, 0x00, 0x00, 0x01}

/* Serial Number *************************************************************/
#define KSZ8863_SN_LENGTH		9
#define KSZ8863_SN_N_PAGES		3
#define KSZ8863_SN_BYTES_PER_PAGE	3
#define KSZ8863_FLASH_PAGE_BYTES	4

/* Other *********************************************************************/
#define KSZ8863_COMMAND_HEADER_SIZE	sizeof(u8) + sizeof(u8)	// command + reg address
#define KSZ8863_SEND_RETRIES		3
#define KSZ8863_DETECT_RETRIES		6
#define KSZ8863_CHECK_BUSY_RETRIES	20


/*****************************************************************************/
/*                                  Typedefs                                 */
/*****************************************************************************/

struct ksz8863_pcp
{
	struct device           *dev;	// Associated device
	struct tty_struct	*tty;	// TTY used for PCP communication
	struct mutex 		lock;	// Lock for TTY line discipline
};


/*****************************************************************************/
/*                              Local variables                              */
/*****************************************************************************/

static const u8 autodetection_message[] = {0xC1, 0xA0, 0xEB, 0xC0};


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

static struct tty_ldisc * get_tty_ldisc(struct ksz8863_pcp *pcp)
{
	struct tty_ldisc * ldisc = tty_ldisc_ref(pcp->tty);
	if (ldisc)
		mutex_lock(&pcp->lock);
	return ldisc;
}

static void release_tty_ldisc(struct ksz8863_pcp *pcp, struct tty_ldisc *ldisc)
{
	mutex_unlock(&pcp->lock);
	tty_ldisc_deref(ldisc);
}

/**
 * Returns true if the message contained in buf is a (part of)
 * autodetection message.
 */
static bool is_autodetection_msg(u8 *buf, size_t count)
{
	return memcmp(buf, autodetection_message, count < sizeof(autodetection_message) ? count : sizeof(autodetection_message)) == 0;
}

/**
 * Check if the latest message is an autodetection message and reboot
 * the whole system if that is the case.
 */
static void check_for_autodetection_msg(u8 *buf, size_t count)
{
	if (unlikely(is_autodetection_msg(buf, count)))
	{
		kernel_restart("EBC Autodetection message received. Restarting system.");
	}
}

/**
 * Get ksz8863_pcp struct stored in ksz_device priv data.
 */
static struct ksz8863_pcp* to_ksz8863_pcp_struct(struct ksz_device *ksz_dev)
{
	return (struct ksz8863_pcp*)((struct ksz8*)ksz_dev->priv)->priv;
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
	if (likely(ret > 0))
	{
		check_for_autodetection_msg(buf, count);
	}
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
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(ksz_dev);
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
	ldisc = get_tty_ldisc(pcp);
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
	release_tty_ldisc(pcp, ldisc);
  free_buff:
	kfree(buff);

	return ret;
}

static int ksz8863_pcp_write(void *ctx, const void *data, size_t count)
{
	struct ksz_device *ksz_dev = (struct ksz_device *)ctx;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(ksz_dev);
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
	ldisc = get_tty_ldisc(pcp);
	if (unlikely(!ldisc))
	{
		dev_err(dev, "%s: Unable to lock ldisc", __FUNCTION__);
		ret = -ENODEV;
		goto free_buff;
	}

	// Send write command and discard response
	ret = ksz8863_pcp_send_command(pcp, ldisc, write_buf, write_size);

	release_tty_ldisc(pcp, ldisc);

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


/* Sysfs functions ***********************************************************/

ssize_t serial_number_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int i;
	u8 serial_number[KSZ8863_SN_LENGTH + 1]; // Includes string terminator
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(dev_get_drvdata(dev));
	struct tty_ldisc *ldisc = get_tty_ldisc(pcp);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	// Reset Wishbone Bus
	{
		const u8 message = KSZ8863_FLASH_RESET_COMMAND;
		ret = ksz8863_pcp_do_write(pcp, ldisc, &message, sizeof(message));
	}
	if (ret)
		goto ldisc_deref;

	// Enable configuration interface in transparent mode
	{
		const u8 message[] = KSZ8863_ISC_ENABLE_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	udelay(5);

	// Set AREA1 (UFM) Address
	{
		const u8 message[] = KSZ8863_LSC_INIT_ADDR_NVCM1_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	udelay(5);

	// Read Serial Number written on several pages
	for (i = 0; i < KSZ8863_SN_N_PAGES; ++i)
	{
		u8 rcv_buf[KSZ8863_FLASH_PAGE_BYTES];
		const u8 message[] = KSZ8863_LSC_READ_TAG_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
		if (ret)
			goto ldisc_deref;
		ret = ksz8863_pcp_do_read(pcp, ldisc, rcv_buf, ARRAY_SIZE(rcv_buf));
		if (ret)
			goto ldisc_deref;

		// Fill serial number bytes
		memcpy(serial_number+(i*KSZ8863_SN_BYTES_PER_PAGE), rcv_buf, KSZ8863_SN_BYTES_PER_PAGE);

		usleep_range(200, 500);
	}
	serial_number[KSZ8863_SN_LENGTH] = '\0'; // Terminate string

	// Disable
	{
		u8 message[] = KSZ8863_ISC_DISABLE_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	// NOOP
	{
		u8 message[] = KSZ8863_NOOP_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	ret = snprintf(buf, PAGE_SIZE, "%s\n", serial_number);

  ldisc_deref:
	release_tty_ldisc(pcp, ldisc);

	return ret;
}

ssize_t serial_number_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	ssize_t ret;
	int i;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(dev_get_drvdata(dev));
	struct tty_ldisc *ldisc;

	// Precondition: input buffer length must be the same of serial number length
	if (len != KSZ8863_SN_LENGTH)
	{
		dev_err(pcp->dev, "%s: Invalid input length (%d != %d)", __FUNCTION__, len, KSZ8863_SN_LENGTH);
		return -EINVAL;
	}

	ldisc = get_tty_ldisc(pcp);
	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	// Reset Wishbone Bus
	{
		const u8 message = KSZ8863_FLASH_RESET_COMMAND;
		ret = ksz8863_pcp_do_write(pcp, ldisc, &message, sizeof(message));
	}
	if (ret)
		goto ldisc_deref;

	usleep_range(100, 200);

	// Enable configuration interface in transparent mode
	{
		const u8 message[] = KSZ8863_ISC_ENABLE_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	usleep_range(100, 200);

	// Erase UFM
	{
		const u8 message[] = KSZ8863_ISC_ERASE_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	// Sleep for a bit waiting for UFM erasure completion.
	// NOTE: Keep the line discipline locked because data cannot
	// be transfered during erasure procedure.
	msleep(700);

	// Check busy until it is ready
	{
		const u8 message[] = KSZ8863_LSC_CHECK_BUSY_MESSAGE;
		u8 rcv_buf[4];
		bool busy;
		do
		{
			ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
			if (ret)
				goto ldisc_deref;
			ret = ksz8863_pcp_do_read(pcp, ldisc, rcv_buf, ARRAY_SIZE(rcv_buf));
			if (ret)
				goto ldisc_deref;

			busy = rcv_buf[0] != 0;

			usleep_range(2000, 4000);
		} while (busy);
	}

	// Set AREA1 (UFM) Address
	{
		const u8 message[] = KSZ8863_LSC_INIT_ADDR_NVCM1_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	usleep_range(100, 200);

	// Write Serial number in the Flash AREA1 (User Flash Memory)
	for (i = 0; i < KSZ8863_SN_N_PAGES; ++i)
	{
		u8 prog_message[21] = KSZ8863_LSC_PROG_TAG_MESSAGE;
		const u8 check_busy_message[] = KSZ8863_LSC_CHECK_BUSY_MESSAGE;
		u8 rcv_buf[4];
		int j;
		bool busy;

		// Fill serial number bytes
		memcpy(prog_message + 5, buf + (i * KSZ8863_SN_BYTES_PER_PAGE), KSZ8863_SN_BYTES_PER_PAGE);

		ret = ksz8863_pcp_do_write(pcp, ldisc, prog_message, ARRAY_SIZE(prog_message));
		if (ret)
			goto ldisc_deref;
		ret = ksz8863_pcp_do_read(pcp, ldisc, rcv_buf, ARRAY_SIZE(rcv_buf));
		if (ret)
			goto ldisc_deref;

		usleep_range(100, 200);

		// Check busy for several times. If not ready, abort.
		for (j = 0, busy = true; j < KSZ8863_CHECK_BUSY_RETRIES && busy; ++j)
		{
			ret = ksz8863_pcp_do_write(pcp, ldisc, check_busy_message, ARRAY_SIZE(check_busy_message));
			if (ret)
				goto ldisc_deref;
			ret = ksz8863_pcp_do_read(pcp, ldisc, rcv_buf, ARRAY_SIZE(rcv_buf));
			if (ret)
				goto ldisc_deref;

			busy = rcv_buf[0] != 0;

			usleep_range(2000, 4000);
		}
		if (busy)
		{
			dev_err(pcp->dev, "%s: Stuck in busy condition. Abort.", __FUNCTION__);
			ret = -EBUSY;
			goto ldisc_deref;
		}
	}

	// Set AREA1 (UFM) Address
	{
		const u8 message[] = KSZ8863_LSC_INIT_ADDR_NVCM1_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	usleep_range(100, 200);

	// Verify if the serial number has been correctly stored
	for (i = 0; i < KSZ8863_SN_N_PAGES; ++i)
	{
		u8 rcv_buf[KSZ8863_FLASH_PAGE_BYTES];
		const u8 message[] = KSZ8863_LSC_READ_TAG_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
		if (ret)
			goto ldisc_deref;
		ret = ksz8863_pcp_do_read(pcp, ldisc, rcv_buf, ARRAY_SIZE(rcv_buf));
		if (ret)
			goto ldisc_deref;

		// Compare with input buffer
		if (memcmp(rcv_buf, buf + (i*KSZ8863_SN_BYTES_PER_PAGE), KSZ8863_SN_BYTES_PER_PAGE) != 0)
		{
			dev_err(pcp->dev, "%s: Stored serial number verification failed.", __FUNCTION__);
			ret = -EIO;
			goto ldisc_deref;
		}

		usleep_range(200, 500);
	}

	// Disable
	{
		u8 message[] = KSZ8863_ISC_DISABLE_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	usleep_range(100, 200);

	// NOOP
	{
		u8 message[] = KSZ8863_NOOP_MESSAGE;
		ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));
	}
	if (ret)
		goto ldisc_deref;

	// Success: return full write size even if we didn't consume all
	ret = len;

  ldisc_deref:
	release_tty_ldisc(pcp, ldisc);

	return ret;
}

static DEVICE_ATTR_RW(serial_number);

ssize_t cpld_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(dev_get_drvdata(dev));
	u8 message[] = {KSZ8863_QUERY_VERSION_COMMAND, 0, 0, 0};
	struct tty_ldisc *ldisc = get_tty_ldisc(pcp);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	ret = ksz8863_pcp_send_command(pcp, ldisc, message, ARRAY_SIZE(message));
	if (ret)
		goto ldisc_deref;

	ret = snprintf(buf, PAGE_SIZE, "%d.%d.%d\n", message[1], message[2], message[3]);

  ldisc_deref:
	release_tty_ldisc(pcp, ldisc);

	return ret;
}

static DEVICE_ATTR_RO(cpld_version);

static struct attribute *ksz8863_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_cpld_version.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ksz8863);


/* Platform device functions *************************************************/

static int __init ksz8863_pcp_init_regmap(struct ksz_device *ksz_dev)
{
	struct device *dev = to_ksz8863_pcp_struct(ksz_dev)->dev;
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

	// Initialize mutex
	mutex_init(&pcp->lock);

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
 * Lock and close TTY (both kernel and user side). Used as devm action.
 */
static void ksz8863_pcp_close_tty(void *ptr)
{
	struct tty_struct *tty = ptr;

	tty_lock(tty);

	if (tty->ops->close)
		tty->ops->close(tty, NULL);

	tty_ldisc_flush(tty);
	tty_unlock(tty);
	tty_kclose(tty);
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

	// Register cleanup function for managed resource unwinding
	ret = devm_add_action_or_reset(pdev, ksz8863_pcp_close_tty, tty);
	if (unlikely(ret))
		dev_err(pdev, "(%s) Unable to add TTY close action", __FUNCTION__);

	// Save TTY reference in private struct.
	to_ksz8863_pcp_struct(ksz_dev)->tty = tty;

	dev_dbg(pdev, "%s: end", __FUNCTION__);

	return ret;
}

/**
 * Listen to the autodetection message from EBC Connection Box
 */
static int ksz8863_pcp_detect_ebc(struct ksz_device *ksz_dev)
{
	int ret, n_retries;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(ksz_dev);
	struct tty_ldisc *ldisc = get_tty_ldisc(pcp);
	u8 *buf;
	size_t buf_size = sizeof(autodetection_message);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (unlikely(!buf))
	{
		dev_err(pcp->dev, "%s: Unable to allocate buffer", __FUNCTION__);
		release_tty_ldisc(pcp, ldisc);
		return -ENOMEM;
	}

	// Read several times from TTY searching for autodetection msg
	for (n_retries = 0; n_retries < KSZ8863_DETECT_RETRIES; ++n_retries)
	{
		dev_dbg(pcp->dev, "%s: retry = %d", __FUNCTION__, n_retries);
		ret = ldisc->ops->read(pcp->tty, NULL, buf, buf_size);
		dev_dbg(pcp->dev, "%s: read returned %d", __FUNCTION__, ret);
		if (ret < 0)
			continue;
		if (ret == buf_size && is_autodetection_msg(buf, buf_size))
		{
			ret = 0;
			dev_info(pcp->dev, "%s: EBC Connection Box found!", __FUNCTION__);
			goto exit;
		}
	}

	// Autodetection msg not found
	ret = -ENODEV;
	dev_err(pcp->dev, "%s: EBC Connection Box not found", __FUNCTION__);

  exit:
	release_tty_ldisc(pcp, ldisc);
	kfree(buf);
	return ret;
}

/**
 * Reset an output GPIO value to 0. Used as devm action.
 */
static void ksz8863_pcp_reset_gpio(void *ptr)
{
	struct gpio_desc *gpio = ptr;

	gpiod_set_value_cansleep(gpio, 0);
}

/**
 * Get an output GPIO and set its value to 1. Reset its value on exit.
 */
static int ksz8863_pcp_init_gpio(struct device *dev, const char *gpio_name)
{
	struct gpio_desc *gpio = devm_gpiod_get(dev, gpio_name, GPIOD_OUT_HIGH);
	int err;

	if (unlikely(IS_ERR(gpio)))
	{
		dev_err(dev, "%s: Failed to retrieve %s GPIO", __FUNCTION__, gpio_name);
		return PTR_ERR(gpio);
	}

	// Register cleanup function for managed resource unwinding
	err = devm_add_action_or_reset(dev, ksz8863_pcp_reset_gpio, gpio);
	if (unlikely(err))
		dev_err(dev, "(%s) Unable to add %s GPIO restore action", __FUNCTION__, gpio_name);

	return err;
}

/**
 * Get all GPIO references and set them
 */
static int ksz8863_pcp_init_gpios(struct platform_device *op)
{
	struct device *pdev = &op->dev;
	int ret;

	ret = ksz8863_pcp_init_gpio(pdev, "alive");
	if (unlikely(ret))
		return ret;
	ret = ksz8863_pcp_init_gpio(pdev, "comm-enable");
	if (unlikely(ret))
		return ret;

	// Wait a bit to be sure that the GPIO transitions are sampled
	msleep(10);

	return 0;
}

/**
 * Disable the autodetection mode on the EBC Connection Box
 */
static int __init ksz8863_pcp_disable_autodetect(struct ksz_device *ksz_dev)
{
	int ret;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(ksz_dev);
	const u8 message[] = {0x0E, 0x0B, 0x0C, 0xDA};
	struct tty_ldisc *ldisc = get_tty_ldisc(pcp);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	ret = ksz8863_pcp_do_write(pcp, ldisc, message, ARRAY_SIZE(message));

	release_tty_ldisc(pcp, ldisc);

	return ret;
}

/**
 * Send a reset command to the CPLD
 */
static int __init ksz8863_pcp_reset_cpld(struct ksz_device *ksz_dev)
{
	int ret;
	struct ksz8863_pcp *pcp = to_ksz8863_pcp_struct(ksz_dev);
	const u8 message = KSZ8863_RESET_CPLD_COMMAND;
	struct tty_ldisc *ldisc = get_tty_ldisc(pcp);

	if (unlikely(!ldisc))
	{
		dev_err(pcp->dev, "%s: Unable to lock ldisc", __FUNCTION__);
		return -ENODEV;
	}

	ret = ksz8863_pcp_do_write(pcp, ldisc, &message, sizeof(message));

	release_tty_ldisc(pcp, ldisc);

	// Wait for reset completion
	msleep(10);

	return ret;
}

/**
 * Perform a dummy read on a switch register because the first read
 * always returns 0.
 */
static int __init ksz8863_pcp_dummy_read(struct ksz_device *ksz_dev)
{
	u8 reg = 0;
	u8 val;

	return ksz8863_pcp_read(ksz_dev, &reg, sizeof(reg), &val, sizeof(val));
}

/**
 * Initialize sysfs files.
 */
static int __init ksz8863_pcp_init_sysfs(struct ksz_device *ksz_dev)
{
	struct device *dev = to_ksz8863_pcp_struct(ksz_dev)->dev;
	int ret = devm_device_add_groups(dev, ksz8863_groups);
	if (unlikely(ret))
	{
		dev_err(dev, "Unable to add sysfs groups");
	}

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

	ret = ksz8863_pcp_detect_ebc(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_init_gpios(op);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_disable_autodetect(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_reset_cpld(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_dummy_read(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8863_pcp_init_sysfs(ksz_dev);
	if (unlikely(ret))
		return ret;

	ret = ksz8_switch_register(ksz_dev);

	if (ret)
	{
		dev_err(pdev, "%s: Unable to register switch device", __FUNCTION__);
		return ret;
	}

	dev_dbg(pdev, "%s: end", __FUNCTION__);

	return 0;
}

static int ksz8863_pcp_remove(struct platform_device *op)
{
	struct ksz_device *ksz_dev = platform_get_drvdata(op);

	ksz_switch_remove(ksz_dev);

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
MODULE_SOFTDEP("pre: pcp_ldisc digitalIOManager");
MODULE_INFO(dl_tag, "kernel");
MODULE_INFO(dl_tag, "fpga");
MODULE_INFO(dl_tag, "ebc");
