// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DMA driver for Altera mSGDMA IP core
 *
 * Copyright (C) 2017 Stefan Roese <sr@denx.de>
 *
 * Based on drivers/dma/xilinx/zynqmp_dma.c, which is:
 * Copyright (C) 2016 Xilinx, Inc. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>          // to match device on DTB
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#include "dmaengine.h"

#define MSGDMA_MAX_TRANS_LEN		U32_MAX
#define MSGDMA_DESC_NUM			1024

/**
 * struct msgdma_reg_desc - implements a regular descriptor
 * @read_addr: data buffer source address
 * @write_addr: data buffer destination address
 * @len: the number of bytes to transfer per descriptor
 * @control: characteristics of the transfer
 */
struct msgdma_reg_desc {
	u32 read_addr;
	u32 write_addr;
	u32 len;
	u32 control;
};

/* mSGDMA descriptor control field bit definitions */
#define MSGDMA_DESC_CTL_SET_CH(x)	((x) & 0xff)
#define MSGDMA_DESC_CTL_GEN_SOP		BIT(8)
#define MSGDMA_DESC_CTL_GEN_EOP		BIT(9)
#define MSGDMA_DESC_CTL_PARK_READS	BIT(10)
#define MSGDMA_DESC_CTL_PARK_WRITES	BIT(11)
#define MSGDMA_DESC_CTL_END_ON_EOP	BIT(12)
#define MSGDMA_DESC_CTL_END_ON_LEN	BIT(13)
#define MSGDMA_DESC_CTL_TR_COMP_IRQ	BIT(14)
#define MSGDMA_DESC_CTL_EARLY_IRQ	BIT(15)
#define MSGDMA_DESC_CTL_TR_ERR_IRQ	GENMASK(23, 16)
#define MSGDMA_DESC_CTL_EARLY_DONE	BIT(24)

/*
 * Writing "1" the "go" bit commits the entire descriptor into the
 * descriptor FIFO(s)
 */
#define MSGDMA_DESC_CTL_GO		BIT(31)

/* Tx buffer control flags */
#define MSGDMA_DESC_CTL_TX_FIRST	(MSGDMA_DESC_CTL_GEN_SOP |	\
					 MSGDMA_DESC_CTL_TR_ERR_IRQ |	\
					 MSGDMA_DESC_CTL_GO)

#define MSGDMA_DESC_CTL_TX_MIDDLE	(MSGDMA_DESC_CTL_TR_ERR_IRQ |	\
					 MSGDMA_DESC_CTL_GO)

#define MSGDMA_DESC_CTL_TX_LAST		(MSGDMA_DESC_CTL_GEN_EOP |	\
					 MSGDMA_DESC_CTL_TR_COMP_IRQ |	\
					 MSGDMA_DESC_CTL_TR_ERR_IRQ |	\
					 MSGDMA_DESC_CTL_GO)

#define MSGDMA_DESC_CTL_TX_SINGLE	(MSGDMA_DESC_CTL_GEN_SOP |	\
					 MSGDMA_DESC_CTL_GEN_EOP |	\
					 MSGDMA_DESC_CTL_TR_COMP_IRQ |	\
					 MSGDMA_DESC_CTL_TR_ERR_IRQ |	\
					 MSGDMA_DESC_CTL_GO)

#define MSGDMA_DESC_CTL_RX_SINGLE	(MSGDMA_DESC_CTL_END_ON_EOP |	\
					 MSGDMA_DESC_CTL_END_ON_LEN |	\
					 MSGDMA_DESC_CTL_TR_COMP_IRQ |	\
					 MSGDMA_DESC_CTL_EARLY_IRQ |	\
					 MSGDMA_DESC_CTL_TR_ERR_IRQ |	\
					 MSGDMA_DESC_CTL_GO)

/* mSGDMA extended descriptor stride definitions */
#define MSGDMA_DESC_STRIDE_RD		0x00000001
#define MSGDMA_DESC_STRIDE_WR		0x00010000
#define MSGDMA_DESC_STRIDE_RW		0x00010001

/* mSGDMA dispatcher control and status register map */
#define MSGDMA_CSR_STATUS		0x00	/* Read / Clear */
#define MSGDMA_CSR_CONTROL		0x04	/* Read / Write */
#define MSGDMA_CSR_RW_FILL_LEVEL	0x08	/* 31:16 - write fill level */
						/* 15:00 - read fill level */
#define MSGDMA_CSR_RESP_FILL_LEVEL	0x0c	/* response FIFO fill level */
#define MSGDMA_CSR_RW_SEQ_NUM		0x10	/* 31:16 - write seq number */
						/* 15:00 - read seq number */

/* mSGDMA CSR status register bit definitions */
#define MSGDMA_CSR_STAT_BUSY			BIT(0)
#define MSGDMA_CSR_STAT_DESC_BUF_EMPTY		BIT(1)
#define MSGDMA_CSR_STAT_DESC_BUF_FULL		BIT(2)
#define MSGDMA_CSR_STAT_RESP_BUF_EMPTY		BIT(3)
#define MSGDMA_CSR_STAT_RESP_BUF_FULL		BIT(4)
#define MSGDMA_CSR_STAT_STOPPED			BIT(5)
#define MSGDMA_CSR_STAT_RESETTING		BIT(6)
#define MSGDMA_CSR_STAT_STOPPED_ON_ERR		BIT(7)
#define MSGDMA_CSR_STAT_STOPPED_ON_EARLY	BIT(8)
#define MSGDMA_CSR_STAT_IRQ			BIT(9)
#define MSGDMA_CSR_STAT_MASK			GENMASK(9, 0)
#define MSGDMA_CSR_STAT_MASK_WITHOUT_IRQ	GENMASK(8, 0)

#define DESC_EMPTY	(MSGDMA_CSR_STAT_DESC_BUF_EMPTY | \
			 MSGDMA_CSR_STAT_RESP_BUF_EMPTY)

/* mSGDMA CSR control register bit definitions */
#define MSGDMA_CSR_CTL_STOP			BIT(0)
#define MSGDMA_CSR_CTL_RESET			BIT(1)
#define MSGDMA_CSR_CTL_STOP_ON_ERR		BIT(2)
#define MSGDMA_CSR_CTL_STOP_ON_EARLY		BIT(3)
#define MSGDMA_CSR_CTL_GLOBAL_INTR		BIT(4)
#define MSGDMA_CSR_CTL_STOP_DESCS		BIT(5)

/* mSGDMA CSR fill level bits */
#define MSGDMA_CSR_WR_FILL_LEVEL_GET(v)		(((v) & 0xffff0000) >> 16)
#define MSGDMA_CSR_RD_FILL_LEVEL_GET(v)		((v) & 0x0000ffff)
#define MSGDMA_CSR_RESP_FILL_LEVEL_GET(v)	((v) & 0x0000ffff)

#define MSGDMA_CSR_SEQ_NUM_GET(v)		(((v) & 0xffff0000) >> 16)

/* mSGDMA response register map */
#define MSGDMA_RESP_BYTES_TRANSFERRED	0x00
#define MSGDMA_RESP_STATUS		0x04

/* mSGDMA response register bit definitions */
#define MSGDMA_RESP_EARLY_TERM	BIT(8)
#define MSGDMA_RESP_ERR_MASK	0xff

/**
 * struct msgdma_sw_desc - implements a regular sw descriptor
 * @async_tx: support for the async_tx api
 * @hw_desc: assosiated HW descriptor
 * @free_list: node of the free SW descriprots list
 */
struct msgdma_sw_desc {
	struct dma_async_tx_descriptor async_tx;
	struct msgdma_reg_desc hw_desc;
	struct list_head node;
	struct list_head tx_list;
};

/**
 * struct msgdma_device - DMA device structure
 */
struct msgdma_device {
	spinlock_t lock;
	struct device *dev;
	struct tasklet_struct irq_tasklet;
	struct list_head pending_list;
	struct list_head free_list;
	struct list_head active_list;
	struct list_head done_list;
	u32 desc_free_cnt;
	bool idle;

	struct dma_device dmadev;
	struct dma_chan	dmachan;
	dma_addr_t hw_desq;
	struct msgdma_sw_desc *sw_desq;
	unsigned int npendings;

	struct dma_slave_config slave_cfg;

	int irq;

	/* mSGDMA controller */
	void __iomem *csr;

	/* mSGDMA descriptors */
	void __iomem *desc;

	/* mSGDMA response */
	void __iomem *resp;
	int resp_enabled;

    wait_queue_head_t waitQueue;
};

#define to_mdev(chan)	container_of(chan, struct msgdma_device, dmachan)
#define tx_to_desc(tx)	container_of(tx, struct msgdma_sw_desc, async_tx)

/**
 * msgdma_get_descriptor - Get the sw descriptor from the pool
 * @mdev: Pointer to the Altera mSGDMA device structure
 *
 * Return: The sw descriptor
 */
static struct msgdma_sw_desc *msgdma_get_descriptor(struct msgdma_device *mdev)
{
	struct msgdma_sw_desc *desc;
	unsigned long flags;

	spin_lock_irqsave(&mdev->lock, flags);
	desc = list_first_entry(&mdev->free_list, struct msgdma_sw_desc, node);
	list_del(&desc->node);
	spin_unlock_irqrestore(&mdev->lock, flags);

	INIT_LIST_HEAD(&desc->tx_list);

	return desc;
}

/**
 * msgdma_free_descriptor - Issue pending transactions
 * @mdev: Pointer to the Altera mSGDMA device structure
 * @desc: Transaction descriptor pointer
 */
static void msgdma_free_descriptor(struct msgdma_device *mdev,
				   struct msgdma_sw_desc *desc)
{
	struct msgdma_sw_desc *child, *next;

	mdev->desc_free_cnt++;
	list_add_tail(&desc->node, &mdev->free_list);
	list_for_each_entry_safe(child, next, &desc->tx_list, node) {
		mdev->desc_free_cnt++;
		list_move_tail(&child->node, &mdev->free_list);
	}
}

/**
 * msgdma_free_desc_list - Free descriptors list
 * @mdev: Pointer to the Altera mSGDMA device structure
 * @list: List to parse and delete the descriptor
 */
static void msgdma_free_desc_list(struct msgdma_device *mdev,
				  struct list_head *list)
{
	struct msgdma_sw_desc *desc, *next;

	list_for_each_entry_safe(desc, next, list, node)
		msgdma_free_descriptor(mdev, desc);
}

/**
 * msgdma_desc_config - Configure the descriptor
 * @desc: Hw descriptor pointer
 * @dst: Destination buffer address
 * @src: Source buffer address
 * @len: Transfer length
 */
static void msgdma_desc_config(struct msgdma_reg_desc *desc,
			       dma_addr_t dst, dma_addr_t src, size_t len,
			       u32 stride)
{
	/* Set src & dst addresses in the descriptor */
	desc->read_addr = src;
	desc->write_addr = dst;

	desc->len = len;

	/*
	 * Don't set interrupt on xfer end yet, this will be done later
	 * for the "last" descriptor.
	 * EOP is enabled for every transfer, since it is required by
	 * some scenarios and it shouldn't create problems to those not using it.
	 */
	desc->control = MSGDMA_DESC_CTL_GO | MSGDMA_DESC_CTL_END_ON_EOP;
}

/**
 * msgdma_desc_config_eod - Mark the descriptor as end descriptor
 * @desc: Hw descriptor pointer
 */
static void msgdma_desc_config_eod(struct msgdma_reg_desc *desc)
{
	desc->control |= (MSGDMA_DESC_CTL_TR_COMP_IRQ | MSGDMA_DESC_CTL_EARLY_IRQ);
}

/**
 * msgdma_tx_submit - Submit DMA transaction
 * @tx: Async transaction descriptor pointer
 *
 * Return: cookie value
 */
static dma_cookie_t msgdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct msgdma_device *mdev = to_mdev(tx->chan);
	struct msgdma_sw_desc *new;
	dma_cookie_t cookie;
	unsigned long flags;

	new = tx_to_desc(tx);
	spin_lock_irqsave(&mdev->lock, flags);
	cookie = dma_cookie_assign(tx);

	list_add_tail(&new->node, &mdev->pending_list);
	spin_unlock_irqrestore(&mdev->lock, flags);

	return cookie;
}

/**
 * msgdma_prep_memcpy - prepare descriptors for memcpy transaction
 * @dchan: DMA channel
 * @dma_dst: Destination buffer address
 * @dma_src: Source buffer address
 * @len: Transfer length
 * @flags: transfer ack flags
 *
 * Return: Async transaction descriptor on success and NULL on failure
 */
static struct dma_async_tx_descriptor *
msgdma_prep_memcpy(struct dma_chan *dchan, dma_addr_t dma_dst,
		   dma_addr_t dma_src, size_t len, ulong flags)
{
	struct msgdma_device *mdev = to_mdev(dchan);
	struct msgdma_sw_desc *new, *first = NULL;
	struct msgdma_reg_desc *desc;
	size_t copy;
	u32 desc_cnt;
	unsigned long irqflags;

	desc_cnt = DIV_ROUND_UP(len, MSGDMA_MAX_TRANS_LEN);

	spin_lock_irqsave(&mdev->lock, irqflags);
	if (desc_cnt > mdev->desc_free_cnt) {
		spin_unlock_irqrestore(&mdev->lock, irqflags);
		dev_dbg(mdev->dev, "mdev %p descs are not available\n", mdev);
		return NULL;
	}
	mdev->desc_free_cnt -= desc_cnt;
	spin_unlock_irqrestore(&mdev->lock, irqflags);

	do {
		/* Allocate and populate the descriptor */
		new = msgdma_get_descriptor(mdev);

		copy = min_t(size_t, len, MSGDMA_MAX_TRANS_LEN);
		desc = &new->hw_desc;
		msgdma_desc_config(desc, dma_dst, dma_src, copy,
				   MSGDMA_DESC_STRIDE_RW);
		len -= copy;
		dma_src += copy;
		dma_dst += copy;
		if (!first)
			first = new;
		else
			list_add_tail(&new->node, &first->tx_list);
	} while (len);

	msgdma_desc_config_eod(desc);
	async_tx_ack(&first->async_tx);
	first->async_tx.flags = flags;

	return &first->async_tx;
}

/**
 * msgdma_prep_slave_sg - prepare descriptors for a slave sg transaction
 *
 * @dchan: DMA channel
 * @sgl: Destination scatter list
 * @sg_len: Number of entries in destination scatter list
 * @dir: DMA transfer direction
 * @flags: transfer ack flags
 * @context: transfer context (unused)
 */
static struct dma_async_tx_descriptor *
msgdma_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
		     unsigned int sg_len, enum dma_transfer_direction dir,
		     unsigned long flags, void *context)

{
	struct msgdma_device *mdev = to_mdev(dchan);
	struct dma_slave_config *cfg = &mdev->slave_cfg;
	struct msgdma_sw_desc *new, *first = NULL;
	void *desc = NULL;
	size_t len, avail;
	dma_addr_t dma_dst, dma_src;
	u32 desc_cnt = 0, i;
	struct scatterlist *sg;
	u32 stride;
	unsigned long irqflags;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	for_each_sg(sgl, sg, sg_len, i)
		desc_cnt += DIV_ROUND_UP(sg_dma_len(sg), MSGDMA_MAX_TRANS_LEN);

	spin_lock_irqsave(&mdev->lock, irqflags);
	if (desc_cnt > mdev->desc_free_cnt) {
		spin_unlock_irqrestore(&mdev->lock, irqflags);
		dev_dbg(mdev->dev, "mdev %p descs are not available\n", mdev);
		return NULL;
	}
	mdev->desc_free_cnt -= desc_cnt;
	spin_unlock_irqrestore(&mdev->lock, irqflags);

	avail = sg_dma_len(sgl);

	/* Run until we are out of scatterlist entries */
	while (true) {
		/* Allocate and populate the descriptor */
		new = msgdma_get_descriptor(mdev);

		desc = &new->hw_desc;
		len = min_t(size_t, avail, MSGDMA_MAX_TRANS_LEN);

		if (dir == DMA_MEM_TO_DEV) {
			dma_src = sg_dma_address(sgl) + sg_dma_len(sgl) - avail;
			dma_dst = cfg->dst_addr;
			stride = MSGDMA_DESC_STRIDE_RD;
		} else {
			dma_src = cfg->src_addr;
			dma_dst = sg_dma_address(sgl) + sg_dma_len(sgl) - avail;
			stride = MSGDMA_DESC_STRIDE_WR;
		}
		dev_dbg(mdev->dev, "%s: new scatterlist %x %x\n", __FUNCTION__, dma_src, dma_dst);
		msgdma_desc_config(desc, dma_dst, dma_src, len, stride);
		avail -= len;

		if (!first)
			first = new;
		else
			list_add_tail(&new->node, &first->tx_list);

		/* Fetch the next scatterlist entry */
		if (avail == 0) {
			if (sg_len == 0)
				break;
			sgl = sg_next(sgl);
			if (sgl == NULL)
				break;
			sg_len--;
			avail = sg_dma_len(sgl);
		}
	}

	msgdma_desc_config_eod(desc);
	first->async_tx.flags = flags;

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);

	return &first->async_tx;
}

static int msgdma_dma_config(struct dma_chan *dchan,
			     struct dma_slave_config *config)
{
	struct msgdma_device *mdev = to_mdev(dchan);

	memcpy(&mdev->slave_cfg, config, sizeof(*config));

	return 0;
}

/* NOTE: this is actually a reset function. Its purpose is to export a reset
    API using dmaengine's 'device_terminate_all' API, which is the closest
    to a reset API we found in the current APIs. */
static int msgdma_terminate(struct dma_chan *dchan) {

    struct msgdma_device *mdev = to_mdev(dchan);
    unsigned long flags;
    struct msgdma_sw_desc *desc, *next;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

    /* free pending sw descriptors */
    spin_lock_irqsave(&mdev->lock, flags);

    if (&mdev->active_list) {
        desc = list_first_entry_or_null(&mdev->active_list,
                        struct msgdma_sw_desc, node);
        if (desc) {
            list_del(&desc->node);
            dma_cookie_complete(&desc->async_tx);
            list_add_tail(&desc->node, &mdev->done_list);
        }
    }

    if (&mdev->done_list) {
        list_for_each_entry_safe(desc, next, &mdev->done_list, node) {
            list_del(&desc->node);

            /* Run any dependencies, then free the descriptor */
            msgdma_free_descriptor(mdev, desc);
        }
    }

    spin_unlock_irqrestore(&mdev->lock, flags);

	/* Reset mSGDMA */
	iowrite32(MSGDMA_CSR_CTL_RESET, mdev->csr + MSGDMA_CSR_CONTROL);
    smp_wmb(); /* barrier to be sure everything is written */

    /* x * HZ ---> wait for x seconds */
    if (wait_event_interruptible_timeout(mdev->waitQueue, (ioread32(mdev->csr + MSGDMA_CSR_CONTROL) & MSGDMA_CSR_STAT_RESETTING) == 0, 1 * HZ) <= 0) {
        dev_err(mdev->dev, "DMA channel did not reset\n");
        return -ERESTARTSYS;
    }

	/* Clear all status bits */
	iowrite32(MSGDMA_CSR_STAT_MASK, mdev->csr + MSGDMA_CSR_STATUS);

	/* Enable the DMA controller including interrupts */
	iowrite32(MSGDMA_CSR_CTL_GLOBAL_INTR, mdev->csr + MSGDMA_CSR_CONTROL);

	mdev->idle = true;

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);

    return 0;
}

static void msgdma_reset(struct msgdma_device *mdev)
{
	u32 val;
	int ret;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	/* Reset mSGDMA */
	iowrite32(MSGDMA_CSR_STAT_MASK, mdev->csr + MSGDMA_CSR_STATUS);
	iowrite32(MSGDMA_CSR_CTL_RESET, mdev->csr + MSGDMA_CSR_CONTROL);

	ret = readl_poll_timeout(mdev->csr + MSGDMA_CSR_STATUS, val,
				 (val & MSGDMA_CSR_STAT_RESETTING) == 0,
				 1, 10000);
	if (ret)
		dev_err(mdev->dev, "DMA channel did not reset\n");

	/* Clear all status bits */
	iowrite32(MSGDMA_CSR_STAT_MASK, mdev->csr + MSGDMA_CSR_STATUS);

	/* Enable the DMA controller including interrupts */
	iowrite32(MSGDMA_CSR_CTL_GLOBAL_INTR, mdev->csr + MSGDMA_CSR_CONTROL);

	mdev->idle = true;

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);
};

static void msgdma_copy_one(struct msgdma_device *mdev,
			    struct msgdma_sw_desc *desc)
{
	void __iomem *hw_desc = mdev->desc;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	/*
	 * Check if the DESC FIFO it not full. If its full, we need to wait
	 * for at least one entry to become free again
	 */
	while (ioread32(mdev->csr + MSGDMA_CSR_STATUS) &
	       MSGDMA_CSR_STAT_DESC_BUF_FULL)
		mdelay(1);

	/*
	 * The descriptor needs to get copied into the descriptor FIFO
	 * of the DMA controller. The descriptor will get flushed to the
	 * FIFO, once the last word (control word) is written. Since we
	 * are not 100% sure that memcpy() writes all word in the "correct"
	 * oder (address from low to high) on all architectures, we make
	 * sure this control word is written last by single coding it and
	 * adding some write-barriers here.
	 */
	//memcpy((void __force *)hw_desc, &desc->hw_desc, sizeof(desc->hw_desc) - sizeof(u32));

	dev_dbg(mdev->dev, "%s: read_addr %x\n", __FUNCTION__, desc->hw_desc.read_addr);
	iowrite32(desc->hw_desc.read_addr, hw_desc +
		  offsetof(struct msgdma_reg_desc, read_addr));
	wmb();

	dev_dbg(mdev->dev, "%s: write_addr %x\n", __FUNCTION__, desc->hw_desc.write_addr);
	iowrite32(desc->hw_desc.write_addr, hw_desc +
		  offsetof(struct msgdma_reg_desc, write_addr));
	wmb();

	dev_dbg(mdev->dev, "%s: len %x\n", __FUNCTION__, desc->hw_desc.len);
	iowrite32(desc->hw_desc.len, hw_desc +
		  offsetof(struct msgdma_reg_desc, len));
	wmb();

	/* Write control word last to flush this descriptor into the FIFO */
	mdev->idle = false;
	wmb();
	dev_dbg(mdev->dev, "%s: control %x\n", __FUNCTION__, desc->hw_desc.control);
	iowrite32(desc->hw_desc.control, hw_desc +
		  offsetof(struct msgdma_reg_desc, control));
	wmb();

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);

}

/**
 * msgdma_copy_desc_to_fifo - copy descriptor(s) into controller FIFO
 * @mdev: Pointer to the Altera mSGDMA device structure
 * @desc: Transaction descriptor pointer
 */
static void msgdma_copy_desc_to_fifo(struct msgdma_device *mdev,
				     struct msgdma_sw_desc *desc)
{
	struct msgdma_sw_desc *sdesc, *next;

	msgdma_copy_one(mdev, desc);

	list_for_each_entry_safe(sdesc, next, &desc->tx_list, node)
		msgdma_copy_one(mdev, sdesc);
}

/**
 * msgdma_start_transfer - Initiate the new transfer
 * @mdev: Pointer to the Altera mSGDMA device structure
 */
static void msgdma_start_transfer(struct msgdma_device *mdev)
{
	struct msgdma_sw_desc *desc;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	if (!mdev->idle)
		return;

	desc = list_first_entry_or_null(&mdev->pending_list,
					struct msgdma_sw_desc, node);
	if (!desc)
		return;

	list_splice_tail_init(&mdev->pending_list, &mdev->active_list);
	msgdma_copy_desc_to_fifo(mdev, desc);
	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);
}

/**
 * msgdma_issue_pending - Issue pending transactions
 * @chan: DMA channel pointer
 */
static void msgdma_issue_pending(struct dma_chan *chan)
{
	struct msgdma_device *mdev = to_mdev(chan);
	unsigned long flags;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	spin_lock_irqsave(&mdev->lock, flags);
	msgdma_start_transfer(mdev);
	spin_unlock_irqrestore(&mdev->lock, flags);

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);

}

/**
 * msgdma_chan_desc_cleanup - Cleanup the completed descriptors
 * @mdev: Pointer to the Altera mSGDMA device structure
 */
static void msgdma_chan_desc_cleanup(struct msgdma_device *mdev)
{
	struct msgdma_sw_desc *desc, *next;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	list_for_each_entry_safe(desc, next, &mdev->done_list, node) {
		dma_async_tx_callback callback;
		void *callback_param;

		list_del(&desc->node);

		callback = desc->async_tx.callback;
		callback_param = desc->async_tx.callback_param;
		if (callback) {
			spin_unlock(&mdev->lock);
			callback(callback_param);
			spin_lock(&mdev->lock);
		}

		/* Run any dependencies, then free the descriptor */
		msgdma_free_descriptor(mdev, desc);
	}
}

/**
 * msgdma_complete_descriptor - Mark the active descriptor as complete
 * @mdev: Pointer to the Altera mSGDMA device structure
 */
static void msgdma_complete_descriptor(struct msgdma_device *mdev)
{
	struct msgdma_sw_desc *desc;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	desc = list_first_entry_or_null(&mdev->active_list,
					struct msgdma_sw_desc, node);
	if (!desc)
		return;
	list_del(&desc->node);
	dma_cookie_complete(&desc->async_tx);
	list_add_tail(&desc->node, &mdev->done_list);
}

/**
 * msgdma_free_descriptors - Free channel descriptors
 * @mdev: Pointer to the Altera mSGDMA device structure
 */
static void msgdma_free_descriptors(struct msgdma_device *mdev)
{
	msgdma_free_desc_list(mdev, &mdev->active_list);
	msgdma_free_desc_list(mdev, &mdev->pending_list);
	msgdma_free_desc_list(mdev, &mdev->done_list);
}

/**
 * msgdma_free_chan_resources - Free channel resources
 * @dchan: DMA channel pointer
 */
static void msgdma_free_chan_resources(struct dma_chan *dchan)
{
	struct msgdma_device *mdev = to_mdev(dchan);
	unsigned long flags;

	spin_lock_irqsave(&mdev->lock, flags);
	msgdma_free_descriptors(mdev);
	spin_unlock_irqrestore(&mdev->lock, flags);
	kfree(mdev->sw_desq);
}

/**
 * msgdma_alloc_chan_resources - Allocate channel resources
 * @dchan: DMA channel
 *
 * Return: Number of descriptors on success and failure value on error
 */
static int msgdma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct msgdma_device *mdev = to_mdev(dchan);
	struct msgdma_sw_desc *desc;
	int i;

	mdev->sw_desq = kcalloc(MSGDMA_DESC_NUM, sizeof(*desc), GFP_NOWAIT);
	if (!mdev->sw_desq)
		return -ENOMEM;

	mdev->idle = true;
	mdev->desc_free_cnt = MSGDMA_DESC_NUM;

	INIT_LIST_HEAD(&mdev->free_list);

	for (i = 0; i < MSGDMA_DESC_NUM; i++) {
		desc = mdev->sw_desq + i;
		dma_async_tx_descriptor_init(&desc->async_tx, &mdev->dmachan);
		desc->async_tx.tx_submit = msgdma_tx_submit;
		list_add_tail(&desc->node, &mdev->free_list);
	}

	return MSGDMA_DESC_NUM;
}

/**
 * msgdma_tasklet - Schedule completion tasklet
 * @data: Pointer to the Altera sSGDMA channel structure
 */
static void msgdma_tasklet(unsigned long data)
{
	struct msgdma_device *mdev = (struct msgdma_device *)data;
	u32 count;
	u32 __maybe_unused size;
	u32 __maybe_unused status;
	unsigned long flags;

	dev_dbg(mdev->dev, "%s: start\n", __FUNCTION__);

	spin_lock_irqsave(&mdev->lock, flags);

	/* Read number of responses that are available */
	count = ioread32(mdev->csr + MSGDMA_CSR_RESP_FILL_LEVEL);
	dev_dbg(mdev->dev, "%s (%d): response count=%d\n",
		__func__, __LINE__, count);

	/* ML: if we not support response register, we don't need that cycle */
	if (mdev->resp) {
		while (count--) {
			/*
			* Read both longwords to purge this response from the FIFO
			* On Avalon-MM implementations, size and status do not
			* have any real values, like transferred bytes or error
			* bits. So we need to just drop these values.
			*/
			size = ioread32(mdev->resp + MSGDMA_RESP_BYTES_TRANSFERRED);
			status = ioread32(mdev->resp + MSGDMA_RESP_STATUS);

			msgdma_complete_descriptor(mdev);
			msgdma_chan_desc_cleanup(mdev);
		}
	} else {
		msgdma_complete_descriptor(mdev);
		msgdma_chan_desc_cleanup(mdev);
	}

	spin_unlock_irqrestore(&mdev->lock, flags);

	dev_dbg(mdev->dev, "%s: end\n", __FUNCTION__);

}

/**
 * msgdma_irq_handler - Altera mSGDMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the Altera mSGDMA device structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t msgdma_irq_handler(int irq, void *data)
{
	struct msgdma_device *mdev = data;
	u32 status;

	status = ioread32(mdev->csr + MSGDMA_CSR_STATUS);
	if ((status & MSGDMA_CSR_STAT_BUSY) == 0) {
		/* Start next transfer if the DMA controller is idle */
		spin_lock(&mdev->lock);
		mdev->idle = true;
		msgdma_start_transfer(mdev);
		spin_unlock(&mdev->lock);
	}

	tasklet_schedule(&mdev->irq_tasklet);

	/* Clear interrupt in mSGDMA controller */
	iowrite32(MSGDMA_CSR_STAT_IRQ, mdev->csr + MSGDMA_CSR_STATUS);

	return IRQ_HANDLED;
}

/**
 * msgdma_chan_remove - Channel remove function
 * @mdev: Pointer to the Altera mSGDMA device structure
 */
static void msgdma_dev_remove(struct msgdma_device *mdev)
{
	if (!mdev)
		return;

	devm_free_irq(mdev->dev, mdev->irq, mdev);
	tasklet_kill(&mdev->irq_tasklet);
	list_del(&mdev->dmachan.device_node);
}

static int request_and_map(struct platform_device *pdev, const char *name,
			   struct resource **res, void __iomem **ptr)
{
	struct resource *region;
	struct device *device = &pdev->dev;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (*res == NULL) {
		dev_err(device, "resource %s not defined\n", name);
		return -ENODEV;
	}

	region = devm_request_mem_region(device, (*res)->start,
					 resource_size(*res), dev_name(device));
	if (region == NULL) {
		dev_err(device, "unable to request %s\n", name);
		return -EBUSY;
	}

	*ptr = devm_ioremap_nocache(device, region->start,
				    resource_size(region));
	if (*ptr == NULL) {
		dev_err(device, "ioremap_nocache of %s failed!", name);
		return -ENOMEM;
	}

	return 0;
}

static struct dma_chan *of_dma_altera_msgdma_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	int count = dma_spec->args_count;
	struct msgdma_device *msgdma = ofdma->of_dma_data;
	struct dma_chan * r;

	if (!msgdma) {
		pr_err("!msgdma\n");
		return NULL;
	}

	if (count != 1) {
		pr_err("count != 1\n");
		return NULL;
	}

	// ignore argument (no needed)
	// chan_id = dma_spec->args[0];

	r = dma_get_slave_channel(&msgdma->dmachan);
	if (!r) {
		pr_err("!r\n");
		return NULL;
	}

	return r;
}

/**
 * msgdma_probe - Driver probe function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int msgdma_probe(struct platform_device *pdev)
{
	struct msgdma_device *mdev;
	struct dma_device *dma_dev;
	struct resource *dma_res;
	int ret;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_NOWAIT);
	if (!mdev)
		return -ENOMEM;

	mdev->dev = &pdev->dev;

	/* Map CSR space */
	ret = request_and_map(pdev, "csr", &dma_res, &mdev->csr);
	if (ret)
		return ret;

	/* Map (extended) descriptor space */
	ret = request_and_map(pdev, "desc", &dma_res, &mdev->desc);
	if (ret)
		return ret;

	/* Map response space */
	ret = request_and_map(pdev, "resp", &dma_res, &mdev->resp);
	if (ret)
		mdev->resp_enabled = 0;
	else
		mdev->resp_enabled = 1;

	platform_set_drvdata(pdev, mdev);

	/* Get interrupt nr from platform data */
	mdev->irq = irq_of_parse_and_map(mdev->dev->of_node, 0);
	if (mdev->irq < 0)
		return -ENXIO;

	ret = request_irq(mdev->irq, msgdma_irq_handler, 0, dev_name(&pdev->dev), mdev);
	if (ret)
		return ret;

	tasklet_init(&mdev->irq_tasklet, msgdma_tasklet, (unsigned long)mdev);

	dma_cookie_init(&mdev->dmachan);

	spin_lock_init(&mdev->lock);

	INIT_LIST_HEAD(&mdev->active_list);
	INIT_LIST_HEAD(&mdev->pending_list);
	INIT_LIST_HEAD(&mdev->done_list);
	// INIT_LIST_HEAD(&mdev->free_list);

	dma_dev = &mdev->dmadev;

	/* Set DMA capabilities */
	dma_cap_zero(dma_dev->cap_mask);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);

	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
		BIT(DMA_MEM_TO_MEM);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;

	/* Init DMA link list */
	INIT_LIST_HEAD(&dma_dev->channels);

    /* init the wait queue */
	init_waitqueue_head(&mdev->waitQueue);

	/* Set base routines */
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_issue_pending = msgdma_issue_pending;
	dma_dev->dev = &pdev->dev;

	dma_dev->copy_align = DMAENGINE_ALIGN_4_BYTES;
	dma_dev->device_prep_dma_memcpy = msgdma_prep_memcpy;
	dma_dev->device_prep_slave_sg = msgdma_prep_slave_sg;
	dma_dev->device_config = msgdma_dma_config;

	dma_dev->device_alloc_chan_resources = msgdma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = msgdma_free_chan_resources;
    dma_dev->device_terminate_all = msgdma_terminate;

	mdev->dmachan.device = dma_dev;
	list_add_tail(&mdev->dmachan.device_node, &dma_dev->channels);

	/* Set DMA mask to 64 bits */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_warn(&pdev->dev, "unable to set coherent mask to 64");
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret)
			goto fail;
	}

	msgdma_reset(mdev);

	ret = dma_async_device_register(dma_dev);
	if (ret)
		goto fail;

	if (pdev->dev.of_node) {
		ret = of_dma_controller_register(pdev->dev.of_node,
					 of_dma_altera_msgdma_xlate, mdev);
		if (ret) {
			dev_err(&pdev->dev,
			"unable to register DMA to the generic DT DMA helpers\n");
		}
	}

	dev_notice(&pdev->dev, "Altera mSGDMA driver probe success\n");

	return 0;

fail:
	msgdma_dev_remove(mdev);

	return ret;
}

/**
 * msgdma_dma_remove - Driver remove function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: Always '0'
 */
static int msgdma_remove(struct platform_device *pdev)
{
	struct msgdma_device *mdev = platform_get_drvdata(pdev);

	dma_async_device_unregister(&mdev->dmadev);
	msgdma_dev_remove(mdev);

	dev_notice(&pdev->dev, "Altera mSGDMA driver removed\n");

	return 0;
}

static struct of_device_id match[] =
{
	{.compatible="altera-msgdma", },
	{},
};
MODULE_DEVICE_TABLE(of, match);

static struct platform_driver msgdma_driver = {
	.driver = {
		.name = "altera-msgdma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(match),
	},
	.probe = msgdma_probe,
	.remove = msgdma_remove,
};

module_platform_driver(msgdma_driver);

MODULE_ALIAS("platform:altera-msgdma");
MODULE_DESCRIPTION("Altera mSGDMA driver");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL");
