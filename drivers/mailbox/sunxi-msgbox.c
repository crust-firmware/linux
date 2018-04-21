// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Samuel Holland <samuel@sholland.org>
 *
 * This driver was based on drivers/mailbox/bcm2835-mailbox.c and
 * drivers/mailbox/rockchip-mailbox.c.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

/*
 * The message box hardware provides 8 unidirectional channels. As the mailbox
 * framework expects them to be bidirectional, create virtual channels out of
 * pairs of opposite-direction hardware channels. The first channel in each pair
 * is set up for AP->SCP communication, and the second channel is set up for
 * SCP->AP transmission.
 */
#define NUM_CHANS		4

/* These macros take a virtual channel number. */
#define CTRL_REG(n)		(0x0000 + 0x4 * ((n) / 2))
#define CTRL_MASK(n)		(0x1111 << 16 * ((n) % 2))
#define CTRL_SET(n)		(0x0110 << 16 * ((n) % 2))

#define IRQ_EN_REG		0x0060
#define IRQ_STATUS_REG		0x0070
#define RX_IRQ(n)		BIT(2 + 4 * (n))
#define TX_IRQ(n)		BIT(1 + 4 * (n))

#define REMOTE_IRQ_EN_REG	0x0040
#define REMOTE_IRQ_STATUS_REG	0x0050
#define REMOTE_RX_IRQ(n)	BIT(0 + 4 * (n))
#define REMOTE_TX_IRQ(n)	BIT(3 + 4 * (n))

#define RX_FIFO_STATUS_REG(n)	(0x0104 + 0x8 * (n))
#define TX_FIFO_STATUS_REG(n)	(0x0100 + 0x8 * (n))
#define FIFO_STATUS_MASK	BIT(0)

#define RX_MSG_STATUS_REG(n)	(0x0144 + 0x8 * (n))
#define TX_MSG_STATUS_REG(n)	(0x0140 + 0x8 * (n))
#define MSG_STATUS_MASK		GENMASK(2, 0)

#define RX_MSG_DATA_REG(n)	(0x0184 + 0x8 * (n))
#define TX_MSG_DATA_REG(n)	(0x0180 + 0x8 * (n))

struct sunxi_msgbox {
	struct mbox_controller controller;
	spinlock_t lock;
	void __iomem *regs;
};

static bool sunxi_msgbox_last_tx_done(struct mbox_chan *chan);
static bool sunxi_msgbox_peek_data(struct mbox_chan *chan);

static inline int channel_number(struct mbox_chan *chan)
{
	return chan - chan->mbox->chans;
}

static inline struct sunxi_msgbox *channel_to_msgbox(struct mbox_chan *chan)
{
	return container_of(chan->mbox, struct sunxi_msgbox, controller);
}

static irqreturn_t sunxi_msgbox_irq(int irq, void *dev_id)
{
	struct mbox_chan *chan;
	struct sunxi_msgbox *mbox = dev_id;
	int n;
	uint32_t msg, reg;

	reg = readl(mbox->regs + IRQ_STATUS_REG);
	for (n = 0; n < NUM_CHANS; ++n) {
		if (!(reg & RX_IRQ(n)))
			continue;
		chan = &mbox->controller.chans[n];
		while (sunxi_msgbox_peek_data(chan)) {
			msg = readl(mbox->regs + RX_MSG_DATA_REG(n));
			dev_dbg(mbox->controller.dev,
				"Received 0x%08x on channel %d\n", msg, n);
			mbox_chan_received_data(chan, &msg);
		}
		/* Clear the pending interrupt once the FIFO is empty. */
		writel(RX_IRQ(n), mbox->regs + IRQ_STATUS_REG);
	}

	return IRQ_HANDLED;
}

static int sunxi_msgbox_send_data(struct mbox_chan *chan, void *data)
{
	struct sunxi_msgbox *mbox = channel_to_msgbox(chan);
	int n = channel_number(chan);
	uint32_t msg = *(uint32_t *)data;

	if (!sunxi_msgbox_last_tx_done(chan)) {
		dev_dbg(mbox->controller.dev,
			"Busy sending 0x%08x on channel %d\n", msg, n);
		return -EBUSY;
	}
	writel(msg, mbox->regs + TX_MSG_DATA_REG(n));
	dev_dbg(mbox->controller.dev,
		"Sent 0x%08x on channel %d\n", msg, n);

	return 0;
}

static int sunxi_msgbox_startup(struct mbox_chan *chan)
{
	struct sunxi_msgbox *mbox = channel_to_msgbox(chan);
	int n = channel_number(chan);
	uint32_t reg;

	/* Ensure FIFO directions are set properly. */
	spin_lock(&mbox->lock);
	reg = readl(mbox->regs + CTRL_REG(n));
	writel((reg & ~CTRL_MASK(n)) | CTRL_SET(n), mbox->regs + CTRL_REG(n));
	spin_unlock(&mbox->lock);

	/* Clear existing messages in the receive FIFO. */
	while (sunxi_msgbox_peek_data(chan))
		readl(mbox->regs + RX_MSG_DATA_REG(n));

	/* Clear and enable the receive interrupt. */
	spin_lock(&mbox->lock);
	reg = readl(mbox->regs + IRQ_STATUS_REG);
	writel(reg | RX_IRQ(n), mbox->regs + IRQ_STATUS_REG);
	reg = readl(mbox->regs + IRQ_EN_REG);
	writel(reg | RX_IRQ(n), mbox->regs + IRQ_EN_REG);
	spin_unlock(&mbox->lock);

	dev_dbg(mbox->controller.dev, "Startup channel %d\n", n);

	return 0;
}

static void sunxi_msgbox_shutdown(struct mbox_chan *chan)
{
	struct sunxi_msgbox *mbox = channel_to_msgbox(chan);
	int n = channel_number(chan);
	uint32_t reg;

	/* Disable the receive interrupt. */
	spin_lock(&mbox->lock);
	reg = readl(mbox->regs + IRQ_EN_REG);
	writel(reg & ~RX_IRQ(n), mbox->regs + IRQ_EN_REG);
	spin_unlock(&mbox->lock);

	dev_dbg(mbox->controller.dev, "Shutdown channel %d\n", n);
}

static bool sunxi_msgbox_last_tx_done(struct mbox_chan *chan)
{
	struct sunxi_msgbox *mbox = channel_to_msgbox(chan);
	int n = channel_number(chan);

	/*
	 * The message box hardware allows us to snoop on the other user's IRQ
	 * statuses. Consider a message to be acknowledged when the reception
	 * IRQ for that channel is cleared. As the hardware only allows clearing
	 * the IRQ for a channel when the FIFO is empty, this still ensures that
	 * the message has actually been read. Compared to checking the number
	 * of messages in the FIFO, it also gives the receiver an opportunity to
	 * perform minimal message handling (such as recording extra information
	 * from a shared memory buffer) before acknowledging a message.
	 */
	return !(readl(mbox->regs + REMOTE_IRQ_STATUS_REG) & REMOTE_RX_IRQ(n));
}

static bool sunxi_msgbox_peek_data(struct mbox_chan *chan)
{
	struct sunxi_msgbox *mbox = channel_to_msgbox(chan);
	int n = channel_number(chan);

	return (readl(mbox->regs + RX_MSG_STATUS_REG(n)) & MSG_STATUS_MASK) > 0;
}

static const struct mbox_chan_ops sunxi_msgbox_chan_ops = {
	.send_data	= sunxi_msgbox_send_data,
	.startup	= sunxi_msgbox_startup,
	.shutdown	= sunxi_msgbox_shutdown,
	.last_tx_done	= sunxi_msgbox_last_tx_done,
	.peek_data	= sunxi_msgbox_peek_data,
};

static struct mbox_chan *sunxi_msgbox_index_xlate(struct mbox_controller *mbox,
	const struct of_phandle_args *sp)
{
	if (sp->args_count != 2)
		return NULL;
	/* Enforce this driver's assumed physical-to-virtual channel mapping. */
	if ((sp->args[0] % 2) || (sp->args[1] != sp->args[0] + 1) ||
	    (sp->args[0] > (NUM_CHANS - 1) * 2))
		return NULL;

	return &mbox->chans[sp->args[0] / 2];
}

static int sunxi_msgbox_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct device *dev = &pdev->dev;
	struct mbox_chan *chans;
	struct reset_control *rst;
	struct resource *res;
	struct sunxi_msgbox *mbox;
	int ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	chans = devm_kcalloc(dev, NUM_CHANS, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mbox->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->regs))
		return PTR_ERR(mbox->regs);

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "Failed to get clock\n");
		return PTR_ERR(clk);
	}

	rst = devm_reset_control_get(dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(dev, "Failed to get reset\n");
		return PTR_ERR(rst);
	}

	/*
	 * The failure path should not disable the clock or assert the reset,
	 * because the PSCI implementation in firmware relies on this device
	 * being functional. Claiming the clock in this driver is required to
	 * prevent Linux from turning it off.
	 */
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock: %d\n", ret);
		return ret;
	}

	ret = reset_control_deassert(rst);
	if (ret) {
		dev_err(dev, "Failed to deassert reset: %d\n", ret);
		return ret;
	}

	/* Disable all interrupts. */
	writel(0, mbox->regs + IRQ_EN_REG);

	ret = devm_request_irq(dev, irq_of_parse_and_map(dev->of_node, 0),
			       sunxi_msgbox_irq, 0, dev_name(dev), mbox);
	if (ret) {
		dev_err(dev, "Failed to register IRQ handler: %d\n", ret);
		return ret;
	}

	mbox->controller.dev = dev;
	mbox->controller.ops = &sunxi_msgbox_chan_ops;
	mbox->controller.chans = chans;
	mbox->controller.num_chans = NUM_CHANS;
	mbox->controller.txdone_irq = false;
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 5;
	mbox->controller.of_xlate = sunxi_msgbox_index_xlate;

	spin_lock_init(&mbox->lock);
	platform_set_drvdata(pdev, mbox);

	ret = mbox_controller_register(&mbox->controller);
	if (ret)
		dev_err(dev, "Failed to register mailbox: %d\n", ret);

	return ret;
}

static int sunxi_msgbox_remove(struct platform_device *pdev)
{
	struct sunxi_msgbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->controller);

	return 0;
}

static const struct of_device_id sunxi_msgbox_of_match[] = {
	{ .compatible = "allwinner,sun6i-a31-msgbox", },
	{ .compatible = "allwinner,sun8i-h3-msgbox", },
	{ .compatible = "allwinner,sun50i-a64-msgbox", },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_msgbox_of_match);

static struct platform_driver sunxi_msgbox_driver = {
	.driver = {
		.name = "sunxi-msgbox",
		.of_match_table = sunxi_msgbox_of_match,
	},
	.probe		= sunxi_msgbox_probe,
	.remove		= sunxi_msgbox_remove,
};
module_platform_driver(sunxi_msgbox_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Allwinner sunxi Message Box");
MODULE_LICENSE("GPL v2");
