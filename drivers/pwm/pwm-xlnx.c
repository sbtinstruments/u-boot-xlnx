// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on the Linux driver by Thomas More.
 *
 * Tested by Thomas More on zedboard - axi timer v2.00a - test
 * Tested by Frederik Peter Aalund on custom Zynq7020-based board
 *
 * Copyright (C) 2014 Thomas More
 * Copyright (C) 2020 Frederik Peter Aalund <fpa@sbtinstruments.com>
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <clk.h>
#include <pwm.h>
#include <dm/read.h>
#include <linux/io.h>
#include <linux/ioport.h>

/* mmio regiser mapping */

#define TCSR0		0x00
#define TLR0		0x04
#define TCSR1		0x10	/* Timer 1 Control and Status Register */
#define TLR1		0x14	/* Timer 1 Load Register */

#define PERIOD		TLR0
#define DUTY		TLR1

#define UDT_BIT		BIT(1)	/* Up/Down Count Timer */
#define GENT_BIT	BIT(2)	/* Enable External Generate Signal Timer */
#define ENT_BIT		BIT(7)	/* Enable Timer */
#define PWMA_BIT	BIT(9)	/* Enable Pulse Width Modulation for Timer */
#define PWM_CONF	(UDT_BIT | GENT_BIT | ENT_BIT | PWMA_BIT)

struct xilinx_pwm_priv {
	void __iomem *mmio_base;
	uint clk_period;
};

static int xlnx_pwm_set_config(struct udevice *dev, uint channel,
                           uint period_ns, uint duty_ns)
{
	struct xilinx_pwm_priv *priv = dev_get_priv(dev);
	u32 tlrx_duty = max(2u, duty_ns / priv->clk_period) - 2;
	u32 tlrx_period = max(2u, period_ns / priv->clk_period) - 2;
	/* When duty_cycle==period, the output is zero. The output should have
	 * been full saturation. As a workaround, we cap the duty cycle so that
	 * duty_cycle<=period. In practice, this means that the output will never
	 * reach full saturation (100% duty cycle) but only close to it (~99.9%
	 * duty cycle). */
	tlrx_duty = min(tlrx_period - 1, tlrx_duty);
	dev_dbg(dev, "duty cycle [ns]: %d\n", duty_ns);
	dev_dbg(dev, "period     [ns]: %d\n", period_ns);
	dev_dbg(dev, "clk_period  [1]: %d\n", priv->clk_period);
	dev_dbg(dev, "tlrx_duty   [1]: %d\n", tlrx_duty);
	dev_dbg(dev, "tlrx_period [1]: %d\n", tlrx_period);
	iowrite32(tlrx_duty, priv->mmio_base + DUTY);
	iowrite32(tlrx_period, priv->mmio_base + PERIOD);
	return 0;
}

static int xlnx_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct xilinx_pwm_priv *priv = dev_get_priv(dev);
	u32 data = (enable) ? PWM_CONF : 0;
	iowrite32(data, priv->mmio_base + TCSR0);
	iowrite32(data, priv->mmio_base + TCSR1);
	return 0;
}

static int xlnx_pwm_set_invert(struct udevice *dev, uint channel,
				bool polarity)
{
	if (!polarity)
		return 0;
	return -ENOTSUPP;
}

static int xlnx_pwm_probe(struct udevice *dev)
{
	int ret;
	struct xilinx_pwm_priv *priv = dev_get_priv(dev);
	struct clk *clk;
    struct resource res;

	clk = devm_clk_get(dev, "s_axi_aclk");
	if (IS_ERR(clk)) {
		dev_err(dev, "Could not find s_axi_aclk: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	/* convert the clock rate into the clock period (Hz to ns) */
	priv->clk_period = 1000000000 / clk_get_rate(clk);

	ret = dev_read_resource(dev, 0, &res);
    if (ret) {
        dev_err(dev, "Could not read resource: %d\n", ret);
        return ret;
    }
	priv->mmio_base = devm_ioremap(dev, res.start, resource_size(&res));
	if (IS_ERR(priv->mmio_base)) {
        dev_err(dev, "Could not remap IO: %d\n", ret);
		return PTR_ERR(priv->mmio_base);
	}

	return 0;
}

static const struct pwm_ops xlnx_pwm_ops = {
	.set_config	= xlnx_pwm_set_config,
	.set_enable	= xlnx_pwm_set_enable,
	.set_invert	= xlnx_pwm_set_invert,
};

static const struct udevice_id xlnx_pwm_ids[] = {
	{ .compatible = "xlnx,pwm-xlnx", },
	{ },
};

U_BOOT_DRIVER(xlnx_pwm) = {
	.name	= "xlnx_pwm",
	.id	= UCLASS_PWM,
	.of_match = xlnx_pwm_ids,
	.ops	= &xlnx_pwm_ops,
	.probe		= xlnx_pwm_probe,
	.priv_auto_alloc_size	= sizeof(struct xilinx_pwm_priv),
};
