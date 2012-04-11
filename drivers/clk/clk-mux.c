/*
 * Copyright (C) 2011 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Standard functionality for the common clock API.
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>

#define to_clk_mux(c) (container_of(c, struct clk_mux, clk))

static struct clk *clk_mux_get_parent(struct clk *clk)
{
	struct clk_mux *mux = to_clk_mux(clk);
	u32 val;

	val = readl(mux->reg) >> mux->shift;
	val &= (1 << mux->width) - 1;

	if (val >= mux->num_clks)
		return ERR_PTR(-EINVAL);

	return mux->clks[val];
}

static int clk_mux_set_parent(struct clk *clk, struct clk *parent)
{
	struct clk_mux *mux = to_clk_mux(clk);
	u32 val;
	int i;
	unsigned long uninitialized_var(flags);

	for (i = 0; i < mux->num_clks; i++)
		if (mux->clks[i] == parent)
			break;

	if (i == mux->num_clks)
		return -EINVAL;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);

	val = readl(mux->reg);
	val &= ~(((1 << mux->width) - 1) << mux->shift);
	val |= i << mux->shift;
	writel(val, mux->reg);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);

	return 0;
}

static long clk_mux_round_rate(struct clk *clk, unsigned long rate)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return PTR_ERR(parent);

	return clk_get_rate(parent);
}

static int clk_mux_set_rate(struct clk *clk, unsigned long desired)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return PTR_ERR(parent);

	if (desired != clk_get_rate(parent))
		return -EINVAL;

	return 0;
}

struct clk_ops clk_mux_ops = {
	.prepare = clk_parent_prepare,
	.unprepare = clk_parent_unprepare,
	.enable = clk_parent_enable,
	.disable = clk_parent_disable,
	.get_rate = clk_parent_get_rate,
	.round_rate = clk_mux_round_rate,
	.set_rate = clk_mux_set_rate,
	.get_parent = clk_mux_get_parent,
	.set_parent = clk_mux_set_parent,
};
EXPORT_SYMBOL_GPL(clk_mux_ops);
