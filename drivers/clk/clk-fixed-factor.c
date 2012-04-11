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

#define to_clk_fixed_factor(c) (container_of(c, struct clk_fixed_factor, clk))

static unsigned long clk_factor_get_rate(struct clk *clk)
{
	struct clk_fixed_factor *factor = to_clk_fixed_factor(clk);
	unsigned long rate = clk_get_rate(factor->parent);

	return (rate / factor->div) * factor->mult;
}

static long clk_factor_round_rate(struct clk *clk, unsigned long rate)
{
	struct clk_fixed_factor *factor = to_clk_fixed_factor(clk);

	rate = (rate / factor->mult) * factor->div;

	return clk_round_rate(factor->parent, rate);
}

static int clk_factor_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk_fixed_factor *factor = to_clk_fixed_factor(clk);

	rate = (rate / factor->mult) * factor->div;

	return clk_set_rate(factor->parent, rate);
}

static struct clk *clk_factor_get_parent(struct clk *clk)
{
	struct clk_fixed_factor *factor = to_clk_fixed_factor(clk);

	return factor->parent;
}

struct clk_ops clk_fixed_factor_ops = {
	.prepare = clk_parent_prepare,
	.unprepare = clk_parent_unprepare,
	.enable = clk_parent_enable,
	.disable = clk_parent_disable,
	.get_rate = clk_factor_get_rate,
	.round_rate = clk_factor_round_rate,
	.set_rate = clk_factor_set_rate,
	.get_parent = clk_factor_get_parent,
};
EXPORT_SYMBOL_GPL(clk_fixed_factor_ops);
