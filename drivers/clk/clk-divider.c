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

#define to_clk_divider(c) (container_of(c, struct clk_divider, clk))

static unsigned long clk_divider_get_rate(struct clk *clk)
{
	struct clk_divider *divider = to_clk_divider(clk);
	unsigned long rate = clk_get_rate(divider->parent);
	unsigned int div;

	div = readl(divider->reg) >> divider->shift;
	div &= (1 << divider->width) - 1;

	if (!(divider->flags & CLK_DIVIDER_FLAG_ONE_BASED))
		div++;

	return DIV_ROUND_UP(rate, div);
}

#define account_for_rounding_errors	1

static int clk_divider_bestdiv(struct clk *clk, unsigned long rate,
		unsigned long *best_parent_rate)
{
	struct clk_divider *divider = to_clk_divider(clk);
	int i, bestdiv = 0;
	unsigned long parent_rate, best = 0, now, maxdiv;

	maxdiv = (1 << divider->width);

	if (divider->flags & CLK_DIVIDER_FLAG_ONE_BASED)
		maxdiv--;

	/*
	 * The maximum divider we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxdiv = min(ULONG_MAX / rate, maxdiv);

	if (divider->flags & CLK_DIVIDER_RATE_PROPAGATES) {
		for (i = 1; i <= maxdiv; i++) {
			parent_rate = clk_round_rate(divider->parent,
					(rate + account_for_rounding_errors) * i);
			now = DIV_ROUND_UP(parent_rate, i);

			if (now <= rate && now >= best) {
				bestdiv = i;
				best = now;
			}
		}

		if (!bestdiv) {
			bestdiv = (1 << divider->width);
			parent_rate = clk_round_rate(divider->parent, 1);
		} else {
			parent_rate = best * bestdiv;
		}
	} else {
		parent_rate = clk_get_rate(divider->parent);
		bestdiv = min(maxdiv, DIV_ROUND_UP(parent_rate, rate));
	}

	*best_parent_rate = parent_rate;

	return bestdiv;
}

static long clk_divider_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long best_parent_rate;
	int div;

	div = clk_divider_bestdiv(clk, rate, &best_parent_rate);

	return DIV_ROUND_UP(best_parent_rate, div);
}

static int clk_divider_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long best_parent_rate;
	struct clk_divider *divider = to_clk_divider(clk);
	unsigned int div;
	int ret;
	unsigned long prate, flags = 0;
	u32 val;

	div = clk_divider_bestdiv(clk, rate, &best_parent_rate);

	if (rate != DIV_ROUND_UP(best_parent_rate, div))
		return -EINVAL;

	if (divider->flags & CLK_DIVIDER_RATE_PROPAGATES) {
		prate = clk_round_rate(divider->parent,
			(rate + account_for_rounding_errors) * div);
		ret = clk_set_rate(divider->parent, prate);
		if (ret)
			return ret;
	}

	if (divider->lock)
		spin_lock_irqsave(divider->lock, flags);

	if (!(divider->flags & CLK_DIVIDER_FLAG_ONE_BASED))
		div--;

	val = readl(divider->reg);
	val &= ~(((1 << divider->width) - 1) << divider->shift);
	val |= div << divider->shift;
	writel(val, divider->reg);

	if (divider->lock)
		spin_unlock_irqrestore(divider->lock, flags);

	return 0;
}

static struct clk *clk_divider_get_parent(struct clk *clk)
{
	struct clk_divider *divider = to_clk_divider(clk);

	return divider->parent;
}

struct clk_ops clk_divider_ops = {
	.prepare = clk_parent_prepare,
	.unprepare = clk_parent_unprepare,
	.enable = clk_parent_enable,
	.disable = clk_parent_disable,
	.get_rate = clk_divider_get_rate,
	.round_rate = clk_divider_round_rate,
	.set_rate = clk_divider_set_rate,
	.get_parent = clk_divider_get_parent,
};
EXPORT_SYMBOL_GPL(clk_divider_ops);
