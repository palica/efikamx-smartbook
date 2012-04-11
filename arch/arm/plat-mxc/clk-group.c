/*
 * clock group support
 *
 * Copyright (c) 2011 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <mach/clock.h>

#define to_clk_group(c) (container_of(c, struct clk_group, clk))

static int clk_group_prepare(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);
	int i, ret;

	for (i = 0; i < group->num_clks; i++) {
		ret = clk_prepare(group->clks[i]);
			if (ret)
				goto out;
	}

	return 0;
out:
	while (i-- > 0)
		clk_disable(group->clks[i]);
	return ret;
}

static void clk_group_unprepare(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);
	int i;

	for (i = 0; i < group->num_clks; i++)
		clk_disable(group->clks[i]);
}

static int clk_group_enable(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);
	int i, ret;

	for (i = 0; i < group->num_clks; i++) {
		ret = clk_enable(group->clks[i]);
			if (ret)
				goto out;
	}

	return 0;
out:
	while (i-- > 0)
		clk_disable(group->clks[i]);
	return ret;
}

static void clk_group_disable(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);
	int i;

	for (i = 0; i < group->num_clks; i++)
		clk_disable(group->clks[i]);
}

static unsigned long clk_group_get_rate(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);

	return clk_get_rate(group->clks[0]);
}

static long clk_group_round_rate(struct clk *clk, unsigned long rate)
{
	struct clk_group *group = to_clk_group(clk);

	return clk_round_rate(group->clks[0], rate);
}

static int clk_group_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk_group *group = to_clk_group(clk);

	return clk_set_rate(group->clks[0], rate);
}

static struct clk *clk_group_get_parent(struct clk *clk)
{
	struct clk_group *group = to_clk_group(clk);

	return group->clks[0];
}

struct clk_ops clk_group_ops = {
	.prepare = clk_group_prepare,
	.unprepare = clk_group_unprepare,
	.enable = clk_group_enable,
	.disable = clk_group_disable,
	.get_rate = clk_group_get_rate,
	.round_rate = clk_group_round_rate,
	.set_rate = clk_group_set_rate,
	.get_parent = clk_group_get_parent,
};

