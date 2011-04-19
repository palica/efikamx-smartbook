/*
 * clock gate support
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

#define to_clk_gate(c) (container_of(c, struct clk_gate, clk))

void clk_gate_bit_set(struct clk_gate *gate)
{
	u32 val;
	unsigned long uninitialized_var(flags);

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);

	val = readl(gate->reg);
	val |= 1 << (gate->shift);
	writel(val, gate->reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
}
EXPORT_SYMBOL_GPL(clk_gate_bit_set);

void clk_gate_bit_clear(struct clk_gate *gate)
{
	u32 val;
	unsigned long uninitialized_var(flags);

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);

	val = readl(gate->reg);
	val &= ~(1 << gate->shift);
	writel(val, gate->reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
}
EXPORT_SYMBOL_GPL(clk_gate_bit_clear);

void clk_gate_enable_set(struct clk_gate *gate)
{
	writel(1 << gate->shift, gate->reg);
}
EXPORT_SYMBOL_GPL(clk_gate_enable_set);

void clk_gate_disable_set(struct clk_gate *gate)
{
	writel(1 << gate->shift, gate->reg1);
}
EXPORT_SYMBOL_GPL(clk_gate_disable_set);

static int clk_gate_enable(struct clk *clk)
{
	struct clk_gate *gate = to_clk_gate(clk);
	int ret;

	ret = clk_parent_enable(clk);
	if (ret)
		return ret;

	gate->enable(gate);

	return 0;
}

static void clk_gate_disable(struct clk *clk)
{
	struct clk_gate *gate = to_clk_gate(clk);

	gate->disable(gate);

	if (gate->parent)
		clk_parent_disable(gate->parent);
}

static struct clk *clk_gate_get_parent(struct clk *clk)
{
	struct clk_gate *gate = to_clk_gate(clk);

	return gate->parent;
}

struct clk_ops clk_gate_ops = {
	.prepare = clk_parent_prepare,
	.unprepare = clk_parent_unprepare,
	.enable = clk_gate_enable,
	.disable = clk_gate_disable,
	.get_rate = clk_parent_get_rate,
	.round_rate = clk_parent_round_rate,
	.set_rate = clk_parent_set_rate,
	.get_parent = clk_gate_get_parent,
};
EXPORT_SYMBOL_GPL(clk_gate_ops);
