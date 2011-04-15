/*
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Standard functionality for the common clock API.
 */

#include <linux/clk.h>
#include <linux/module.h>

int clk_prepare(struct clk *clk)
{
	int ret = 0;

	if (!clk)
		return 0;

	mutex_lock(&clk->prepare_lock);
	if (clk->prepare_count == 0 && clk->ops->prepare)
		ret = clk->ops->prepare(clk);

	if (!ret)
		clk->prepare_count++;
	mutex_unlock(&clk->prepare_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(clk_prepare);

void clk_unprepare(struct clk *clk)
{
	if (!clk)
		return;

	mutex_lock(&clk->prepare_lock);

	WARN_ON(clk->prepare_count == 0);

	if (--clk->prepare_count == 0 && clk->ops->unprepare) {
		WARN_ON(clk->enable_count != 0);
		clk->ops->unprepare(clk);
	}

	mutex_unlock(&clk->prepare_lock);
}
EXPORT_SYMBOL_GPL(clk_unprepare);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;

	if (!clk)
		return 0;

	WARN_ON(clk->prepare_count == 0);

	spin_lock_irqsave(&clk->enable_lock, flags);
	if (clk->enable_count == 0 && clk->ops->enable)
		ret = clk->ops->enable(clk);

	if (!ret)
		clk->enable_count++;
	spin_unlock_irqrestore(&clk->enable_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (!clk)
		return;

	spin_lock_irqsave(&clk->enable_lock, flags);

	WARN_ON(clk->enable_count == 0);

	if (!--clk->enable_count == 0 && clk->ops->disable)
		clk->ops->disable(clk);

	spin_unlock_irqrestore(&clk->enable_lock, flags);
}
EXPORT_SYMBOL_GPL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (!clk)
		return 0;

	if (clk->ops->get_rate)
		return clk->ops->get_rate(clk);
	return 0;
}
EXPORT_SYMBOL_GPL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (!clk)
		return -ENOSYS;

	if (clk->ops->round_rate)
		return clk->ops->round_rate(clk, rate);
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (!clk)
		return -ENOSYS;

	might_sleep();

	if (clk->ops->set_rate)
		return clk->ops->set_rate(clk, rate);
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (!clk)
		return -ENOSYS;

	if (clk->ops->set_parent)
		return clk->ops->set_parent(clk, parent);
	return -ENOSYS;
}
EXPORT_SYMBOL_GPL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	if (!clk)
		return ERR_PTR(-ENOSYS);

	if (clk->ops->get_parent)
		return clk->ops->get_parent(clk);
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL_GPL(clk_get_parent);

int __clk_get(struct clk *clk)
{
	if (clk->ops->get)
		return clk->ops->get(clk);
	return 1;
}

void __clk_put(struct clk *clk)
{
	if (clk->ops->put)
		clk->ops->put(clk);
}

/* clk_fixed support */

#define to_clk_fixed(clk) (container_of(clk, struct clk_fixed, clk))

static unsigned long clk_fixed_get_rate(struct clk *clk)
{
	return to_clk_fixed(clk)->rate;
}

struct clk_ops clk_fixed_ops = {
	.get_rate = clk_fixed_get_rate,
};
EXPORT_SYMBOL_GPL(clk_fixed_ops);

int clk_parent_prepare(struct clk *clk)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return -ENOSYS;

	return clk_prepare(parent);
}
EXPORT_SYMBOL_GPL(clk_parent_prepare);

void clk_parent_unprepare(struct clk *clk)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return;

	clk_unprepare(parent);
}
EXPORT_SYMBOL_GPL(clk_parent_unprepare);

int clk_parent_enable(struct clk *clk)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return 0;

	return clk_enable(parent);
}
EXPORT_SYMBOL_GPL(clk_parent_enable);

void clk_parent_disable(struct clk *clk)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return;

	clk_disable(parent);
}
EXPORT_SYMBOL_GPL(clk_parent_disable);

unsigned long clk_parent_get_rate(struct clk *clk)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return 0;

	return clk_get_rate(parent);
}
EXPORT_SYMBOL_GPL(clk_parent_get_rate);

long clk_parent_round_rate(struct clk *clk, unsigned long rate)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return -ENOSYS;

	return clk_round_rate(parent, rate);
}
EXPORT_SYMBOL_GPL(clk_parent_round_rate);

int clk_parent_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *parent = clk_get_parent(clk);

	if (IS_ERR(parent))
		return -ENOSYS;

	return clk_set_rate(parent, rate);
}
EXPORT_SYMBOL_GPL(clk_parent_set_rate);
