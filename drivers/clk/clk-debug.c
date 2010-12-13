/*
 * Copyright (C) 2010 Yong Shen <yong.shen@linaro.org>
 * Copyright (C) 2011 Canonical Ltd <jeremy.kerr@canonical.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Standard functionality for the common clock API.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

static struct dentry *clk_root;

/*
 * debugfs support to trace clock tree hierarchy and attributes
 */
static int clk_debug_rate_get(void *data, u64 *val)
{
	struct clk *clk = data;

	*val = (u64)clk_get_rate(clk);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(clk_debug_rate_fops, clk_debug_rate_get, NULL,
		"%llu\n");

static int clk_debug_register_one(struct clk *clk)
{
	struct dentry *d, *parent_dir, *child, *child_tmp;
	struct clk *parent = clk_get_parent(clk);
	int err;

	if (!IS_ERR_OR_NULL(parent))
		parent_dir = parent->dentry;
	else
		parent_dir = clk_root;

	d = debugfs_create_dir(clk->name, parent_dir);
	if (!d)
		return -ENOMEM;

	clk->dentry = d;

	d = debugfs_create_u32("enable_count", S_IRUGO, clk->dentry,
			(u32 *)&clk->enable_count);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	d = debugfs_create_u32("prepare_count", S_IRUGO, clk->dentry,
			(u32 *)&clk->prepare_count);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	d = debugfs_create_file("rate", S_IRUGO, clk->dentry, (void *)clk,
			&clk_debug_rate_fops);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	return 0;

err_out:
	d = clk->dentry;
	list_for_each_entry_safe(child, child_tmp, &d->d_subdirs, d_u.d_child)
		debugfs_remove(child);
	debugfs_remove(clk->dentry);
	return err;
}

#define NR_PREINIT_CLKS	16

static struct clk *preinit_clks[NR_PREINIT_CLKS];
static int n_preinit_clks;
static DEFINE_MUTEX(preinit_lock);
static int init_done;

void clk_debug_register(struct clk *clk)
{
	int err;
	struct clk *pa;

	if (init_done) {
		pa = clk_get_parent(clk);

		if (pa && !IS_ERR(pa) && !pa->dentry)
			clk_debug_register(pa);

		if (!clk->dentry) {
			err = clk_debug_register_one(clk);
			if (err)
				return;
		}
	} else {
		mutex_lock(&preinit_lock);
		if (n_preinit_clks == NR_PREINIT_CLKS) {
			pr_warn("clk:debug: ran out of preinit_clks, "
					"ignoring clk %s\n", clk->name);
			return;
		}

		preinit_clks[n_preinit_clks] = clk;
		n_preinit_clks++;

		mutex_unlock(&preinit_lock);
	}
}
EXPORT_SYMBOL_GPL(clk_debug_register);

static int __init clk_debugfs_init(void)
{
	int i;

	/* Here, debugfs is supposed to be initialised */
	if (!clk_root)
		clk_root = debugfs_create_dir("clocks", NULL);

	if (!clk_root)
		return -ENOMEM;

	init_done = 1;

	mutex_lock(&preinit_lock);
	for (i = 0; i < n_preinit_clks; i++)
		clk_debug_register(preinit_clks[i]);

	mutex_unlock(&preinit_lock);

	return 0;
}
late_initcall(clk_debugfs_init);
