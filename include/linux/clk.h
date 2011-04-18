/*
 *  linux/include/linux/clk.h
 *
 *  Copyright (C) 2004 ARM Limited.
 *  Written by Deep Blue Solutions Limited.
 *  Copyright (c) 2010-2011 Jeremy Kerr <jeremy.kerr@canonical.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_CLK_H
#define __LINUX_CLK_H

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

struct device;

#ifdef CONFIG_USE_COMMON_STRUCT_CLK

/* If we're using the common struct clk, we define the base clk object here */

/**
 * struct clk - hardware independent clock structure
 * @ops:		implementation-specific ops for this clock
 * @enable_count:	count of clk_enable() calls active on this clock
 * @enable_lock:	lock for atomic enable
 * @prepare_count:	count of clk_prepare() calls active on this clock
 * @prepare_lock:	lock for sleepable prepare
 *
 * The base clock object, used by drivers for hardware-independent manipulation
 * of clock lines. This will be 'subclassed' by device-specific implementations,
 * which add device-specific data to struct clk. For example:
 *
 *  struct clk_foo {
 *      struct clk;
 *      [device specific fields]
 *  };
 *
 * The clock driver code will manage the device-specific data, and pass
 * clk_foo.clk to the common clock code. The clock driver will be called
 * through the @ops callbacks.
 *
 * The @enable_lock and @prepare_lock members are used to serialise accesses
 * to the ops->enable and ops->prepare functions (and the corresponding
 * ops->disable and ops->unprepare functions).
 */
struct clk {
	const struct clk_ops	*ops;
	unsigned int		enable_count;
	unsigned int		prepare_count;
	spinlock_t		enable_lock;
	struct mutex		prepare_lock;
#ifdef CONFIG_CLK_DEBUG
#define CLK_NAME_LEN	32
	char			name[CLK_NAME_LEN];
	struct dentry		*dentry;
#endif
};

#ifdef CONFIG_CLK_DEBUG
#define __INIT_CLK_DEBUG(n)	.name = #n,
#else
#define __INIT_CLK_DEBUG(n)
#endif

/* static initialiser for clocks. */
#define INIT_CLK(name, o) {						\
	.ops		= &o,						\
	.enable_lock	= __SPIN_LOCK_UNLOCKED(name.enable_lock),	\
	.prepare_lock	= __MUTEX_INITIALIZER(name.prepare_lock),	\
	__INIT_CLK_DEBUG(name)						\
}

/**
 * struct clk_ops -  Callback operations for clocks; these are to be provided
 * by the clock implementation, and will be called by drivers through the clk_*
 * API.
 *
 * @prepare:	Prepare the clock for enabling. This must not return until
 *		the clock is fully prepared, and it's safe to call clk_enable.
 *		This callback is intended to allow clock implementations to
 *		do any initialisation that may sleep. Called with
 *		clk->prepare_lock held.
 *
 * @unprepare:	Release the clock from its prepared state. This will typically
 *		undo any work done in the @prepare callback. Called with
 *		clk->prepare_lock held.
 *
 * @enable:	Enable the clock atomically. This must not return until the
 *		clock is generating a valid clock signal, usable by consumer
 *		devices. Called with clk->enable_lock held. This function
 *		must not sleep.
 *
 * @disable:	Disable the clock atomically. Called with clk->enable_lock held.
 *		This function must not sleep.
 *
 * @get:	Called by the core clock code when a device driver acquires a
 *		clock via clk_get(). Optional.
 *
 * @put:	Called by the core clock code when a devices driver releases a
 *		clock via clk_put(). Optional.
 *
 * The clk_enable/clk_disable and clk_prepare/clk_unprepare pairs allow
 * implementations to split any work between atomic (enable) and sleepable
 * (prepare) contexts.  If a clock requires sleeping code to be turned on, this
 * should be done in clk_prepare. Switching that will not sleep should be done
 * in clk_enable.
 *
 * Typically, drivers will call clk_prepare when a clock may be needed later
 * (eg. when a device is opened), and clk_enable when the clock is actually
 * required (eg. from an interrupt). Note that clk_prepare *must* have been
 * called before clk_enable.
 *
 * For other callbacks, see the corresponding clk_* functions. Parameters and
 * return values are passed directly from/to these API functions, or
 * -ENOSYS (or zero, in the case of clk_get_rate) is returned if the callback
 * is NULL, see drivers/clk/clk.c for implementation details. All are optional.
 */
struct clk_ops {
	int		(*prepare)(struct clk *);
	void		(*unprepare)(struct clk *);
	int		(*enable)(struct clk *);
	void		(*disable)(struct clk *);
	int		(*get)(struct clk *);
	void		(*put)(struct clk *);
	unsigned long	(*get_rate)(struct clk *);
	long		(*round_rate)(struct clk *, unsigned long);
	int		(*set_rate)(struct clk *, unsigned long);
	int		(*set_parent)(struct clk *, struct clk *);
	struct clk *	(*get_parent)(struct clk *);
};

/**
 * clk_common_init - initialise a clock for driver usage
 *
 * @clk: The clock to initialise
 *
 * Used for runtime intialization of clocks; you don't need to call this
 * if your clock has been (statically) initialized with INIT_CLK.
 */
static inline void clk_common_init(struct clk *clk)
{
	clk->enable_count = clk->prepare_count = 0;
	spin_lock_init(&clk->enable_lock);
	mutex_init(&clk->prepare_lock);
}

/* Simple fixed-rate clock */
struct clk_fixed {
	struct clk	clk;
	unsigned long	rate;
};

extern struct clk_ops clk_fixed_ops;

#define INIT_CLK_FIXED(name, r) { \
	.clk = INIT_CLK(name.clk, clk_fixed_ops), \
	.rate = (r) \
}

#define DEFINE_CLK_FIXED(name, r) \
	struct clk_fixed name = INIT_CLK_FIXED(name, r)

/* generic pass-through-to-parent functions */
int clk_parent_prepare(struct clk *clk);
void clk_parent_unprepare(struct clk *clk);
int clk_parent_enable(struct clk *clk);
void clk_parent_disable(struct clk *clk);
unsigned long clk_parent_get_rate(struct clk *clk);
long clk_parent_round_rate(struct clk *clk, unsigned long rate);
int clk_parent_set_rate(struct clk *clk, unsigned long rate);

/**
 * struct clk_divider - clock divider
 *
 * @clk		clock source
 * @reg		register containing the divider
 * @shift	shift to the divider
 * @width	width of the divider
 * @lock	register lock
 * @parent	parent clock
 *
 * This clock implements get_rate/set_rate/round_rate. prepare/unprepare and
 * enable/disable are passed through to the parent.
 *
 * The divider is calculated as div = reg_val + 1
 * or if CLK_DIVIDER_FLAG_ONE_BASED is set as div = reg_val
 * (with reg_val == 0 considered invalid)
 */
struct clk_divider {
	struct clk	clk;
	void __iomem	*reg;
	unsigned char	shift;
	unsigned char	width;
	unsigned int	div;
	struct clk	*parent;
	spinlock_t	*lock;
#define CLK_DIVIDER_FLAG_ONE_BASED	(1 << 0)
#define CLK_DIVIDER_RATE_PROPAGATES	(1 << 1)
	unsigned int	flags;
};

extern struct clk_ops clk_divider_ops;

/**
 * struct clk_fixed_factor - fixed factor between input and output
 *
 * @clk		clock source
 * @mult	fixed multiplier value
 * @div		fixed divider value
 * @parent	parent clock
 *
 * This clock implements a fixed divider with an additional multiplier to
 * specify fractional values. clk_enable/clk_disable are passed through
 * to the parent. Note that the divider is applied before the multiplier
 * to prevent overflows. This may result in a less accurat result.
 */
struct clk_fixed_factor {
	struct clk	clk;
	unsigned int	mult;
	unsigned int	div;
	struct clk	*parent;
};

extern struct clk_ops clk_fixed_factor_ops;

#define DEFINE_CLK_FIXED_FACTOR(name, _parent, _mult, _div) \
	struct clk_fixed_factor name = { \
		.clk = INIT_CLK(name.clk, clk_fixed_factor_ops), \
		.parent = (_parent), \
		.mult = (_mult), \
		.div = (_div), \
	}

/**
 * struct clk_mux - clock multiplexer
 *
 * @clk		clock source
 * @reg		the register this multiplexer can be configured with
 * @shift	the shift to the start bit of this multiplexer
 * @width	the width in bits of this multiplexer
 * @num_clks	number of parent clocks
 * @lock	register lock
 * @clks	array of possible parents for this multiplexer. Can contain
 *		holes with NULL in it for invalid register settings
 *
 * This clock implements get_parent/set_parent. prepare/unprepare,
 * enable/disable and get_rate operations are passed through to the parent,
 * the rate is not adjustable.
 */
struct clk_mux {
	struct clk	clk;
	void __iomem	*reg;
	unsigned char	shift;
	unsigned char	width;
	unsigned char	num_clks;
	spinlock_t	*lock;
	struct clk	**clks;
};

extern struct clk_ops clk_mux_ops;

#else /* !CONFIG_USE_COMMON_STRUCT_CLK */

/*
 * Global clock object, actual structure is declared per-machine
 */
struct clk;

static inline void clk_common_init(struct clk *clk) { }

#endif /* !CONFIG_USE_COMMON_STRUCT_CLK */

/**
 * clk_get - lookup and obtain a reference to a clock producer.
 * @dev: device for clock "consumer"
 * @id: clock comsumer ID
 *
 * Returns a struct clk corresponding to the clock producer, or
 * valid IS_ERR() condition containing errno.  The implementation
 * uses @dev and @id to determine the clock consumer, and thereby
 * the clock producer.  (IOW, @id may be identical strings, but
 * clk_get may return different clock producers depending on @dev.)
 *
 * Drivers must assume that the clock source is not enabled.
 *
 * clk_get should not be called from within interrupt context.
 */
struct clk *clk_get(struct device *dev, const char *id);

/**
 * clk_prepare - prepare a clock source
 * @clk: clock source
 *
 * This prepares the clock source for use.
 *
 * Must not be called from within atomic context.
 */
#ifdef CONFIG_HAVE_CLK_PREPARE
int clk_prepare(struct clk *clk);
#else
static inline int clk_prepare(struct clk *clk)
{
	might_sleep();
	return 0;
}
#endif

/**
 * clk_enable - inform the system when the clock source should be running.
 * @clk: clock source
 *
 * If the clock can not be enabled/disabled, this should return success.
 *
 * May be called from atomic contexts.
 *
 * Returns success (0) or negative errno.
 */
int clk_enable(struct clk *clk);

/**
 * clk_disable - inform the system when the clock source is no longer required.
 * @clk: clock source
 *
 * Inform the system that a clock source is no longer required by
 * a driver and may be shut down.
 *
 * May be called from atomic contexts.
 *
 * Implementation detail: if the clock source is shared between
 * multiple drivers, clk_enable() calls must be balanced by the
 * same number of clk_disable() calls for the clock source to be
 * disabled.
 */
void clk_disable(struct clk *clk);


/**
 * clk_unprepare - undo preparation of a clock source
 * @clk: clock source
 *
 * This undoes a previously prepared clock.  The caller must balance
 * the number of prepare and unprepare calls.
 *
 * Must not be called from within atomic context.
 */
#ifdef CONFIG_HAVE_CLK_PREPARE
void clk_unprepare(struct clk *clk);
#else
static inline void clk_unprepare(struct clk *clk)
{
	might_sleep();
}
#endif

/* clk_prepare_enable helps cases using clk_enable in non-atomic context. */
static inline int clk_prepare_enable(struct clk *clk)
{
	int ret;

	ret = clk_prepare(clk);
	if (ret)
		return ret;
	ret = clk_enable(clk);
	if (ret)
		clk_unprepare(clk);

	return ret;
}

/* clk_disable_unprepare helps cases using clk_disable in non-atomic context. */
static inline void clk_disable_unprepare(struct clk *clk)
{
	clk_disable(clk);
	clk_unprepare(clk);
}

/**
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source.
 *		  This is only valid once the clock source has been enabled.
 *		  Returns zero if the clock rate is unknown.
 * @clk: clock source
 */
unsigned long clk_get_rate(struct clk *clk);

/**
 * clk_put	- "free" the clock source
 * @clk: clock source
 *
 * Note: drivers must ensure that all clk_enable calls made on this
 * clock source are balanced by clk_disable calls prior to calling
 * this function.
 *
 * clk_put should not be called from within interrupt context.
 */
void clk_put(struct clk *clk);

/**
 * clk_round_rate - adjust a rate to the exact rate a clock can provide
 * @clk: clock source
 * @rate: desired clock rate in Hz
 *
 * Returns rounded clock rate in Hz, or negative errno.
 */
long clk_round_rate(struct clk *clk, unsigned long rate);
 
/**
 * clk_set_rate - set the clock rate for a clock source
 * @clk: clock source
 * @rate: desired clock rate in Hz
 *
 * Returns success (0) or negative errno.
 */
int clk_set_rate(struct clk *clk, unsigned long rate);
 
/**
 * clk_set_parent - set the parent clock source for this clock
 * @clk: clock source
 * @parent: parent clock source
 *
 * Returns success (0) or negative errno.
 */
int clk_set_parent(struct clk *clk, struct clk *parent);

/**
 * clk_get_parent - get the parent clock source for this clock
 * @clk: clock source
 *
 * Returns struct clk corresponding to parent clock source, or
 * valid IS_ERR() condition containing errno.
 */
struct clk *clk_get_parent(struct clk *clk);

/**
 * clk_get_sys - get a clock based upon the device name
 * @dev_id: device name
 * @con_id: connection ID
 *
 * Returns a struct clk corresponding to the clock producer, or
 * valid IS_ERR() condition containing errno.  The implementation
 * uses @dev_id and @con_id to determine the clock consumer, and
 * thereby the clock producer. In contrast to clk_get() this function
 * takes the device name instead of the device itself for identification.
 *
 * Drivers must assume that the clock source is not enabled.
 *
 * clk_get_sys should not be called from within interrupt context.
 */
struct clk *clk_get_sys(const char *dev_id, const char *con_id);

/**
 * clk_add_alias - add a new clock alias
 * @alias: name for clock alias
 * @alias_dev_name: device name
 * @id: platform specific clock name
 * @dev: device
 *
 * Allows using generic clock names for drivers by adding a new alias.
 * Assumes clkdev, see clkdev.h for more info.
 */
int clk_add_alias(const char *alias, const char *alias_dev_name, char *id,
			struct device *dev);

#ifdef CONFIG_CLK_DEBUG
void clk_debug_register(struct clk *clk);
#else
static inline void clk_debug_register(struct clk *clk) { }
#endif

#endif
