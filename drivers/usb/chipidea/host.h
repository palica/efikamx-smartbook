#ifndef __DRIVERS_USB_CHIPIDEA_HOST_H
#define __DRIVERS_USB_CHIPIDEA_HOST_H

#ifdef CONFIG_USB_CHIPIDEA_HOST

int ci_hdrc_host_init(struct ci13xxx *ci);
void ci_hdrc_host_destroy(struct ci13xxx *ci);
#else

static inline int ci_hdrc_host_init(struct ci13xxx *ci)
{
	return -ENXIO;
}
static void ci_hdrc_host_destroy(struct ci13xxx *ci)
{}

#endif

#endif /* __DRIVERS_USB_CHIPIDEA_HOST_H */
