/*	$NetBSD$	*/
#ifndef EVBARM_GRPEACH_IOMUX_H
#define EVBARM_GRPEACH_IOMUX_H

#include <sys/device.h>

void grpeach_setup_iomux(void);
void grpeach_device_register(device_t, void *);

#endif	/* EVBARM_GRPEACH_IOMUX_H */
