/*	$NetBSD$	*/

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/bus.h>
#include <sys/device.h>
#include <sys/param.h>
#include <arm/rza1/rza1_var.h>
#include <evbarm/grpeach/platform.h>
#include <evbarm/grpeach/grpeach_iomux.h>

void
grpeach_setup_iomux(void)
{
}

void
grpeach_device_register(device_t self, void *aux)
{
	prop_dictionary_t dict = device_properties(self);

	(void)&dict;

	rza1_device_register(self, aux);
}
