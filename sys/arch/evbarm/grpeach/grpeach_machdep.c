/*	$NetBSD$	*/

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_evbarm_boardtype.h"
#include "opt_arm_debug.h"
#include "opt_kgdb.h"
#include "com.h"
#include "opt_machdep.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/atomic.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/msgbuf.h>
#include <sys/reboot.h>
#include <sys/termios.h>

#include <dev/cons.h>

#include <uvm/uvm_extern.h>

#include <arm/db_machdep.h>
#include <arm/arm32/machdep.h>

#include <machine/autoconf.h>
#include <machine/bootconfig.h>

#include <arm/cortex/scu_reg.h>
#include <arm/rza1/rza1_var.h>

#include <evbarm/grpeach/platform.h>

/* static char bootargs[MAX_BOOT_STRING]; */
BootConfig bootconfig;
char *boot_args = NULL;

u_int uboot_args[4] = { 0 };

bus_space_tag_t spi_bst = &armv7_generic_bs_tag;
bus_space_handle_t spi_bsh;
bus_space_tag_t io0_bst = &armv7_generic_bs_tag;
bus_space_handle_t io0_bsh;
bus_space_tag_t io1_bst = &armv7_generic_bs_tag;
bus_space_handle_t io1_bsh;
bus_space_tag_t io2_bst = &armv7_generic_bs_tag;
bus_space_handle_t io2_bsh;
bus_space_tag_t io3_bst = &armv7_generic_bs_tag;
bus_space_handle_t io3_bsh;
bus_space_tag_t io4_bst = &armv7_generic_bs_tag;
bus_space_handle_t io4_bsh;
bus_space_tag_t armcore_bst = &armv7_generic_bs_tag;
bus_space_handle_t armcore_bsh;

void consinit(void);
static void led(int);

/*
 * Static device mappings. These peripheral registers are mapped at
 * fixed virtual addresses very early in initarm() so that we can use
 * them while booting the kernel, and stay at the same address
 * throughout whole kernel's life time.
 *
 * We use this table twice; once with bootstrap page table, and once
 * with kernel's page table which we build up in initarm().
 *
 * Since we map these registers into the bootstrap page table using
 * pmap_devmap_bootstrap() which calls pmap_map_chunk(), we map
 * registers segment-aligned and segment-rounded in order to avoid
 * using the 2nd page tables.
 */
static const struct pmap_devmap devmap[] = {
	{
		KERNEL_IO_SPI_VBASE,
		RZA1_SPI_BASE,
		RZA1_SPI_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_IO0_VBASE,
		RZA1_IO0_BASE,
		RZA1_IO0_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_IO1_VBASE,
		RZA1_IO1_BASE,
		RZA1_IO1_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_IO2_VBASE,
		RZA1_IO2_BASE,
		RZA1_IO2_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_IO3_VBASE,
		RZA1_IO3_BASE,
		RZA1_IO3_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_IO4_VBASE,
		RZA1_IO4_BASE,
		RZA1_IO4_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		KERNEL_IO_ARMCORE_VBASE,
		RZA1_ARMCORE_BASE,
		RZA1_ARMCORE_SIZE,
		VM_PROT_READ | VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{ 0, 0, 0, 0, 0 }
};

/*
 * u_int initarm(...)
 *
 * Initial entry point on startup. This gets called before main() is
 * entered.
 * It should be responsible for setting up everything that must be
 * in place when main is called.
 * This includes
 *   Taking a copy of the boot configuration structure.
 *   Initialising the physical console so characters can be printed.
 *   Setting up page tables for the kernel
 */
u_int
initarm(void *arg)
{
	int error;

	led(1);

	pmap_devmap_register(devmap);

	io0_bsh = (bus_space_handle_t)KERNEL_IO_IO0_VBASE;
	error = bus_space_map(io0_bst, RZA1_IO0_BASE,
			      RZA1_IO0_SIZE, 0, &io0_bsh);
	if (error)
		panic("%s: failed to map io0 registers: %d",
		      __func__, error);

	io1_bsh = (bus_space_handle_t)KERNEL_IO_IO1_VBASE;
	error = bus_space_map(io1_bst, RZA1_IO1_BASE,
			      RZA1_IO1_SIZE, 0, &io1_bsh);
	if (error)
		panic("%s: failed to map io1 registers: %d",
		      __func__, error);

	io2_bsh = (bus_space_handle_t)KERNEL_IO_IO2_VBASE;
	error = bus_space_map(io2_bst, RZA1_IO2_BASE,
			      RZA1_IO2_SIZE, 0, &io2_bsh);
	if (error)
		panic("%s: failed to map io2 registers: %d",
		      __func__, error);

	io3_bsh = (bus_space_handle_t)KERNEL_IO_IO3_VBASE;
	error = bus_space_map(io3_bst, RZA1_IO3_BASE,
			      RZA1_IO3_SIZE, 0, &io3_bsh);
	if (error)
		panic("%s: failed to map io3 registers: %d",
		      __func__, error);

	io4_bsh = (bus_space_handle_t)KERNEL_IO_IO4_VBASE;
	error = bus_space_map(io4_bst, RZA1_IO4_BASE,
			      RZA1_IO3_SIZE, 0, &io4_bsh);
	if (error)
		panic("%s: failed to map io4 registers: %d",
		      __func__, error);

	armcore_bsh = (bus_space_handle_t)KERNEL_IO_ARMCORE_VBASE;
	error = bus_space_map(armcore_bst, RZA1_ARMCORE_BASE,
			      RZA1_ARMCORE_SIZE, 0, &armcore_bsh);
	if (error)
		panic("%s: failed to map armcore registers: %d",
		      __func__, error);

#if NARML2CC > 0
	arml2cc_init(armcore_bst, armcore_bsh, ARMCORE_L2C_BASE);
#endif

	led(2);

	/* The console is going to try to map things.  Give pmap a devmap. */
	consinit();

/*	bootargs[0] = '\0'; */

	return 0;
}

void
consinit(void)
{
}

static void
led(int rgb)
{
	volatile uint32_t *gpio = (uint32_t *)0xFCFE3118;
	*gpio = 0xe0000000 | ((rgb << 13) & 0xe000);
}
