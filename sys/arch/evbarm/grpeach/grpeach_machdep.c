/*	$NetBSD$	*/

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_evbarm_boardtype.h"
#include "opt_arm_debug.h"
#include "opt_kgdb.h"
#include "com.h"
#include "opt_machdep.h"
#include "opt_rza1uart.h"
#include "rza1uart.h"

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
#include <evbarm/grpeach/grpeach_iomux.h>

extern int KERNEL_BASE_phys[];
extern int KERNEL_BASE_virt[];

BootConfig bootconfig;
static char bootargs[MAX_BOOT_STRING];
char *boot_args = NULL;

u_int uboot_args[4] = { 0 };

/*
 * Macros to translate between physical and virtual for a subset of the
 * kernel address space.  *Not* for general use.
 */
#define KERN_VTOPDIFF	((vaddr_t)KERNEL_BASE_phys - (vaddr_t)KERNEL_BASE_virt)
#define KERN_VTOPHYS(va) ((paddr_t)((vaddr_t)va + (vaddr_t)KERN_VTOPDIFF))

#ifndef MEMSIZE
#define MEMSIZE		10
#endif

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
	psize_t memsize;
	int error;

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

	/* The console is going to try to map things.  Give pmap a devmap. */
	consinit();

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	if (set_cpufuncs())		// starts PMC counter
		panic("cpu not recognized!");

	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);

#ifdef NO_POWERSAVE
	cpu_do_powersave = 0;
#endif

	cortex_pmc_ccnt_init();

	printf("\nuboot arg = %#x, %#x, %#x, %#x\n",
	    uboot_args[0], uboot_args[1], uboot_args[2], uboot_args[3]);

	/* Talk to the user */
	printf("\nNetBSD/evbarm (" ___STRING(EVBARM_BOARDTYPE) ") booting ...\n");

#ifdef BOOT_ARGS
	char mi_bootargs[] = BOOT_ARGS;
	parse_mi_bootargs(mi_bootargs);
#endif /* BOOT_ARGS */
	bootargs[0] = '\0';

#ifdef VERBOSE_INIT_ARM
	printf("initarm: Configuring system, CLIDR=%010o CTR=%#x\n",
	       armreg_clidr_read(), armreg_ctr_read());
#endif /* VERBOSE_INIT_ARM */

	memsize = MEMSIZE * 1024 * 1024;

	bootconfig.dramblocks = 1;
	bootconfig.dram[0].address = KERN_VTOPHYS(KERNEL_BASE);
	bootconfig.dram[0].pages = memsize / PAGE_SIZE;

#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	const bool mapallmem_p = true;
#ifndef PMAP_NEED_ALLOC_POOLPAGE
	if (memsize > KERNEL_VM_BASE - KERNEL_BASE) {
		printf("%s: dropping RAM size from %luMB to %uMB\n",
		   __func__, (unsigned long) (memsize >> 20),
		   (KERNEL_VM_BASE - KERNEL_BASE) >> 20);
		memsize = KERNEL_VM_BASE - KERNEL_BASE;
	}
#endif
#else /* !__HAVE_MM_MD_DIRECT_MAPPED_PHYS */
	const bool mapallmem_p = false;
#endif /* __HAVE_MM_MD_DIRECT_MAPPED_PHYS */

	arm32_bootmem_init(bootconfig.dram[0].address,
	    memsize, (paddr_t)KERNEL_BASE_phys);
	led(5);
	arm32_kernel_vm_init(KERNEL_VM_BASE, ARM_VECTORS_LOW, 0, devmap,
	    mapallmem_p);
	led(6);

	/* we've a specific device_register routine */
	evbarm_device_register = grpeach_device_register;


#ifdef PMAP_NEED_ALLOC_POOLPAGE
	/*
	 * If we couldn't map all of memory via TTBR1, limit the memory the
	 * kernel can allocate from to be from the highest available 1GB.
	 */
	if (atop(memsize) > bp_highgig.bp_pages) {
		bp_highgig.bp_start += atop(memsize) - bp_highgig.bp_pages;
		arm_poolpage_vmfreelist = bp_highgig.bp_freelist;
		return initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE,
		    &bp_highgig, 1);
	}
#endif
	led(7);

	return initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE, NULL, 0);
}

#ifdef CONSDEVNAME
const char consdevname[] = CONSDEVNAME;

#ifndef CONADDR
#define CONADDR	RZA1_SCIF_BASE(2)
#endif
#ifndef CONMODE
#define CONMODE	((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
#ifndef CONSPEED
#define CONSPEED	115200
#endif

int consmode = CONMODE;
int consrate = CONSPEED;

#endif /* CONSDEVNAME */

#ifndef RZA1UART_FREQ
#define RZA1UART_FREQ	80000000
#endif

void
consinit(void)
{
	static int consinit_called = FALSE;

	if (consinit_called)
		return;

	consinit_called = TRUE;

#ifdef CONSDEVNAME
#if NRZA1UART > 0
	rza1uart_set_frequency(RZA1UART_FREQ, 2);
#endif
	led(1);
#if (NRZA1UART > 0) && defined(RZA1UARTCONSOLE)
	if (strcmp(consdevname, CONSDEVNAME) == 0) {
		paddr_t consaddr = CONADDR;
		led(2);
		rza1uart_cons_attach(&armv7_generic_bs_tag, consaddr,
				     consrate, consmode);
		led(3);
		return;
	}
#endif	/* (NRZA1UART >0) && defined(RZA1UARTCONSOLE) */
#endif	/* CONSDEVNAME */
}

static void
led(int rgb)
{
	volatile uint32_t *gpio = (uint32_t *)(KERNEL_IO_IO3_VBASE + (0xFCFE3118 - RZA1_IO3_BASE));
	*gpio = 0xe0000000 | ((rgb << 13) & 0xe000);
}
