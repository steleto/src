/*	$NetBSD$	*/

#include "opt_cpuoptions.h"
#include "opt_cputypes.h"
#include "opt_arm_debug.h"

#include <arm/asm.h>
#include <arm/armreg.h>
#include <arm/cortex/scu_reg.h>
#include "assym.h"

RCSID("$NetBSD$")

#ifndef CONADDR
#define CONADDR	0xE8008000
#endif	/* !CONADDR */
#define	SCFSR	0x10
#define	SCFTDR	0x0C
#define SCFDR	0x1C

#define	SCFSR_TDFE	0x0020
#define	SCFSR_TEND	0x0040
#define SCFDR_TFULL	0x1000

#define BOOT_MEMSIZE	10	/* temporary for boot up kernel */
#define TEMP_L1_TABLE	(KERNEL_BASE - KERNEL_BASE_VOFFSET +		\
			 BOOT_MEMSIZE * 0x100000 - L1_TABLE_SIZE)

#ifdef VERBOSE_INIT_ARM
# define XPUTC(n)	mov r0, n; bl xputc
# define XPUTC2(n)	mov r0, n; blx r11
# define PRINT(str)	bl xprint; .ascii str, "\0"; .align 2
# define PRINT_R0	bl print_r0
#else
# define XPUTC(n)
# define XPUTC2(n)
# define PRINT(str)
# define PRINT_R0
#endif /* VERBOSE_INIT_ARM */

/*
 * Kernel start routine for GR-PEACH boards.
 * At this point, this code has been loaded into RAM
 * and the MMU is off
 */
	.section .start,"ax",%progbits

	.global	_C_LABEL(grpeach_start)
_C_LABEL(grpeach_start):
	/*
	 * Save any arguments u-boot passed us.
	 */
	movw	r4, #:lower16:uboot_args
	movt	r4, #:upper16:uboot_args
#if KERNEL_BASE_VOFFSET != 0
	sub	r4, r4, #KERNEL_BASE_VOFFSET
#endif
	stmia	r4, {r0-r3}


#ifdef VERBOSE_INIT_ARM
	PRINT("  PC=")
	mov	r0, pc
	PRINT_R0

	PRINT("  SP=")
	mov	r0, sp
	PRINT_R0

	PRINT("CPSR=")
	mrs	r0, cpsr
	PRINT_R0
#endif /* VERBOSE_INIT_ARM */

	/* set temporary stack */
	movw	sp, #:lower16:tmpstack
	movt	sp, #:upper16:tmpstack

	PRINT("<cortex_init>")
	bl	cortex_init
	PRINT("</cortex_init>\r\n")

	/*
	 * Set up a preliminary mapping in the MMU to allow us to run
	 * at KERNEL_BASE with caches on.
	 */
	PRINT("<mmu_init_table>")
	movw	r1, #:lower16:mmu_init_table
	movt	r1, #:upper16:mmu_init_table
	movw	r0, #:lower16:TEMP_L1_TABLE
	movt	r0, #:upper16:TEMP_L1_TABLE
	bl	arm_boot_l1pt_init
	PRINT("</mmu_init_table>\r\n")

	/*
	 * init the CPU TLB, Cache, MMU.
	 */
	PRINT("<arm_cpuinit>")
#ifdef VERBOSE_INIT_ARM
	adr	r11, xputc
#endif
	movw	r0, #:lower16:TEMP_L1_TABLE
	movt	r0, #:upper16:TEMP_L1_TABLE
	bl	arm_cpuinit
	PRINT("</arm_cpuinit>\r\n")

	PRINT("jump to start()\r\n")
	movw	lr, #:lower16:start
	movt	lr, #:upper16:start
	bx	lr
	/* NOTREACHED */

#include <arm/cortex/a9_mpsubr.S>

mmu_init_table:
	/* fill all table VA==PA */
	MMU_INIT(0x00000000, 0x00000000, 1 << (32 - L1_S_SHIFT),
		L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_BASE, KERNEL_BASE - KERNEL_BASE_VOFFSET,
	    (BOOT_MEMSIZE * L1_S_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_CACHEABLE)
//#if KERNEL_BASE_VOFFSET != 0
//	MMU_INIT(KERNEL_BASE - KERNEL_BASE_VOFFSET,
//	    KERNEL_BASE - KERNEL_BASE_VOFFSET,
//	    (BOOT_MEMSIZE * L1_S_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_CACHEABLE)
//#endif	/* KERNEL_BASE_VOFFSET != 0 */
	MMU_INIT(KERNEL_IO_SPI_VBASE, RZA1_SPI_BASE,
	    (RZA1_SPI_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_IO0_VBASE, RZA1_IO0_BASE,
	    (RZA1_IO0_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_IO1_VBASE, RZA1_IO1_BASE,
	    (RZA1_IO1_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_IO2_VBASE, RZA1_IO2_BASE,
	    (RZA1_IO2_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_IO3_VBASE, RZA1_IO3_BASE,
	    (RZA1_IO3_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_IO4_VBASE, RZA1_IO4_BASE,
	    (RZA1_IO4_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
	MMU_INIT(KERNEL_IO_ARMCORE_VBASE, RZA1_ARMCORE_BASE,
	    (RZA1_ARMCORE_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)

	/* map VA==PA for IO+MEM (0x00000000-0x4fffffff) */
//	MMU_INIT(0x00000000, 0x00000000,
//	    0x100000000 / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)

//#if KERNEL_BASE_VOFFSET != 0
//	MMU_INIT(KERNEL_BASE, KERNEL_BASE - KERNEL_BASE_VOFFSET,
//	    (0xa0000000 + L1_S_SIZE - 1) / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_CACHEABLE)
//#endif
//	MMU_INIT(KERNEL_BASE, KERNEL_BASE - KERNEL_BASE_VOFFSET,
//	    (0xa0000000 + L1_S_SIZE - 1) / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW | L1_S_CACHEABLE)

//	MMU_INIT(KERNEL_IO1_BASE, KERNEL_IO1_BASE,
//	    (KERNEL_IO1_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)
//
//	MMU_INIT(KERNEL_IO2_BASE, KERNEL_IO2_BASE,
//	    (KERNEL_IO2_SIZE + L1_S_SIZE - 1) / L1_S_SIZE,
//	    L1_S_PROTO_armv7 | L1_S_APv7_KRW)

	/* end of table */
	MMU_INIT(0, 0, 0, 0)

#if defined(VERBOSE_INIT_ARM)
#define TIMO		0x25000
xputc:
	mov	r2, #TIMO
#ifdef CONADDR
//	movw	r3, #:lower16:(KERNEL_IO_IO1_VBASE + (CONADDR - RZA1_IO1_BASE))
//	movt	r3, #:upper16:(KERNEL_IO_IO1_VBASE + (CONADDR - RZA1_IO1_BASE))
	movw	r3, #:lower16:CONADDR
	movt	r3, #:upper16:CONADDR
#endif
1:
	ldrh	r1, [r3, #SCFDR]
	tst	r1, #SCFDR_TFULL
	beq	2f
	subs	r2, r2, #1
	bne	1b
2:
	strb	r0, [r3, #SCFTDR]
	ldrh	r1, [r3, #SCFSR]
	bic	r1, #(SCFSR_TDFE | SCFSR_TEND)
	str	r1, [r3, #SCFSR]
	bx	lr

/*
 * xprint - print strings pointed by $PC(LR)
 *          and return to the end of string.
 *          all registers are saved.
 * e.g.)
 *    bl	xprint        <- call
 *    .ascii	"Hello\r\n\0" <- don't return here
 *    .align	2
 *    nop                     <- return to here
 */
xprint:
	stmfd	sp!, {r0, r1}
	mov	r1, lr
1:
	ldrb	r0, [r1], #1

	stmfd	sp!, {r0, r1, r2, r3}
	bl	xputc
	ldmfd	sp!, {r0, r1, r2, r3}

	cmp	r0, #0
	bne	1b

	add	r1, r1, #3
	bic	lr, r1, #3	/* lr = 4byte-aligned end of string */
	ldmfd	sp!, {r0, r1}
	bx	lr

/*
 * print_r0 - show r0 hexadecimal with CR/LF.
 *            all registers are saved.
 */
print_r0:
	stmfd	sp!, {r0, r3, lr}
	mov	r3, r0
	mov	r0, #'0'
	bl	debugputc
	mov	r0, #'x'
	bl	debugputc
	bl	print_r3
	mov	r0, #'\r'
	bl	debugputc
	mov	r0, #'\n'
	bl	debugputc
	ldmfd	sp!, {r0, r3, pc}

/*
 * print_r3 - show r3 hexadecimal without CR/LF nor prefix(0x).
 *            all registers are saved.
 */
print_r3:
	stmfd	sp!, {r0, r3-r6, lr}
	mov	r4, #28		/* num of shift. 28,24,20...8,4,0 */
	mov	r5, #0xf	/* mask */
1:
	and	r6, r5, r3, ROR r4
	cmp	r6, #10
	addlt	r0, r6, #'0'
	addge	r0, r6, #('a' - 0x0a)
	bl	debugputc
	subs	r4, r4, #4
	bge	1b
	ldmfd	sp!, {r0, r3-r6, pc}

/*
 * debugputc - putc r0. xputc() is defined in arm/cortex/a9_mpsubr.S
 *             all registers are saved.
 */
debugputc:
	stmfd	sp!, {r0, r1, r2, r3, lr}
	bl	xputc
	ldmfd	sp!, {r0, r1, r2, r3, pc}
#endif /* VERBOSE_INIT_ARM */

/*
 * temporary local stack
 */
	.align 8
	.space	1024
tmpstack:

END(_C_LABEL(grpeach_start))
