/*	$NetBSD$	*/

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
	PRINT("hey, favstar! ban stop me premiamu! -- @toshi_a")

loop:
	b loop

#if defined(VERBOSE_INIT_ARM)
#define TIMO		0x25000
xputc:
	mov	r2, #TIMO
#ifdef CONADDR
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

END(_C_LABEL(grpeach_start))
