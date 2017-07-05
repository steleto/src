/*	$NetBSD$	*/

#ifndef ARM_RZA1_INTR_H
#define ARM_RZA1_INTR_H

#define	PIC_MAXSOURCES			256
#define	PIC_MAXMAXSOURCES		(256 + 6 * 32)

#include <arm/cortex/gic_intr.h>
#include <arm/cortex/a9tmr_intr.h>	/* A9 Timer PPIs */

#endif	/* !ARM_RZA1_INTR_H */
