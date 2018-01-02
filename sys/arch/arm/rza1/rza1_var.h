/*	$NetBSD$	*/

#ifndef ARM_RZA1_RZA1_VAR_H
#define ARM_RZA1_RZA1_VAR_H

#include <sys/cdefs.h>

struct axi_attach_args {
	const char *aa_name;
	bus_space_tag_t aa_iot;
	bus_dma_tag_t aa_dmat;
	bus_addr_t aa_addr;
	bus_size_t aa_size;
	int aa_irq;
	int aa_irqbase;
};

extern struct bus_space armv7_generic_bs_tag;

/*
 * rza1_board.c
 */
void rza1_bootstrap(void);
void rza1_device_register(device_t, void *);

#endif	/* !ARM_RZA1_RZA1_VAR_H */
