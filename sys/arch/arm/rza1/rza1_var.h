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

int rza1uart_cons_attach(bus_space_tag_t, paddr_t, u_int, tcflag_t);

/*
 * Set platform dependent values
 */
void rza1uart_set_frequency(u_int, u_int);

#endif	/* !ARM_RZA1_RZA1_VAR_H */
