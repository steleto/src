#ifndef ARM_RZA1UART_VAR_H
#define ARM_RZA1UART_VAR_H

#include <sys/cdefs.h>
#include  <sys/termios.h>	/* for tcflag_t */

int rza1uart_cons_attach(bus_space_tag_t, paddr_t, u_int, tcflag_t);

/*
 * Set platform dependent values
 */
void rza1uart_set_frequency(u_int, u_int);

#endif	/* !ARM_RZA1UART_VAR_H */
