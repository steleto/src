/*	$NetBSD: imx6_board.c,v 1.8 2017/06/09 18:14:59 ryo Exp $	*/

/*
 * Copyright (c) 2012  Genetec Corporation.  All rights reserved.
 * Written by Hashimoto Kenichi for Genetec Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY GENETEC CORPORATION ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GENETEC CORPORATION
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(1, "$NetBSD: imx6_board.c,v 1.8 2017/06/09 18:14:59 ryo Exp $");

#include "arml2cc.h"
#include "opt_cputypes.h"
#include "opt_evbarm_boardtype.h"
#include "opt_arm_debug.h"
#include "opt_kgdb.h"
#include "com.h"
#include "opt_machdep.h"
#include "opt_rza1uart.h"
#include "rza1uart.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>
#include <sys/gpio.h>

#include <arm/locore.h>
#include <arm/cortex/a9tmr_var.h>
#include <arm/cortex/mpcore_var.h>
#include <arm/cortex/pl310_var.h>
#include <arm/mainbus/mainbus.h>

#include <arm/rza1/rza1_reg.h>
#include <arm/rza1/rza1_var.h>

bus_space_tag_t armcore_bst = &armv7_generic_bs_tag;
bus_space_handle_t armcore_bsh;
#if NARML2CC > 0
bus_space_tag_t l2cc_bst = &armv7_generic_bs_tag;
bus_space_handle_t l2cc_bsh;
#endif

void
rza1_bootstrap(void)
{
	int error;

	error = bus_space_map(armcore_bst, RZA1_ARMCORE_BASE,
			      RZA1_ARMCORE_SIZE, 0, &armcore_bsh);
	if (error)
		panic("%s: failed to map armcore registers: %d",
		      __func__, error);

#if NARML2CC > 0
	error = bus_space_map(l2cc_bst, RZA1_L2CC_BASE,
			      RZA1_L2CC_SIZE, 0, &l2cc_bsh);
	if (error)
		panic("%s: failed to map l2cc registers: %d",
		      __func__, error);
	
	arml2cc_init(l2cc_bst, l2cc_bsh, 0);
#endif
}

void
rza1_device_register(device_t self, void *aux)
{
	prop_dictionary_t dict = device_properties(self);

	if (device_is_a(self, "armperiph") &&
	    device_is_a(device_parent(self), "mainbus")) {
		/*
		 * XXX KLUDGE ALERT XXX
		 * The iot mainbus supplies is completely wrong since it scales
		 * addresses by 2.  The simpliest remedy is to replace with our
		 * bus space used for the armcore registers (which armperiph uses).
		 */
		struct mainbus_attach_args * const mb = aux;
		mb->mb_iot = armcore_bst;
		return;
	}
#if NARML2CC > 0
	if (device_is_a(self, "arml2cc")) {
		struct mpcore_attach_args * const mpcaa = aux;
		mpcaa->mpcaa_memt = l2cc_bst;
		mpcaa->mpcaa_memh = l2cc_bsh;
                prop_dictionary_set_uint32(dict, "offset", 0);
		return;
	}
#endif
#if 0
	/*
	 * We need to tell the A9 Global/Watchdog Timer
	 * what frequency it runs at.
	 */
	if (device_is_a(self, "a9tmr") || device_is_a(self, "a9wdt")) {
		prop_dictionary_set_uint32(dict, "frequency",
		   imx6_armrootclk() / IMX6_PERIPHCLK_N);
		return;
	}
#endif
}
