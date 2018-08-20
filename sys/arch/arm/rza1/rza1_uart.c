/*	$NetBSD$	*/

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <sys/tty.h>
#include <sys/proc.h>
#include <sys/conf.h>
#include <sys/file.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/syslog.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/timepps.h>
#include <sys/vnode.h>
#include <sys/kauth.h>
#include <sys/intr.h>

#include <sys/bus.h>

#include <arm/rza1/rza1_reg.h>
#include <arm/rza1/rza1_var.h>
#include <arm/rza1/rza1uart_var.h>
#include <dev/cons.h>

#include "opt_rza1uart.h"

struct rza1uart_regs {
	bus_space_tag_t iot;
	bus_space_handle_t ioh;
	paddr_t iobase;
};

struct rza1uart_softc {
	device_t sc_dev;

	struct rza1uart_regs regs;
	struct callout sc_diag_callout;
	struct tty *sc_tty;
	kmutex_t sc_lock;

	u_char	sc_hwflags;
/* Hardware flag masks */
#define	RZA1UART_HW_FLOW 	__BIT(0)
#define	RZA1UART_HW_DEV_OK	__BIT(1)
#define	RZA1UART_HW_CONSOLE	__BIT(2)
#define	RZA1UART_HW_KGDB        __BIT(3)

	bool enabled;
};

int rza1uart_match(device_t, cfdata_t, void *);
void rza1uart_attach(device_t, device_t, void *);


CFATTACH_DECL_NEW(rza1uart, sizeof(struct rza1uart_softc),
		  rza1uart_match, rza1uart_attach, NULL, NULL);

extern struct cfdriver rza1uart_cd;

dev_type_open(rza1uart_open);
dev_type_close(rza1uart_close);
dev_type_read(rza1uart_read);
dev_type_write(rza1uart_write);
dev_type_ioctl(rza1uart_ioctl);
dev_type_stop(rza1uart_stop);
dev_type_tty(rza1uart_tty);
dev_type_poll(rza1uart_poll);

const struct cdevsw rza1uart_cdevsw = {
	.d_open     = rza1uart_open,
	.d_close    = rza1uart_close,
	.d_read     = rza1uart_read,
	.d_write    = rza1uart_write,
	.d_ioctl    = rza1uart_ioctl,
	.d_stop     = rza1uart_stop,
	.d_tty      = rza1uart_tty,
	.d_poll     = rza1uart_poll,
	.d_mmap     = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard  = nodiscard,
	.d_flag     = D_TTY
};

/* struct tty */
static void rza1uart_start(struct tty *);
static int rza1uart_param(struct tty *, struct termios *);

static struct rza1uart_regs rza1ucons_regs;
static int rza1ucons_rate;
static tcflag_t rza1ucons_cflag;
static struct cnm_state rza1ucons_cnm_state;
static int rza1ucons_attached;

inline static void
led(int rgb)
{
	volatile uint32_t *gpio = (uint32_t *)0xff6e3118;
	*gpio = 0xe0000000 | ((rgb << 13) & 0xe000);
}

int
rza1uart_match(device_t parent, struct cfdata *cf, void *aux)
{
	struct axi_attach_args * const aa = aux;
	switch (aa->aa_addr) {
	case RZA1_SCIF_BASE(0):
	case RZA1_SCIF_BASE(1):
	case RZA1_SCIF_BASE(2):
	case RZA1_SCIF_BASE(3):
	case RZA1_SCIF_BASE(4):
	case RZA1_SCIF_BASE(5):
	case RZA1_SCIF_BASE(6):
	case RZA1_SCIF_BASE(7):
		return 1;
	default:
		break;
	}
	return 0;
}

void
rza1uart_attach(device_t parent, device_t self, void *aux)
{
	struct axi_attach_args *aa = aux;
	struct rza1uart_softc *sc = device_private(self);
	struct rza1uart_regs *regsp = &sc->regs;
	struct tty *tp;
	bus_space_handle_t ioh;
	size_t size;

	aprint_naive("\n");
	aprint_normal("\n");

	sc->sc_dev = self;

	size = (aa->aa_size > 0 ? size : RZA1_SCIF_SIZE);
	regsp->iot = aa->aa_iot;
	regsp->iobase = aa->aa_addr;

	if (bus_space_map(regsp->iot, regsp->iobase, size, 0, &ioh))
		return;
	regsp->ioh = ioh;

	callout_init(&sc->sc_diag_callout, 0);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_HIGH);

	if (regsp->iobase == rza1ucons_regs.iobase) {
		rza1ucons_attached = 1;

		/* Make sure the console is always "hardwired". */
		SET(sc->sc_hwflags, RZA1UART_HW_CONSOLE);
	}

	tp = tty_alloc();
	tp->t_oproc = rza1uart_start;
	tp->t_param = rza1uart_param;

	sc->sc_tty = tp;

	tty_attach(tp);

	if (ISSET(sc->sc_hwflags, RZA1UART_HW_CONSOLE)) {
		int maj;
		maj = cdevsw_lookup_major(&rza1uart_cdevsw);

		if (maj != NODEVMAJOR) {
			dev_t dev;
			dev = makedev(maj, device_unit(sc->sc_dev));
			tp->t_dev = dev;
			cn_tab->cn_dev = dev;

			aprint_normal_dev(sc->sc_dev, "console\n");
		}
	}

	sc->enabled = 1;

	SET(sc->sc_hwflags, RZA1UART_HW_DEV_OK);

	// XXX
	return;
}

static void
rza1uart_start(struct tty *tp)
{
}

static int
rza1uart_param(struct tty *tp, struct termios *t)
{
	return 0;
}

#if defined(RZA1UARTCONSOLE) || defined(KGDB)

unsigned char rza1uart_getc(struct rza1uart_regs *);
void rza1uart_putc(struct rza1uart_regs *, unsigned char);
int rza1uart_common_getc(dev_t, struct rza1uart_regs *);
void rza1uart_common_putc(dev_t, struct rza1uart_regs *, unsigned char);

/*
 * The following functions are polled getc and putc routines, shared
 * by the console and kgdb glue.
 *
 * The read-ahead code is so that you can detect pending in-band
 * cn_magic in polled mode while doing output rather than having to
 * wait until the kernel decides it needs input.
 */
#define	READAHEAD_RING_LEN	16
static unsigned char rza1uart_readahead[READAHEAD_RING_LEN];
static int rza1uart_readahead_in = 0;
static int rza1uart_readahead_out = 0;
#define	READAHEAD_IS_EMPTY	\
	(rza1uart_readahead_in == rza1uart_readahead_out)
#define	READAHEAD_IS_FULL	\
	(((rza1uart_readahead_in + 1) & (READAHEAD_RING_LEN - 1)) == \
	 rza1uart_readahead_out)

unsigned char
rza1uart_getc(struct rza1uart_regs *regsp)
{
	uint16_t scfdr, scfsr, sclsr, scfsr_err, sclsr_err;
	unsigned char c;

	for (;;) {
		/* wait for ready */
		do {
			scfdr = bus_space_read_2(regsp->iot, regsp->ioh,
						 RZA1_SCFDR);
		} while ((scfdr & RZA1_SCFDR_RXCNT) == 0);

		c = bus_space_read_1(regsp->iot, regsp->ioh, RZA1_SCFRDR);

		scfsr_err = bus_space_read_2(regsp->iot, regsp->ioh,
					     RZA1_SCFSR);
		scfsr = bus_space_read_2(regsp->iot, regsp->ioh, RZA1_SCFSR);
		scfsr &= ~(RZA1_SCFSR_ER | RZA1_SCFSR_BRK | RZA1_SCFSR_RDF |
			   RZA1_SCFSR_DR);
		bus_space_write_2(regsp->iot, regsp->ioh, RZA1_SCFSR, scfsr);

		sclsr_err = bus_space_read_2(regsp->iot, regsp->ioh,
					     RZA1_SCLSR);
		sclsr = bus_space_read_2(regsp->iot, regsp->ioh, RZA1_SCLSR);
		sclsr &= ~RZA1_SCLSR_ORER;
		bus_space_write_2(regsp->iot, regsp->ioh, RZA1_SCLSR, sclsr);

		scfsr_err &= (RZA1_SCFSR_ER | RZA1_SCFSR_BRK | RZA1_SCFSR_FER);
		sclsr_err &= RZA1_SCLSR_ORER;
		if (scfsr_err == 0 && sclsr_err == 0)
			return c;
	}
}

void
rza1uart_putc(struct rza1uart_regs *regsp, unsigned char c)
{
	uint16_t scfdr, scfsr;

	/* wait for ready */
	do {
		scfdr = bus_space_read_2(regsp->iot, regsp->ioh, RZA1_SCFDR);
	} while ((scfdr & RZA1_SCFDR_TXCNT) == RZA1_SCFDR_TX_FULL);

	/* write send data to send register */
	bus_space_write_1(regsp->iot, regsp->ioh, RZA1_SCFTDR, c);

	/* clear ready flag */
	scfsr = bus_space_read_2(regsp->iot, regsp->ioh, RZA1_SCFSR);
	scfsr &= ~(RZA1_SCFSR_TDFE | RZA1_SCFSR_TEND);
	bus_space_write_2(regsp->iot, regsp->ioh, RZA1_SCFSR, scfsr);
}

int
rza1uart_common_getc(dev_t dev, struct rza1uart_regs *regsp)
{
	int s = splserial();
	unsigned char c;

	/* got a character from reading things earlier */
	if (rza1uart_readahead_in != rza1uart_readahead_out) {
		c = rza1uart_readahead[rza1uart_readahead_out];
		rza1uart_readahead_out = (rza1uart_readahead_out + 1) &
			(READAHEAD_RING_LEN - 1);
		splx(s);
		return (c);
	}

	c = rza1uart_getc(regsp);

	{
		int __attribute__((__unused__))cn_trapped = 0; /* unused */
#ifdef DDB
		extern int db_active;
		if (!db_active)
#endif
			cn_check_magic(dev, c, rza1ucons_cnm_state);
	}
	splx(s);

	return c;
}

void
rza1uart_common_putc(dev_t dev, struct rza1uart_regs *regsp, unsigned char c)
{
	int s = splserial();

	if (!READAHEAD_IS_FULL) {
		int __attribute__((__unused__))cn_trapped = 0;
		rza1uart_readahead_in = (rza1uart_readahead_in + 1) &
			(READAHEAD_RING_LEN - 1);
	}

	rza1uart_putc(regsp, c);

	bus_space_barrier(regsp->iot, regsp->ioh, 0, RZA1_SCIF_SIZE,
			  BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE);

	splx(s);
}
#endif /* defined(RZA1UARTCONSOLE) || defined(KGDB) */
	    
#ifdef RZA1UARTCONSOLE
int rza1ucngetc(dev_t);
void rza1ucnputc(dev_t, int);
void rza1ucnpollc(dev_t, int);
/*
 * Following are all routines needed for UART to act as console
 */
struct consdev rza1ucons = {
	NULL, NULL, rza1ucngetc, rza1ucnputc, rza1ucnpollc, NULL, NULL, NULL,
	NODEV, CN_NORMAL
};

int
rza1uart_cons_attach(bus_space_tag_t iot, paddr_t iobase, u_int rate,
		     tcflag_t cflag)
{
	struct rza1uart_regs regs;
	int error;
	int16_t scscr, scfcr, scfsr;


	regs.iot = iot;
	regs.iobase = iobase;

	error = bus_space_map(regs.iot, regs.iobase, RZA1_SCIF_SIZE,
			      0, &regs.ioh);
	if (error)
		return error;

	/* wait for TEND */
	do {
		scfsr = bus_space_read_2(regs.iot, regs.ioh, RZA1_SCFSR);
	} while((scfsr & RZA1_SCFSR_TEND) == 0);

	/* Initialize SCSCR */
	scscr = bus_space_read_2(regs.iot, regs.ioh, RZA1_SCSCR);
	scscr &= ~(RZA1_SCSCR_TE | RZA1_SCSCR_RE);
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR, scscr);

	/* Reset FIFO data */
	scfcr = bus_space_read_2(regs.iot, regs.ioh, RZA1_SCFCR);
	scfcr |= RZA1_SCFCR_TFRST | RZA1_SCFCR_RFRST;
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFCR, scfcr);

	/* Reset SCFSR, SCLSR */
	scfsr = bus_space_read_2(regs.iot, regs.ioh, RZA1_SCFSR);
	scfsr &= ~(RZA1_SCFSR_ER | RZA1_SCFSR_DR | RZA1_SCFSR_BRK);
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFSR, scfsr);
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCLSR, 0);

	/* clock */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR, 0);

	/* Serial Mode */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSMR, 0); /* TODO: 8N1 */

	/* Serial Extend */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCEMR, 0);

	/* Bit rate */ /* TODO: 115200 */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCBRR, ((66666666 + 16 * 115200) / (32 * 115200)) - 1);

	/* set FIFO control */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFCR, 0);

	/* clock */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR, 0);

	/* Send permission, Receive permission ON */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR, RZA1_SCSCR_TE | RZA1_SCSCR_RE);

#if 0
	/* Initialize SCSCR */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR, 0);

	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFCR,
			  RZA1_SCFCR_TFRST | RZA1_SCFCR_RFRST);

	/* Serial Mode Register */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSMR, 0); /* 8N1 */

	/* bit rate */
	/* bus_space_write_1(iot, ioh, RZA1_SCBRR, 0); */

	/*
	 * wait 2m Sec, because Send/Recv must begin 1 bit period after
	 * BRR is set.
	 */
	/* delay (2000); */

	led(1);

	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFCR,
			  FIFO_RCV_TRIGGER_14 | FIFO_XMT_TRIGGER_1);

	/* Send permission, Receive permission ON */
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCSCR,
			  RZA1_SCSCR_TE | RZA1_SCSCR_RE);

	/* Serial Status Register */
	scfsr = bus_space_read_2(regs.iot, regs.ioh, RZA1_SCFSR);
	bus_space_write_2(regs.iot, regs.ioh, RZA1_SCFSR,
			  scfsr & RZA1_SCFSR_TDFE);
#endif

	cn_tab = &rza1ucons;
	cn_init_magic(&rza1ucons_cnm_state);
	cn_set_magic("\047\001"); /* default magic is BREAK */

	rza1ucons_rate = rate;
	rza1ucons_cflag = cflag;
	rza1ucons_regs = regs;

	return 0;
}

int
rza1ucngetc(dev_t dev)
{
//	led(2);
	return rza1uart_common_getc(dev, &rza1ucons_regs);
}

/*
 * Console kernel output character routine.
 */
void
rza1ucnputc(dev_t dev, int c)
{
//	led(1);
	rza1uart_common_putc(dev, &rza1ucons_regs, c);
//	led(3);
}

void
rza1ucnpollc(dev_t dev, int on)
{
//	led(4);
	rza1uart_readahead_in = 0;
	rza1uart_readahead_out = 0;
}

#endif	/* RZA1UARTCONSOLE */

int
rza1uart_open(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct rza1uart_softc *sc;
	struct tty *tp;
	int s;
	int error;

	sc = device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	if (sc == NULL || !ISSET(sc->sc_hwflags, RZA1UART_HW_DEV_OK))
		return ENXIO;
	if (!device_is_active(sc->sc_dev))
		return ENXIO;

	tp = sc->sc_tty;

	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp))
		return EBUSY;

	s = spltty();

	/*
	 * Do the following iff this is a first open.
	 */
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		struct termios t;

		tp->t_dev = dev;

		/*
		 * Initialize the termios status to the defaults.  Add in the
		 * sticky bits from TIOCSFLAGS.
		 */
		if (ISSET(sc->sc_hwflags, RZA1UART_HW_CONSOLE)) {
			t.c_ospeed = rza1ucons_rate;
			t.c_cflag = rza1ucons_cflag;
		} else {
			t.c_ospeed = TTYDEF_SPEED;
			t.c_cflag = TTYDEF_CFLAG;
		}
		t.c_ispeed = t.c_ospeed;
//		if (ISSET(sc->sc_swflags, TIOCFLAG_CLOCAL))
//			SET(t.c_cflag, CLOCAL);
//		if (ISSET(sc->sc_swflags, TIOCFLAG_CRTSCTS))
//			SET(t.c_cflag, CRTSCTS);
//		if (ISSET(sc->sc_swflags, TIOCFLAG_MDMBUF))
//			SET(t.c_cflag, MDMBUF);
		/* Make sure imxuparam() will do something. */
		tp->t_ospeed = 0;
//		(void) imxuparam(tp, &t);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		ttychars(tp);
		ttsetwater(tp);

		mutex_spin_enter(&sc->sc_lock);

		/*
		 * Turn on DTR.  We must always do this, even if carrier is not
		 * present, because otherwise we'd have to use TIOCSDTR
		 * immediately after setting CLOCAL, which applications do not
		 * expect.  We always assert DTR while the device is open
		 * unless explicitly requested to deassert it.
		 */
//		imxuart_modem(sc, 1);

		/* Clear the input ring, and unblock. */
//		sc->sc_rbuf_in = sc->sc_rbuf_out = 0;
//		imxuart_iflush(sc);
//		CLR(sc->sc_rx_flags, IMXUART_RX_ANY_BLOCK);
//		imxuart_hwiflow(sc);

		/* Turn on interrupts. */
//		imxuart_control_rxint(sc, true);

		mutex_spin_exit(&sc->sc_lock);
	}

	splx(s);

	error = ttyopen(sc->sc_tty, TTDIALOUT(dev), ISSET(flag, O_NONBLOCK));
	if (error)
		goto bad;

	error = (*tp->t_linesw->l_open)(dev, tp);

	if (error)
		goto bad;

	return 0;

bad:
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		/*
		 * We failed to open the device, and nobody else had it opened.
		 * Clean up the state as appropriate.
		 */
//		imxuart_shutdown(sc);
	}

	return error;
}

int
rza1uart_close(dev_t dev, int flag, int mode, struct lwp *l)
{
	return 0;
}

int
rza1uart_read(dev_t dev, struct uio *uio, int flag)
{
	struct rza1uart_softc *sc =
	    device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (sc->enabled != 0 &&
	    device_is_active(sc->sc_dev) == 0)
		return (EIO);

	led(5);
	return (*tp->t_linesw->l_read)(tp, uio, flag);
}

int
rza1uart_write(dev_t dev, struct uio *uio, int flag)
{
	struct rza1uart_softc *sc =
	    device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (sc->enabled != 0 && device_is_active(sc->sc_dev) == 0)
		return (EIO);

	led(6);
	return (*tp->t_linesw->l_write)(tp, uio, flag);
}


int
rza1uart_poll(dev_t dev, int events, struct lwp *l)
{
	struct rza1uart_softc *sc =
	    device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (sc->enabled != 0 && device_is_active(sc->sc_dev) == 0)
		return (POLLHUP);

	led(7);
	return (*tp->t_linesw->l_poll)(tp, events, l);
}

struct tty *
rza1uart_tty(dev_t dev)
{
	struct rza1uart_softc *sc =
	    device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	struct tty *tp = sc->sc_tty;

	return tp;
}

int
rza1uart_ioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct rza1uart_softc *sc;
	struct tty *tp;
	int error;

	sc = device_lookup_private(&rza1uart_cd, TTUNIT(dev));
	if (sc == NULL)
		return EIO;
	if (sc->enabled != 0 && device_is_active(sc->sc_dev) == 0)
		return EIO;

	tp = sc->sc_tty;
	error = (*tp->t_linesw->l_ioctl)(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	error = ttioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	return EPASSTHROUGH;
}

/*
 * Stop output on a line.
 */
void
rza1uart_stop(struct tty *tp, int flag)
{
}

void
rza1uart_set_frequency(u_int freq, u_int div)
{
}
