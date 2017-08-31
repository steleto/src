/*	$NetBSD$	*/

#ifndef ARM_RZA1_REG_H
#define ARM_RZA1_REG_H

#define RZA1_IOREG_SIZE		(RZA1_SPI_SIZE + \
				 RZA1_IO0_SIZE + \
				 RZA1_IO1_SIZE + \
				 RZA1_IO2_SIZE + \
				 RZA1_IO3_SIZE + \
				 RZA1_IO4_SIZE)

#define RZA1_IO_SIZE		(RZA1_IOREG_SIZE + RZA1_ARMCORE_SIZE)

#define RZA1_SPI_BASE		RZA1_SPI0_BASE
#define RZA1_SPI_SIZE		(RZA1_SPI0_SIZE + RZA1_SPI1_SIZE)

#define RZA1_SPI0_BASE		0x18000000
#define RZA1_SPI0_SIZE		0x04000000

#define RZA1_SPI1_BASE		0x1c000000
#define RZA1_SPI1_SIZE		0x04000000

#define RZA1_IO0_BASE		0x3fe00000
#define RZA1_IO0_SIZE		0x00200000

#define RZA1_IO1_BASE		0xe8000000
#define RZA1_IO1_SIZE  		0x00300000

#define RZA1_ARMCORE_BASE	0xf0000000
#define RZA1_ARMCORE_SIZE	0x00100000

#define RZA1_IO2_BASE		0xfc000000
#define RZA1_IO2_SIZE		0x00100000

#define RZA1_IO3_BASE		0xfcf00000
#define RZA1_IO3_SIZE		0x00100000

#define RZA1_IO4_BASE		0xfff00000
#define RZA1_IO4_SIZE		0x00100000

/*
 * Serial Communication Interface with FIFO (SCIF)
 */
#define RZA1_SCIF_BASE(n)	(0xe8007000 + 0x800 * (n))
#define RZA1_SCIF_SIZE		0x2c

#define RZA1_SCSMR		0x00 /* serial mode */
#define RZA1_SCBRR		0x04 /* bit rate */
#define RZA1_SCSCR		0x08 /* serial control */
#define RZA1_SCFTDR		0x0c /* transmit fifo data */
#define RZA1_SCFSR		0x10 /* serial status */
#define RZA1_SCFRDR		0x14 /* receive fifo data */
#define RZA1_SCFCR		0x18 /* fifo control */
#define RZA1_SCFDR		0x1c /* fifo data count set */
#define RZA1_SCSPTR		0x20 /* serial port */
#define RZA1_SCLSR		0x24 /* line status */
#define RZA1_SCEMR		0x28 /* serial extend mode */

#define RZA1_SCSCR_TIE		0x0080 /* transmit interrupt enable */
#define RZA1_SCSCR_RIE		0x0040 /* receive interrupt enable */
#define RZA1_SCSCR_TE		0x0020 /* transmit enable */
#define RZA1_SCSCR_RE		0x0010 /* receive enable */
#define RZA1_SCSCR_REIE		0x0008 /* receive error interrupt enable */
#define RZA1_SCSCR_CKE1		0x0002 /* clock enable 1-0 */
#define RZA1_SCSCR_CKE0		0x0001 /* clock enable 1-0 */

#define RZA1_SCFSR_PER3		0x8000 /* parity error count 3-0 */
#define RZA1_SCFSR_PER2		0x4000
#define RZA1_SCFSR_PER1		0x2000
#define RZA1_SCFSR_PER0		0x1000
#define RZA1_SCFSR_FER3		0x0800 /* framing error count 3-0 */
#define RZA1_SCFSR_FER2		0x0400
#define RZA1_SCFSR_FER1		0x0200
#define RZA1_SCFSR_FER0		0x0100
#define RZA1_SCFSR_ER		0x0080 /* reveice error */
#define RZA1_SCFSR_TEND		0x0040 /* transmit end */
#define RZA1_SCFSR_TDFE		0x0020 /* transmit data fifo empty */
#define RZA1_SCFSR_BRK		0x0010 /* break detection */
#define RZA1_SCFSR_FER		0x0008 /* framing error */
#define RZA1_SCFSR_PER		0x0004 /* parity error */
#define RZA1_SCFSR_RDF		0x0002 /* receive fifo data full */
#define RZA1_SCFSR_DR		0x0001 /* receive data ready */

#define RZA1_SCFCR_RSTRG2	0x0400 /* RTS output active trigger 2-0 */
#define RZA1_SCFCR_RSTRG1	0x0200
#define RZA1_SCFCR_RSTRG0	0x0100
#define RZA1_SCFCR_RTRG1	0x0080 /* receive data count trigger 1-0 */
#define RZA1_SCFCR_RTRG0	0x0040
#define RZA1_SCFCR_TTRG1	0x0020 /* transmit data count trigger 1-0 */
#define RZA1_SCFCR_TTRG0	0x0010
#define RZA1_SCFCR_MCE		0x0008 /* model control enable */
#define RZA1_SCFCR_TFRST	0x0004 /* transmit fifo register reset */
#define RZA1_SCFCR_RFRST	0x0002 /* receive fifo register reset */
#define RZA1_SCFCR_LOOP		0x0001 /* loop back test */

#define	FIFO_RCV_TRIGGER_1	0x0000
#define	FIFO_RCV_TRIGGER_4	0x0040
#define	FIFO_RCV_TRIGGER_8	0x0080
#define	FIFO_RCV_TRIGGER_14	0x00c0
#define	FIFO_XMT_TRIGGER_8	0x0000
#define	FIFO_XMT_TRIGGER_4	0x0010
#define	FIFO_XMT_TRIGGER_2	0x0020
#define	FIFO_XMT_TRIGGER_1	0x0030

#define RZA1_SCFDR_TXCNT	0x1f00 /* transmit count */
#define RZA1_SCFDR_RXCNT	0x001f /* receive count */
#define	RZA1_SCFDR_TX_FULL	0x1000	/* transmit full */
#define	RZA1_SCFDR_RX_EPTY	0x0000	/* receive empty */

#define RZA1_SCLSR_ORER		0x0001	/* overrun error */

#endif	/* !ARM_RZA1_REG_H */
