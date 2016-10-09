#	$NetBSD$

.include <bsd.own.mk>

PROG=		Xwsfb
BINMODE=	4711

.include "../../../../Makefile.serverlib"
.include "../../../../Makefile.servermod"

.PATH:	${X11SRCDIR.xorg-server}/hw/netbsd/kdrive/wsfb
SRCS=	wsfbinit.c wsfb.c

#.PATH:  ${X11SRCDIR.xorg-server}/Xext
#SRCS+=  dpmsstubs.c

CPPFLAGS+=	-I${X11SRCDIR.xorg-server}/include \
		-I${X11SRCDIR.xorg-server}/hw/kdrive/src \
		-I${X11SRCDIR.xorg-server}/fb \
		-I${X11SRCDIR.xorg-server}/mi \
		-I${X11SRCDIR.xorg-server}/miext/damage \
		-I${X11SRCDIR.xorg-server}/miext/shadow \
		-I${X11SRCDIR.xorg-server}/randr \
		-I${X11SRCDIR.xorg-server}/render \
		-I${X11SRCDIR.xorg-server}/Xi \
		-I${DESTDIR}${X11INCDIR}/X11 \

CPPFLAGS+=	-DHAVE_CONFIG_H -DHAVE_KDRIVE_CONFIG_H

.include "../../../../Makefile.Xserver"

LDADD+=	${LDADD.dix} \
	${LDADD.hw/kdrive/src/kdrive} \
	${LDADD.hw/netbsd/kdrive/wscons} \
	${LDADD.Xext} \
	${LDADD.Xi} \
	${LDADD.glx} \
	${LDADD.damageext} \
	${LDADD.composite} \
	${LDADD.randr} \
	${LDADD.xfixes} \
	${LDADD.dbe} \
	${LDADD.record} \
	${LDADD.fb} \
	${LDADD.hw/kdrive/src/kdrivestub} \
	${LDADD.miext/sync} \
	${LDADD.miext/damage} \
	${LDADD.miext/shadow} \
	${LDADD.xkb} \
	${LDADD.xkbstubs} \
	${LDADD.render} \
	${LDADD.os} \
	${LDADD.mi} \
	-lXfont \
	-lpixman-1 \
	-lXext \
	-lX11 \
	-lXdmcp \
	-lXau

DPADD+= ${DPADD.dix} \
	${DPADD.hw/kdrive/src/kdrive} \
	${DPADD.hw/netbsd/kdrive/wscons} \
	${DPADD.Xext} \
	${DPADD.Xi} \
	${DPADD.xkb} \
	${DPADD.glx} \
	${DPADD.damageext} \
	${DPADD.composite} \
	${DPADD.randr} \
	${DPADD.xfixes} \
	${DPADD.dbe} \
	${DPADD.record} \
	${DPADD.fb} \
	${DPADD.hw/kdrive/src/kdrivestub} \
	${DPADD.miext/sync} \
	${DPADD.miext/damage} \
	${DPADD.miext/shadow} \
	${DPADD.xkbstubs} \
	${DPADD.render} \
	${DPADD.os} \
	${DPADD.mi} \
	${LIBXFONT} \
	${LIBPIXMAN-1} \
	${LIBXEXT} \
	${LIBX11} \
	${LIBXDMCP} \
	${LIBXAU}

.include <bsd.x11.mk>
.include <bsd.prog.mk>
