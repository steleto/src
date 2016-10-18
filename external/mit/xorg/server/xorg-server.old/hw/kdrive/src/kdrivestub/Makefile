#	$NetBSD$

.include "../../../../Makefile.serverlib"
.include "../../../../Makefile.servermod"

LIB=	kdrivestub
SHLIB_MAJOR=0

.PATH:	${X11SRCDIR.xorg-server}/fb
SRCS+=	fbcmap_mi.c

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

CPPFLAGS+=	-DHAVE_CONFIG_H -DHAVE_DIX_CONFIG_H -DHAVE_KDRIVE_CONFIG_H

.include "../../../../Makefile.Xserver"

.include <bsd.x11.mk>
.include <bsd.lib.mk>
