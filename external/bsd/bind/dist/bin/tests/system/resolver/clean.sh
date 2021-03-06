#!/bin/sh
#
# Copyright (C) 2008-2015  Internet Systems Consortium, Inc. ("ISC")
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
# OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
# PERFORMANCE OF THIS SOFTWARE.

#
# Clean up after resolver tests.
#
rm -f */named.memstats
rm -f */named.run
rm -f */ans.run
rm -f */*.jdb
rm -f dig.out dig.out.*
rm -f dig.*.out.*
rm -f dig.*.foo.*
rm -f dig.*.bar.*
rm -f dig.*.prime.*
rm -f ns4/tld.db
rm -f ns6/K*
rm -f ns6/example.net.db.signed ns6/example.net.db
rm -f ns6/dsset-example.net. ns6/example.net.db.signed.jnl
rm -f ns6/to-be-removed.tld.db ns6/to-be-removed.tld.db.jnl
rm -f ns7/server.db ns7/server.db.jnl ns7/named.conf
rm -f resolve.out
rm -f .digrc
