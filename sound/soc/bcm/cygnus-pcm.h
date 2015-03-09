/*
 * Copyright (C) 2014-2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __CYGNUS_PCM_H__
#define __CYGNUS_PCM_H__

struct ringbuf_regs {
	unsigned rdaddr;
	unsigned wraddr;
	unsigned baseaddr;
	unsigned endaddr;
	unsigned fmark;   /* freemark for play, fullmark for caputure */
	unsigned period_bytes;
	unsigned buf_size;
};

#define RINGBUF_REG_PLAYBACK(num) ((struct ringbuf_regs) { \
	.rdaddr = SRC_RBUF_ ##num## _RDADDR_OFFSET, \
	.wraddr = SRC_RBUF_ ##num## _WRADDR_OFFSET, \
	.baseaddr = SRC_RBUF_ ##num## _BASEADDR_OFFSET, \
	.endaddr = SRC_RBUF_ ##num## _ENDADDR_OFFSET, \
	.fmark = SRC_RBUF_ ##num## _FREE_MARK_OFFSET, \
	.period_bytes = 0, \
	.buf_size = 0, \
})

#define RINGBUF_REG_CAPTURE(num) ((struct ringbuf_regs)  { \
	.rdaddr = DST_RBUF_ ##num## _RDADDR_OFFSET, \
	.wraddr = DST_RBUF_ ##num## _WRADDR_OFFSET, \
	.baseaddr = DST_RBUF_ ##num## _BASEADDR_OFFSET, \
	.endaddr = DST_RBUF_ ##num## _ENDADDR_OFFSET, \
	.fmark = DST_RBUF_ ##num## _FULL_MARK_OFFSET, \
	.period_bytes = 0, \
	.buf_size = 0, \
})
#endif
