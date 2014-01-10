/*
 *	ev3dev_dev.h -- some device specific structures/functions for the EV3 infrastructure by Ralph Hempel.
 *	Copyright (C) 2014 Fred Barnes <frmb@kent.ac.uk>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef __EV3DEV_DEV_H
#define __EV3DEV_DEV_H

enum {
	EV3IO_SYSCFG0,
	EV3IO_EHRPWM1,
	EV3IO_ECAP0,
	EV3IO_ECAP1,
	EV3IO_TIMER64P3,
	EV3IO_PSC1,
	EV3IO_COUNT
};

struct __ev3dev_regioninfo {
	const char *name;
	unsigned int base;
	unsigned int size;
};

/* device info structure, available to sub-modules, provides mapped I/O memory locations */
extern ULONG **ev3dev_get_mmio_regions (void);
extern void ev3dev_release_mmio_regions (void);


#endif	/* !__EV3DEV_DEV_H */

