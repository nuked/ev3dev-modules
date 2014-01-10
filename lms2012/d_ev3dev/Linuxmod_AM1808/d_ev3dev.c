/*
 * Copyright (c) 2013-2013 - Ralph Hempel - based on original code attributed
 *                                          to the copyright holder(s) below:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* -----------------------------------------------------------------------
 * Every ev3dev_xx module needs access to the ev3dev device node, which
 * is created by this module.
 *
 * When an ev3dev_xx module loads a check is made for the "ev3dev" platform
 * device node. If that node does not exist, then the ev3dev_xx module will
 * fail to load.
 *
 * The ev3dev_xx module must register the xx device as a child of the
 * ev3dev device, and then register any additional instances of subdevices
 * or device attributes.
 *
 * The result is a tree that looks like this, for example:
 *
 * sys/devices/platform/ev3dev/pwm/motorA
 *                                /motorB 
 *
 * Key functions that make this work: device_find_child()
 *                                    device foreach child()
 *
 * The ev3dev_release function is responsible for cleaning up the 
 * ev3dev entry when the last module is unloaded.
 *
 * When the ev3dev module that holds the ev3dev_device structure is
 * unloaded, a check is made for modules that still are registered
 * under the ev3dev node. If there are still device children
 * registered the module will not unload.
 */

/* Modified 2014, Fred Barnes:
 *	separate out request of I/O memory regions to here.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <asm/gpio.h>

#include "lmstypes.h"
#include "ev3dev_util.h"
#include "ev3dev_dev.h"


/*{{{  private global data*/

/*}}}*/


/*{{{  static void ev3dev_release (struct device *dev)*/
/*
 *	device release.
 */
static void ev3dev_release (struct device *dev)
{
       printk( "ev3dev_release\n");
}
/*}}}*/
/*{{{  ev3dev_device (struct platform_device), ev3dev (platform_device *)*/
static struct platform_device ev3dev_device = {
        .name        = "ev3dev",
        .id          = -1,
        .dev.release = ev3dev_release,
};

struct platform_device *ev3dev = NULL;
EXPORT_SYMBOL_GPL(ev3dev);

/*}}}*/
/*{{{  ev3dev_regioninfo[IO_COUNT] (struct __ev3dev_regioninfo), ev3dev_ioregions (ULONG **), ev3dev_mmiocount (int)*/
static struct __ev3dev_regioninfo ev3dev_regioninfo[EV3IO_COUNT] = {
	{"SYSCFG0",	0x01C14000,	0x190},
	{"EHRPWM1",	0x01F02000,	0x2854},
	{"ECAP0",	0x01F06000,	0x60},
	{"ECAP1",	0x01F07000,	0x60},
	{"TIMER64P3",	0x01F0D000,	0x80},
	{"PSC1",	0x01E27000,	0xA80}
};

static ULONG *ev3dev_ioregions[EV3IO_COUNT];
static int ev3dev_mmiocount;

/*}}}*/

/*{{{  static int ev3dev_init_regions (void)*/
/*
 *	initialises memory-mapped I/O bits, populating ev3dev_ioregions from ev3dev_regioninfo.
 *	returns 0 on success, non-zero on failure.
 */
static int ev3dev_init_regions (void)
{
	int i;
	int ret = 0;

	ev3dev_mmiocount = 0;

	for (i=0; i<EV3IO_COUNT; i++) {
		void *p = request_mem_region (ev3dev_regioninfo[i].base, ev3dev_regioninfo[i].size, "ev3dev");
		if (!p) {
			printk (KERN_ERR "ev3dev: failed to request memory for region '%s', error %d\n", ev3dev_regioninfo[i].name, ret);
			goto out_err1;
		}

		ev3dev_ioregions[i] = (ULONG *)ioremap (ev3dev_regioninfo[i].base, ev3dev_regioninfo[i].size);
		if (!ev3dev_ioregions[i]) {
			printk (KERN_ERR "ev3dev: failed to remap I/O memory for region '%s'\n", ev3dev_regioninfo[i].name);
			ret = -EIO;
			goto out_err2;
		}
	}

	return 0;

out_err2:
	/* release the current one first; mapping failed */
	release_mem_region (ev3dev_regioninfo[i].base, ev3dev_regioninfo[i].size);
out_err1:
	/* release all the ones we would have done already */
	for (i--; i>=0; i--) {
		iounmap (ev3dev_ioregions[i]);
		ev3dev_ioregions[i] = NULL;
		release_mem_region (ev3dev_regioninfo[i].base, ev3dev_regioninfo[i].size);
	}
	return ret;
}
/*}}}*/
/*{{{  static void ev3dev_free_regions (void)*/
/*
 *	frees memory-mapped I/O bits.
 */
static void ev3dev_free_regions (void)
{
	int i;

	if (ev3dev_mmiocount) {
		printk (KERN_ERR "ev3dev_free_regions(): not freeing because mmiocount=%d (still in-use, bug..)", ev3dev_mmiocount);
		return;
	}

	for (i=0; i<EV3IO_COUNT; i++) {
		if (ev3dev_ioregions[i]) {
			iounmap (ev3dev_ioregions[i]);
			ev3dev_ioregions[i] = NULL;
			release_mem_region (ev3dev_regioninfo[i].base, ev3dev_regioninfo[i].size);
		}
	}
}
/*}}}*/

/*{{{  ULONG **ev3dev_get_mmio_regions (void)*/
/*
 *	called to request access to EV3 I/O spaces.
 */
ULONG **ev3dev_get_mmio_regions (void)
{
	if (!ev3dev) {
		return NULL;
	}
	ev3dev_mmiocount++;
	return (ULONG **)ev3dev_ioregions;
}

EXPORT_SYMBOL_GPL (ev3dev_get_mmio_regions);

/*}}}*/
/*{{{  void ev3dev_release_mmio_regions (void)*/
/*
 *	called to give up access to EV3 I/O spaces.
 */
void ev3dev_release_mmio_regions (void)
{
	ev3dev_mmiocount--;
}

EXPORT_SYMBOL_GPL (ev3dev_release_mmio_regions);

/*}}}*/


/*{{{  static int ev3dev_init(void)*/
/*
 *	initialises the base EV3 driver.
 *	returns 0 on success, non-zero on error.
 */
static int ev3dev_init(void)
{
	int ret;

	ret = ev3dev_init_regions ();
	if (ret) {
		printk ("ev3dev_init(): failed to get I/O regions, giving up");
		return ret;
	}

	ret = platform_device_register (&ev3dev_device);
	ev3dev = &ev3dev_device;

	printk( "ev3dev_init registers ev3dev at %08x\n", (unsigned int)&ev3dev_device);

	return ret;
}
/*}}}*/
/*{{{  static void ev3dev_exit(void)*/
/*
 *	called when exiting.
 */
static void ev3dev_exit(void)
{
	//  int children = 0;

       // struct device *dev;

       // Check to see if the "ev3dev" device is still registered...
       //
       // dev = device_find_child( &platform_bus, "ev3dev", match_child );

       // printk( "ev3dev_unregister search for ev3dev platform child %08x\n", (unsigned int)dev);

       // printk( "ev3dev_unregister local   ev3dev_device is %08x\n", (unsigned int)&ev3dev_device );
       // printk( "ev3dev_unregister passed  ev3dev_device is %08x\n", (unsigned int)*pdev          );

       // if ( NULL != dev ) {

       //     device_for_each_child( dev, &children, count_child );

        //    printk( "ev3dev_unregister %08x still has %d children\n", (unsigned int)dev, children );
        
         //   if( 0 == children ) {
        //      platform_device_unregister( to_platform_device(dev) );

	ev3dev = NULL;
	platform_device_unregister( &ev3dev_device );

	ev3dev_free_regions ();

//              printk( "ev3dev_unregister result  %08x\n", (unsigned int)(ev3dev_device.dev) );

      //          if( NULL != pdev ) {
       //            *pdev  = NULL;
        //        }
       //     }
     //   }
	return;
}
/*}}}*/


module_init(ev3dev_init);
module_exit(ev3dev_exit)

MODULE_AUTHOR("Ralph Hempel <rhempel@hempeldesigngroup.com>, Fred Barnes <frmb@kent.ac.uk>");
MODULE_DESCRIPTION("Driver node for LEGO MINDSTORMS EV3");
MODULE_LICENSE("GPL");

