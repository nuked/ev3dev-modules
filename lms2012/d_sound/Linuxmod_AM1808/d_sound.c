/*
 * LEGO® MINDSTORMS EV3
 *
 * Copyright (C) 2010-2013 The LEGO Group
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

/*
 *	folded + updated for Ralph Hempel's ev3dev infrastructure.
 */

/*! \page Sound Driver Module
 *
 *
 *-  \subpage SoundDriverModuleResources
 */


#include "lms2012.h"
#include "am1808.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>

//#include  <linux/kernel.h>
//#include  <linux/fs.h>
//#include  <linux/sched.h>
//
//#include  <linux/hrtimer.h>
//
//#include  <linux/mm.h>
//#include  <linux/hrtimer.h>
//
//#include  <linux/init.h>
//#include  <linux/uaccess.h>
//#include  <linux/debugfs.h>

//#include  <linux/ioport.h>
#include  <asm/gpio.h>
//#include  <asm/io.h>
//#include  <linux/module.h>
//#include  <linux/miscdevice.h>
//#include  <asm/uaccess.h>

#include <mach/mux.h>

#include "ev3dev_util.h"

/*{{{  module constants*/
#define DEVICE_NAME "ev3dev_sound"

/*}}}*/
/*{{{  timer-mode constants (enumerated)*/

enum {
	TIMING_SAMPLES,
	ONE_SHOT,
	MANUAL,
	READY_FOR_SAMPLES,
	IDLE
} TIMER_MODES;

/*}}}*/

/*{{{  global variables*/
static ULONG *SYSCFG0;
static ULONG *PSC1;
static UWORD *eHRPWM0;

static UWORD Duration;
static UWORD Period;
static UBYTE Level;
static UBYTE TimerMode = READY_FOR_SAMPLES;
static SOUND SoundDefault;
static SOUND *pSound = &SoundDefault;

static struct hrtimer Device1Timer;
static ktime_t Device1Time;

/*}}}*/
/*{{{  defines related to PWM output*/

#define CFGCHIP0          0x60	// 32 bit wide index into SYSCFG0
#define PLL_MASTER_LOCK   0x00001000
#define DEFAULT_FREQUENCY 16499	// 16499 should give 8 KHz
#define DEFAULT_LEVEL     1000	// 25% D.C.

/*}}}*/
/*{{{  sound-buffer*/

static UWORD SoundBuffers[SOUND_BUFFER_COUNT][SOUND_FILE_BUFFER_SIZE];
static UBYTE SoundChunkSize[SOUND_BUFFER_COUNT];
static UWORD BufferReadPointer;
static UBYTE BufferReadIndex;
static UBYTE BufferWriteIndex;

/*}}}*/

/*{{{  TBCTL (timer-base control) bits*/

#define   TB_COUNT_UP                   0x0	// TBCNT MODE bits
#define   TB_DISABLE                    0x0	// PHSEN bit
#define   TB_ENABLE                     0x4
#define   TB_SHADOW                     0x0	// PRDLD bit
#define   TB_IMMEDIATE                  0x8
#define   TB_SYNC_DISABLE               0x30	// SYNCOSEL bits
#define   TB_HDIV1                      0x0	// HSPCLKDIV bits
#define   TB_DIV1                       0x0	// CLKDIV bits
#define   TB_UP                         0x2000	// PHSDIR bit

/*}}}*/
/*{{{  CMPCTL (compare control) bits*/

#define   CC_CTR_A_ZERO                 0x0	// LOADAMODE bits
#define   CC_CTR_B_ZERO                 0x0
#define   CC_A_SHADOW                   0x00	// SHDWAMODE and SHDWBMODE bits
#define   CC_B_SHADOW                   0x00
#define   CC_B_NO_SHADOW                0x40

#define   TBCTL                         0x0
#define   TBPHS                         0x3
#define   TBCNT                         0x4
#define   TBPRD                         0x5
#define   CMPCTL                        0x7
#define   CMPA                          0x9
#define   CMPB                          0xA
#define   AQCTLA                        0xB
#define   AQCTLB                        0xC

/*}}}*/
/*{{{  action-qualifier output B control register (AQCTLB)*/

//                              1111 11
//                        bit   5432 1098 7654 3210

//      Bit 15 Reserved         R000 0000 0000 0000
//      Bit 14 -                0R00 0000 0000 0000
//      Bit 13 -                00R0 0000 0000 0000
//      Bit 12 -                000R 0000 0000 0000
#define SOUND_RESERVED          0x0000

//      Bit 11 CBD              0000 0000 0000 0000
//      Bit 10 -                0000 0100 0000 0000 CLEAR Forces EPWM0B LOW
#define SOUND_CBD               0x0400

//      Bit  9 CBU              0000 0000 0000 0000
//      Bit  8 -                0000 0001 0000 0000 CLEAR Forces EPWM0B LOW
#define SOUND_CBU               0x0100

//      Bit  7 CAD              0000 0000 N000 0000 DO NOTHING
//      Bit  6 -                0000 0000 0N00 0000 -
#define SOUND_CAD               0x0000

//      Bit  5 CAU              0000 0000 00N0 0000 DO NOTHING
//      Bit  4 -                0000 0000 000N 0000 -
#define SOUND_CAU               0x0000

//      Bit  3 PRD              0000 00N0 0000 1000 Forces EPWM0B HIGH
//      Bit  2 -                0000 000N 0000 0000 -
#define SOUND_PRD               0x0008

//      Bit  1 ZRO              0000 00N0 0000 0010 Forces EPWM0B HIGH
//      Bit  0 -                0000 000N 0000 0000 -
#define SOUND_ZRO               0x0002




#define SOUND_SYMM_TONE (SOUND_RESERVED +\
                         SOUND_CBD +\
                         SOUND_CBU +\
                         SOUND_CAD +\
                         SOUND_CAU +\
                         SOUND_PRD +\
                         SOUND_ZRO)


/*}}}*/

/*{{{  macros for hardware PWM output*/

// Put the "Magic key" into KICK0R and KICK1R to open for manipulation
// Stop Time Base Clock (TBCLK) - clear bit TBCLKSYNC in CFGCHIP1
// Remove the "Magic key" from KICK0R and KICK1R to lock

/*{{{  EHRPWMClkDisable ()*/
#define   EHRPWMClkDisable              {\
                                          eHRPWM0[TBCTL]  = 0xC033;\
                                        }
/*}}}*/

// Put the "Magic key" into KICK0R and KICK1R to open for manipulation
// Start Time Base Clock (TBCLK) - set bit TBCLKSYNC in CFGCHIP1
// Remove the "Magic key" from KICK0R and KICK1R to lock

/*{{{  EHRPWMClkEnable ()*/
#define   EHRPWMClkEnable               {\
                                          eHRPWM0[TBCTL]  = 0xC030; \
                                          iowrite32((ioread32(&SYSCFG0[CFGCHIP0]) | PLL_MASTER_LOCK),&SYSCFG0[0x60]);\
                                        }
/*}}}*/
/*{{{  EHRPWMClkEnableTone ()*/
#define   EHRPWMClkEnableTone           {\
                                          eHRPWM0[TBCTL]  = 0xDC30;\
                                          iowrite32((ioread32(&SYSCFG0[CFGCHIP0]) | PLL_MASTER_LOCK),&SYSCFG0[0x60]);\
                                        }
/*}}}*/
/*{{{  SETPwmPeriod (Prd)*/
#define   SETPwmPeriod(Prd)               {\
                                          eHRPWM0[TBPRD] = Prd; /* A factor of sample-rate  */\
                                                                /* For lowering the quanti- */\
                                                                /* zation noise (simpler    */\
                                                                /* filtering)               */\
                                        }
/*}}}*/
/*{{{  SETSoundLevel (Level)*/
#define   SETSoundLevel(Level)          {\
                                          eHRPWM0[CMPB] = Level;  /* The amplitude for this */\
                                                                  /* very and/or following  */\
                                                                  /* samples/periods        */\
                                        }
/*}}}*/
/*{{{  STOPPwm ()*/
#define   STOPPwm                       {\
                                          iowrite16(0x00, &eHRPWM0[TBCTL]);\
                                          iowrite16(0x00, &eHRPWM0[CMPCTL]);\
                                          EHRPWMClkDisable;\
                                        }
/*}}}*/
/*{{{  SOUNDPwmModuleSetupPcm ()*/
#define   SOUNDPwmModuleSetupPcm        { \
                                          \
                                          /* eHRPWM Module */\
                                          /*  In eHRPWM1[TBCTL] TB_DISABLE, TB_SHADOW, TB_HDIV1, TB_DIV1, TB_COUNT_UP all cleared */\
                                          EHRPWMClkDisable;\
                                          eHRPWM0[TBPHS]  = 0; /* Phase register cleared */\
                                          eHRPWM0[TBCNT]  = 0; /* Clear TB counter */\
/*                                        eHRPWM0[TBCTL]  = 0xC030;*/\
                                          eHRPWM0[CMPCTL] = (CC_B_SHADOW | CC_CTR_B_ZERO);\
                                          eHRPWM0[AQCTLB] = 0x0102;\
                                          EHRPWMClkEnable;\
                                        }
/*}}}*/
/*{{{  SOUNDPwmModuleSetupTone ()*/
#define   SOUNDPwmModuleSetupTone        { \
                                          \
                                          /* eHRPWM Module */\
                                          /*  In eHRPWM1[TBCTL] TB_DISABLE, TB_SHADOW, TB_HDIV1, TB_DIV1, TB_COUNT_UP all cleared */\
                                          EHRPWMClkDisable;\
                                          eHRPWM0[TBPHS]  = 0; /* Phase register cleared */\
                                          eHRPWM0[TBCNT]  = 0; /* Clear TB counter */\
/*                                        eHRPWM0[TBCTL]  = 0xDC30;*/\
                                          eHRPWM0[CMPCTL] = (CC_B_SHADOW | CC_CTR_B_ZERO);\
                                          eHRPWM0[AQCTLB] = 0x0102;\
/*                                        EHRPWMClkEnable;*/\
                                          EHRPWMClkEnableTone;\
                                        }
/*}}}*/
/*{{{  OLDCODE (SOUNDEnable)*/
// #define   SOUNDEnable           {
//                                   (*SoundPin[SOUNDEN].pGpio).set_data    =  SoundPin[SOUNDEN].Mask;
//                                   (*SoundPin[SOUNDEN].pGpio).dir        &= ~SoundPin[SOUNDEN].Mask;
//                                 }
/*}}}*/
/*{{{  SOUNDEnable ()*/
#define   SOUNDEnable           {\
				  gpio_direction_output (legoev3_sound_gpio[SOUNDEN], 1); \
				  gpio_direction_input (legoev3_sound_gpio[SOUNDEN]); \
                                }
/*}}}*/
/*{{{  OLDCODE (SOUNDDisable)*/
//#define   SOUNDDisable          {
//                                  (*SoundPin[SOUNDEN].pGpio).clr_data    =  SoundPin[SOUNDEN].Mask;
//                                  (*SoundPin[SOUNDEN].pGpio).dir        &= ~SoundPin[SOUNDEN].Mask;
//                                }
/*}}}*/
/*{{{  SOUNDDisable ()*/
#define   SOUNDDisable          {\
				  gpio_direction_output (legoev3_sound_gpio[SOUNDEN], 0); \
				  gpio_direction_input (legoev3_sound_gpio[SOUNDEN]); \
                                }
/*}}}*/
/*{{{  SOUNDPwmPoweron ()*/
#define   SOUNDPwmPoweron       {\
                                  iowrite32(0x00000003, &PSC1[0x291]); /* Set ePWM module power on      */\
                                  iowrite32(0x00000003, &PSC1[0x48]);  /* Evaluate all the NEXT fields  */\
                                }
/*}}}*/

/*}}}*/

/*{{{  SoundPins enumeration*/

enum SoundPins {
	SOUNDEN,		// Sound Enable to Amp
	SOUND_ARMA,		// PWM audio signal from ARM
	SOUND_PINS
};

/*}}}*/

/*{{{  SoundPin configuration (FINAL hardware)*/

/*! \page SoundModuleResources Gpios and Resources used for Module
 *
 *  Describes use of gpio and resources\n
 *
 *  \verbatim
 */

#define SND_SOUND_EN GPIO_TO_PIN(6, 15)
#define SND_SOUND_ARMA EPWM0B

#define EV3_SOUND_EN EV3_GPIO6_15
#define EV3_SOUND_ARMA EV3_EPWM0B

static const int legoev3_sound_gpio[SOUND_PINS] = {
	SND_SOUND_EN,
	SND_SOUND_ARMA
};

static const short legoev3_sound_pins[] = {
	EV3_SOUND_EN,
	EV3_SOUND_ARMA,
	(-1)
};

/*  \endverbatim
 *  \n
 */


/*}}}*/
/*{{{  static void sound_init_gpio (void)*/
/*
 *	does GPIO setup.
 *	returns 0 on success, non-zero on failure.
 */
static int sound_init_gpio (void)
{
	int ret = 0;
	int pin;

#warning Move this to board-level init at some point (when d_pwm stuff goes).
	ret = davinci_cfg_reg_list (legoev3_sound_pins);
	if (ret) {
		pr_warning ("ev3dev_sound: legoev3_sound_pins setup failed: %d\n", ret);
		return ret;
	}

	for (pin = 0; pin < SOUND_PINS; pin++) {
		if (legoev3_sound_gpio[pin] >= 0) {
			ret = gpio_request (legoev3_sound_gpio[pin], "ev3dev_sound");
			if (ret) {
				pr_warning ("ev3dev_sound: failed to claim gpio pin 0x%x, gpio_request returned %d\n", legoev3_sound_gpio[pin], ret);
				return ret;
			}
			gpio_direction_input (legoev3_sound_gpio[pin]);
		}
	}
	return 0;
}
/*}}}*/
/*{{{  static void sound_free_gpio (void)*/
/*
 *	releases GPIO resources.
 */
static void sound_free_gpio (void)
{
	int pin;

	for (pin = 0; pin < SOUND_PINS; pin++) {
		if (legoev3_sound_gpio[pin] >= 0) {
			gpio_free (legoev3_sound_gpio[pin]);
		}
	}
}
/*}}}*/

/*{{{  void GetPeripheralBasePtr (ULONG Address, ULONG Size, ULONG **Ptr)*/

/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    GetPeripheralBasePtr
 *
 *  Helper function for getting the peripheral HW base address
 *
 */

void GetPeripheralBasePtr (ULONG Address, ULONG Size, ULONG **Ptr)
{
	/* eCAP0 pointer */
	if (request_mem_region (Address, Size, "ev3dev_sound") >= 0) {

		*Ptr = (ULONG *) ioremap (Address, Size);

		if (*Ptr != NULL) {
			printk ("ev3dev_sound: memory Remapped from 0x%08lX\n", (unsigned long) *Ptr);
		} else {
			printk ("Memory remap ERROR");
		}
	} else {
		printk ("Region request error");
	}
}
/*}}}*/

// DEVICE1 ********************************************************************

/*{{{  static void Device1TimerSetTiming (ULONG Secs, ULONG NanoSecs)*/
/*
 */
static void Device1TimerSetTiming (ULONG Secs, ULONG NanoSecs)
{
	Device1Time = ktime_set (Secs, NanoSecs);
}
/*}}}*/
/*{{{  static void Device1TimerStart (void)*/
/*
 *	(Re) start the high-resolution timer
 */
static void Device1TimerStart (void)
{
	hrtimer_start (&Device1Timer, Device1Time, HRTIMER_MODE_REL);
}
/*}}}*/
/*{{{  static void Device1TimerInitDuration (void)*/
/*
 */
static void Device1TimerInitDuration (void)
{
	UWORD Sec;
	ULONG nSec;

	Sec = (UWORD) (Duration / 1000);	// Whole secs
	nSec = (ULONG) ((Duration - Sec * 1000) * 1000000);	// Raised to mSec
	TimerMode = ONE_SHOT;
	Device1TimerSetTiming (Sec, nSec);
	Device1TimerStart ();	// Start / reStart
}
/*}}}*/
/*{{{  static void Device1TimerCancel (void)*/
/*
 */
static void Device1TimerCancel (void)
{
	TimerMode = IDLE;
	hrtimer_cancel (&Device1Timer);
}
/*}}}*/
/*{{{  static void Device1TimerExit (void)*/
/*
 */
static void Device1TimerExit (void)
{
	Device1TimerCancel ();
}
/*}}}*/

/*{{{  static enum hrtimer_restart Device1TimerInterrupt1 (struct hrtimer *pTimer)*/
/*
 */
static enum hrtimer_restart Device1TimerInterrupt1 (struct hrtimer *pTimer)
{
	UWORD TempLevel;

	enum hrtimer_restart ret = HRTIMER_RESTART;

	if (0 < hrtimer_forward_now (pTimer, Device1Time)) {
		// Take care sample missed!!!!!
	}

	switch (TimerMode) {
	case READY_FOR_SAMPLES:
		if (SoundChunkSize[BufferReadIndex] > 0)	// Any samples D/L yet?
		{

			TimerMode = TIMING_SAMPLES;	// We're ready, next interrupt will
			// bring the sound alive
		}
		break;

	case TIMING_SAMPLES:	// Get new sample - if any

		if (SoundChunkSize[BufferReadIndex] > 0)	// Anything to use in Buffers?
		{
			// Get raw sample
			TempLevel = SoundBuffers[BufferReadIndex][BufferReadPointer++];
			if (TempLevel == 0)
				TempLevel++;
			// Add volume i.e. scale the PWM level

			TempLevel = (UWORD) (Level * TempLevel);
			SETSoundLevel (TempLevel);

			SoundChunkSize[BufferReadIndex]--;

			if (SoundChunkSize[BufferReadIndex] < 1) {
				BufferReadPointer = 0;
				BufferReadIndex++;
				if (BufferReadIndex >= SOUND_BUFFER_COUNT)
					BufferReadIndex = 0;
			}
		} else {
			ret = HRTIMER_NORESTART;

			SOUNDDisable;
			TimerMode = IDLE;
			(*pSound).Status = OK;
			// ret(urn value) already set
		}
		break;

	case ONE_SHOT:		// Stop tone - duration elapsed!
		ret = HRTIMER_NORESTART;
		EHRPWMClkDisable;
		SOUNDDisable;
		TimerMode = IDLE;
		(*pSound).Status = OK;
		// ret(urn value) already set

	case IDLE:
		break;
	case MANUAL:
		break;
	default:
		ret = HRTIMER_NORESTART;
		break;
	}

	return (ret);		// Keep on or stop doing the job
}
/*}}}*/
/*{{{  static void Device1TimerInit8KHz (void)*/
/*
 */
static void Device1TimerInit8KHz (void)
{
	/* Setup 125 uS timer interrupt */

	TimerMode = READY_FOR_SAMPLES;	// Allow for D/L to SoundBuffer(s)
	Device1TimerSetTiming (0, 125000);	// 8 KHz. sample-rate
	Device1TimerStart ();	// Start / reStart
}
/*}}}*/
/*{{{  static ssize_t Device1Write (struct file *File, const char *Buffer, size_t Count, loff_t *Data)*/
/*
 */
static ssize_t Device1Write (struct file *File, const char *Buffer, size_t Count, loff_t *Data)
{

	char CommandBuffer[SOUND_FILE_BUFFER_SIZE + 1];		// Space for command @ byte 0
	UWORD Frequency;
	UWORD PwmLevel;
	int BytesWritten = 0;
	int i = 0;

	if (Count > (SOUND_FILE_BUFFER_SIZE + 1)) {
		Count = (SOUND_FILE_BUFFER_SIZE + 1);
	}

	copy_from_user (CommandBuffer, Buffer, Count);

	switch (CommandBuffer[0]) {
	case SERVICE: /*{{{*/
		if (Count > 1) {
			if (SoundChunkSize[BufferWriteIndex] == 0) {	// Is the requested RingBuffer[Index] ready for write?
				for (i = 1; i < Count; i++) {
					SoundBuffers[BufferWriteIndex][i - 1] = CommandBuffer[i];
				}
				SoundChunkSize[BufferWriteIndex] = Count - 1;
				BufferWriteIndex++;
				if (BufferWriteIndex >= SOUND_BUFFER_COUNT) {
					BufferWriteIndex = 0;
				}
				BytesWritten = Count - 1;
			}
		}
		break;
		/*}}}*/
	case TONE: /*{{{*/
		SOUNDDisable;
		SOUNDPwmModuleSetupTone;

		Level = CommandBuffer[1];
		Frequency = (UWORD) (CommandBuffer[2] + (CommandBuffer[3] << 8));
		Duration = (UWORD) (CommandBuffer[4] + (CommandBuffer[5] << 8));

		if (Frequency < SOUND_MIN_FRQ) {
			Frequency = SOUND_MIN_FRQ;
		} else if (Frequency > SOUND_MAX_FRQ) {
			Frequency = SOUND_MAX_FRQ;
		}

		Period = (UWORD) ((SOUND_TONE_MASTER_CLOCK / Frequency) - 1);
		SETPwmPeriod (Period);

		if (Level > 100) {
			Level = 100;
		}

		PwmLevel = (UWORD) (((ulong) (Period) * (ulong) (Level)) / (ulong) (200));	// Level from % to ticks (Duty Cycle)

		SETSoundLevel (PwmLevel);

		if (Duration > 0) {	// Infinite or specific?
			Device1TimerInitDuration ();
			TimerMode = ONE_SHOT;
		} else {
			TimerMode = MANUAL;
		}

		SOUNDEnable;

		break;
		/*}}}*/
	case REPEAT: /*{{{*/
		// Handled in Green Layer
		break;
		/*}}}*/
	case PLAY: /*{{{*/
		SOUNDDisable;
		SOUNDPwmModuleSetupPcm;

		Level = CommandBuffer[1];
		Period = (UWORD) (SOUND_MASTER_CLOCK / 64000);	// 64 KHz => 8 shots of each sample
		SETPwmPeriod (Period);	// helps filtering

		BufferWriteIndex = 0;	// Reset to first ring Buffer
		BufferReadIndex = 0;	// -
		BufferReadPointer = 0;	// Ready for very first sample

		for (i = 0; i < SOUND_BUFFER_COUNT; i++) {
			SoundChunkSize[i] = 0;	// Reset all Buffer sizes
		}

		Device1TimerInit8KHz ();	// TimerMode = READY_FOR_SAMPLES;
		if (Level != 0) {
			SOUNDEnable;
		}
		// Else we remove any "selfmade" noise
		// Better n/s+n ratio ;-)
		BytesWritten = 2;	// CMD and Level
		break;

		/*}}}*/
	case BREAK: /*{{{*/
		Device1TimerCancel ();
		SOUNDDisable;
		EHRPWMClkDisable;
		TimerMode = IDLE;
		(*pSound).Status = OK;
		break;

		/*}}}*/
	default: /*{{{*/
		break;
		/*}}}*/
	}

	return (BytesWritten);
}
/*}}}*/
/*{{{  static ssize_t Device1Read (struct file *File, char *Buffer, size_t Count, loff_t *Offset)*/
/*
 */
static ssize_t Device1Read (struct file *File, char *Buffer, size_t Count, loff_t *Offset)
{
	int Lng = 0;

	Lng = snprintf (Buffer, Count, "%s\r", DEVICE_NAME);

	return (Lng);
}
/*}}}*/

#define     SHM_LENGTH    (sizeof(SoundDefault))
#define     NPAGES        ((SHM_LENGTH + PAGE_SIZE - 1) / PAGE_SIZE)
static void *kmalloc_ptr;

/*{{{  OLDCODE*/
// static int Device1Mmap (struct file *filp, struct vm_area_struct *vma)
// {
// 	int ret;
// 
// 	ret = remap_pfn_range (vma, vma->vm_start, virt_to_phys ((void *) ((unsigned long) pSound)) >> PAGE_SHIFT, vma->vm_end - vma->vm_start, PAGE_SHARED);
// 
// 	if (ret != 0) {
// 		ret = -EAGAIN;
// 	}
// 
// 	return (ret);
// }
/*}}}*/

/*{{{  struct miscdevice Device1 & fileops*/

static const struct file_operations Device1Entries = {
	.owner = THIS_MODULE,
	.read = Device1Read,
	.write = Device1Write,
//	.mmap = Device1Mmap,
};

static struct miscdevice Device1 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &Device1Entries,
	.mode = 0666,
};

/*}}}*/
/*{{{  static int Device1Init (void)*/
/*
 *	initialises the sound device, registering the miscdevice entry.
 *	returns 0 on success, non-zero on error.
 */
static int Device1Init (void)
{
	int Result = -1;
	int i = 0;
	UWORD *pTmp;

	// Get pointers to memory-mapped registers
	GetPeripheralBasePtr (0x01C14000, 0x190, (ULONG **) & SYSCFG0);	/* SYSCFG0 Pointer */
	GetPeripheralBasePtr (0x01F00000, 0x1042, (ULONG **) & eHRPWM0);	/* eHRPWM0 Pointer */
	GetPeripheralBasePtr (0x01E27000, 0xA80, (ULONG **) & PSC1);	/* PSC1 pointer    */

	Result = misc_register (&Device1);
	if (Result) {
		//#define DEBUG
#undef DEBUG
#ifdef DEBUG
		printk ("  %s device register failed\n", DEVICE_NAME);
#endif
	} else {
		// allocate kernel shared memory for SoundFlags used by Test etc.
		// showing the state of the sound-module used by async and waiting
		// use from VM

		if ((kmalloc_ptr = kmalloc ((NPAGES + 2) * PAGE_SIZE, GFP_KERNEL)) != NULL) {
			pTmp = (UWORD *) ((((unsigned long) kmalloc_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
			for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE) {
				SetPageReserved (virt_to_page (((unsigned long) pTmp) + i));
			}
			pSound = (SOUND *) pTmp;

			SOUNDPwmPoweron;

			/* Setup the Sound PWM peripherals */

			SOUNDPwmModuleSetupPcm;

			/* Setup 125 uS timer interrupt */
			Device1TimerSetTiming (0, 125000);	// Default to 8 KHz. sample-rate
			hrtimer_init (&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			// Timer Callback function - do the sequential job
			Device1Timer.function = Device1TimerInterrupt1;
			Device1TimerCancel ();

			SOUNDDisable;	// Disable the Sound Power Amp

			//#define DEBUG
#undef DEBUG
#ifdef DEBUG
			printk ("  %s device register succes\n", DEVICE_NAME);
#endif

			(*pSound).Status = OK;	// We're ready for making "noise"
		}
	}

	return (Result);
}
/*}}}*/
/*{{{  static void Device1Exit (void)*/
/*
 */
static void Device1Exit (void)
{
	UWORD *pTmp;
	int i = 0;

	Device1TimerExit ();
	misc_deregister (&Device1);

#define DEBUG
#undef DEBUG
#ifdef DEBUG
	printk ("%s exit started\n", MODULE_NAME);
#endif

	iounmap (SYSCFG0);
	iounmap (eHRPWM0);
	iounmap (PSC1);

	// free shared memory
	pTmp = (UWORD *) pSound;
	pSound = &SoundDefault;

	for (i = 0; i < NPAGES * PAGE_SIZE; i += PAGE_SIZE) {
		ClearPageReserved (virt_to_page (((unsigned long) pTmp) + i));

#define DEBUG
#undef DEBUG
#ifdef DEBUG
		printk ("  %s memory page %d unmapped\n", DEVICE_NAME, i);
#endif
	}

	kfree (kmalloc_ptr);


}
/*}}}*/

// MODULE *********************************************************************

/*{{{  definition for EV3 platform device*/
extern struct platform_device *ev3dev;

/* we'll stick something in sysfs for now and link into ev3dev */
static struct platform_device *sound = NULL;

/*}}}*/

/*{{{  static int sound_module_init (void)*/
/*
 *	does local module initialisation.
 *	returns 0 on success, non-zero on error.
 */
static int sound_module_init (void)
{
	int ret;

	/* Note: GPIO region is (should be) handled upstream in ev3dev.
	 * FIXME: still need some clean-up here.
	 * XXX: the conflict in I/O mapping may be causing some issue here.
	 */

// 	if (request_mem_region (DA8XX_GPIO_BASE, 0xD8, MODULE_NAME) >= 0) {
// 		GpioBase = (void *) ioremap (DA8XX_GPIO_BASE, 0xD8);
// 		if (GpioBase != NULL) {
// 			InitGpio ();
// 			Device1Init ();
// 
// 		}
// 	}

	ret = sound_init_gpio ();
	if (ret) {
		/* failed */
		goto out_err0;
	}

	/* initialised GPIO okay, so do the rest */
	ret = Device1Init ();
	if (ret) {
		goto out_err1;
	}

	return 0;

out_err1:
	sound_free_gpio ();
out_err0:
	return ret;
}
/*}}}*/
/*{{{  static void sound_module_exit (void)*/
/*
 *	does local module clean-up.
 */
static void sound_module_exit (void)
{
	SOUNDDisable;		// Disable the Sound Power Amp

	Device1Exit ();
	sound_free_gpio ();

//	iounmap (GpioBase);

	return;
}
/*}}}*/

/*{{{  static int ev3dev_sound_init (void)*/
/*
 *	initialises the sound module.
 *	returns 0 on success, non-zero on failure.
 */
static int ev3dev_sound_init (void)
{
	int ret = 0;

	if (!ev3dev) {
		printk ("ev3dev platform device not yet registered, load the ev3dev module\n");
		ret = -ENODEV;
		goto out_err0;
	}

	sound = platform_device_alloc ("sound", -1);
	if (!sound) {
		printk (KERN_CRIT "failed to allocate ev3dev/sound device\n");
		ret = -ENOMEM;
		goto out_err1;
	}

#if 1
	printk ("  ev3dev_sound_init(): allocated platform device at 0x%8.8x\n", (unsigned int)sound);
#endif

	/* connect */
	sound->dev.parent = &ev3dev->dev;

	ret = platform_device_add (sound);
	if (ret) {
		printk ("failed to register sound device\n");
		goto out_err2;
	}

	ret = sound_module_init ();
	if (ret) {
		printk ("failed to initialise module internals\n");
		goto out_err3;
	}

	return 0;

out_err3:
out_err2:
	platform_device_put (sound);			/* trashes it? */
out_err1:
out_err0:
	return ret;
}
/*}}}*/
/*{{{  static void ev3dev_sound_exit (void)*/
/*
 *	called to clean-up before module exit.
 */
static void ev3dev_sound_exit (void)
{
	sound_module_exit ();

	platform_device_del (sound);
	platform_device_put (sound);
}
/*}}}*/


module_init (ev3dev_sound_init);
module_exit (ev3dev_sound_exit);


MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("The LEGO Group and Fred Barnes <frmb@kent.ac.uk>");
MODULE_DESCRIPTION ("Driver for LEGO Mindstorms EV3 sound device");



