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
 *	this version hacked around by Ralph Hempel;
 *	folding added by Fred.
 */

/*! \page PwmModule PWM Module
 *
 */


#include  "lms2012.h"
#include  "am1808.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <asm/gpio.h>

#if 0
#include <linux/pwm/ehrpwm.h>
#endif

#warning "The following include is only temporary for GetPeripheralBase"
#include <asm/gpio.h>

#include <mach/mux.h>

#include "ev3dev_util.h"
#include "ev3dev_dev.h"

#ifndef CONFIG_DAVINCI_EHRPWM
#error "This requires the EHRPWM module to be enabled"
#endif


/*{{{  assorted device constants*/
#define DEVICE_NAME "ev3dev_pwm"

#define EV3DEV_PWM_OFFSET_VER_MAJOR	0x00
#define EV3DEV_PWM_OFFSET_VER_MINOR	0x01

#define EV3DEV_PWM_EHRPWM1A_NAME	"ehrpwm.1:0"
#define EV3DEV_PWM_EHRPWM1B_NAME	"ehrpwm.1:1"

#define   SOFT_TIMER_MS                 2
#define   SOFT_TIMER_SETUP              (SOFT_TIMER_MS * 1000000)

/*
 *  NO_OF_TACHO_SAMPLES holds the number of recent tacho samples
 */
#define   NO_OF_TACHO_SAMPLES           128
#define   NO_OF_OUTPUT_PORTS            4

#define   MAX_PWM_CNT                   (10000)
#define   MAX_SPEED                     (100)
#define   SPEED_PWMCNT_REL              (100)	//(MAX_PWM_CNT/MAX_SPEED)
#define   RAMP_FACTOR                   (1000)
#define   MAX_SYNC_MOTORS               (2)

//#define   COUNTS_PER_PULSE_LM           25600L
//#define   COUNTS_PER_PULSE_MM           16200L

#define   COUNTS_PER_PULSE_LM           12800L
#define   COUNTS_PER_PULSE_MM           8100L

enum {
	SAMPLES_BELOW_SPEED_25 = 0,
	SAMPLES_SPEED_25_50 = 1,
	SAMPLES_SPEED_50_75 = 2,
	SAMPLES_ABOVE_SPEED_75 = 3,
	NO_OF_SAMPLE_STEPS = 4
};

/*}}}*/

/*
 * Defines related to hardware PWM output
 */

/*{{{  TBCTL (Time-Base Control)*/

#define   TB_COUNT_UP                   0x0	// TBCNT MODE bits
#define   TB_COUNT_FREEZE               0x3	// TBCNT MODE bits
#define   TB_DISABLE                    0x0	// PHSEN bit
#define   TB_ENABLE                     0x4
#define   TB_SHADOW                     0x0	// PRDLD bit
#define   TB_IMMEDIATE                  0x8
#define   TB_SYNC_DISABLE               0x30	// SYNCOSEL bits
#define   TB_HDIV1                      0x0	// HSPCLKDIV bits
#define   TB_DIV1                       0x0	// CLKDIV bits
#define   TB_UP                         0x2000	// PHSDIR bit

/*}}}*/
/*{{{  CMPCTL (Compare Control)*/

#define   CC_CTR_A_ZERO                 0x0	// LOADAMODE bits
#define   CC_CTR_B_ZERO                 0x0
#define   CC_A_SHADOW                   0x00	// SHDWAMODE and SHDWBMODE bits
#define   CC_B_SHADOW                   0x00

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
/*{{{  SYS config register configuration and bits*/
enum {
	CFGCHIP1 = 0x60,
};

enum {
	TBCLKSYNC = 0x00001000,
};

/*}}}*/
/*{{{  TIMER64 register configuration*/
enum {
	REVID = 0,
	EMUMGT = 1,
	GPINTGPEN = 2,
	GPDATGPDIR = 3,
	TIM12 = 4,
	TIM34 = 5,
	PRD12 = 6,
	PRD34 = 7,
	TCR = 8,
	TGCR = 9,
	WDTCR = 10,
	NOTUSED1 = 11,
	NOTUSED2 = 12,
	REL12 = 13,
	REL34 = 14,
	CAP12 = 15,
	CAP34 = 16,
	NOTUSED3 = 17,
	NOTUSED4 = 18,
	NOTUSED5 = 19,
	NOTUSED6 = 20,
	NOTUSED7 = 21,
	NOTUSED8 = 22,
	INTCTLSTAT = 23,
	CMP0 = 24,
	CMP1 = 25,
	CMP2 = 26,
	CMP3 = 27,
	CMP4 = 28,
	CMP5 = 39,
	CMP6 = 30,
	CMP7 = 31,
};

/*}}}*/
/*{{{  eCAP Register configuration*/
enum {
	TSCTR = 0,
	CTRPHS = 2,
	CAP1 = 4,
	CAP2 = 6,
	CAP3 = 8,
	CAP4 = 10,
	ECCTL1 = 20,
	ECCTL2 = 21,
	ECEINT = 22,
	ECFLG = 23,
	ECCLR = 24,
	ECFRC = 25,
	ECAP_REVID = 46,
};

/*}}}*/

/*{{{  assorted motor constants*/
#define  NON_INV   1
#define  INV      -1

enum {
	FORWARD,
	BACKWARD,
	BRAKE,
	COAST,
};

enum {
	UNLIMITED_UNREG,
	UNLIMITED_REG,
	LIMITED_REG_STEPUP,
	LIMITED_REG_STEPCONST,
	LIMITED_REG_STEPDOWN,
	LIMITED_UNREG_STEPUP,
	LIMITED_UNREG_STEPCONST,
	LIMITED_UNREG_STEPDOWN,
	LIMITED_REG_TIMEUP,
	LIMITED_REG_TIMECONST,
	LIMITED_REG_TIMEDOWN,
	LIMITED_UNREG_TIMEUP,
	LIMITED_UNREG_TIMECONST,
	LIMITED_UNREG_TIMEDOWN,
	LIMITED_STEP_SYNC,
	LIMITED_TURN_SYNC,
	LIMITED_DIFF_TURN_SYNC,
	SYNCED_SLAVE,
	RAMP_DOWN_SYNC,
	HOLD,
	BRAKED,
	STOP_MOTOR,
	IDLE,
};

enum {
	UNUSED_SYNC_MOTOR = 0xFF
};

/*}}}*/
/*{{{  MOTOR struct: settings for a single motor*/
typedef struct {
	SLONG IrqTacho;
	SLONG TachoCnt;
	SLONG TachoSensor;
	SLONG TimeCnt;
	SLONG TimeInc;
	SLONG OldTachoCnt;
	SLONG TachoCntUp;
	SLONG TachoCntConst;
	SLONG TachoCntDown;
	SLONG RampUpFactor;
	SLONG RampUpOffset;
	SLONG RampDownFactor;
	SLONG PVal;
	SLONG IVal;
	SLONG DVal;
	SLONG OldSpeedErr;
	SLONG Power;
	SLONG TargetPower;
	SLONG Dir;
	SBYTE Speed;
	SBYTE TargetSpeed;
	SBYTE TargetBrake;
	UBYTE BrakeAfter;
	UBYTE LockRampDown;
	SBYTE Pol;
	UBYTE Direction;
	UBYTE Type;
	UBYTE State;
	UBYTE TargetState;
	UBYTE Owner;
	UBYTE Mutex;
	UBYTE DirChgPtr;
	SWORD TurnRatio;
} MOTOR;

/*}}}*/
/*{{{  TACHOSAMPLES struct: array of tacho values*/
typedef struct {
	ULONG TachoArray[NO_OF_TACHO_SAMPLES];
	UBYTE ArrayPtr;
	UBYTE ArrayPtrOld;
} TACHOSAMPLES;

/*}}}*/

/*{{{  forward declarations*/
static int ev3dev_pwm_doinit (void);
static void ev3dev_pwm_doexit (void);

static irqreturn_t IntA (int irq, void *dev);
static irqreturn_t IntB (int irq, void *dev);
static irqreturn_t IntC (int irq, void *dev);
static irqreturn_t IntD (int irq, void *dev);

UBYTE dCalculateSpeed (UBYTE No, SBYTE *pSpeed);
void SetGpioAnyEdgeIrq (UBYTE PinNo, irqreturn_t (*IntFuncPtr) (int, void *));
void GetSyncDurationCnt (SLONG *pCount0, SLONG *pCount1);
void CheckforEndOfSync (void);


void SetDutyMA (ULONG Duty);
void SetDutyMB (ULONG Duty);
void SetDutyMC (ULONG Duty);
void SetDutyMD (ULONG Duty);

/*}}}*/

/*{{{  eCAP register offsets.  XXX: these replace some earlier enums*/
#define CAP1					0x08
#define CAP2					0x0C
#define CAP3					0x10
#define CAP4					0x14
#define ECCTL1					0x28
#define ECCTL2					0x2A
#define ECEINT					0x2C
#define ECFLG					0x2E
#define ECCLR					0x30

/*}}}*/
/*{{{  Interrupt enable bits*/
#define CAP1_INT				0x2
#define CAP2_INT				0x4
#define CAP3_INT				0x8
#define CAP4_INT				0x10
#define OVF_INT					0x20
#define INT_FLAG_CLR				0x3F

/*}}}*/

/*{{{  enum OutputPortPins: output pins for motors */
enum OutputPortPins {
	PWM,
	DIR0,
	DIR1,
	INT,
	DIR,
	SLEEP,
	FAULT,
	OUTPUT_PORT_PINS
};

/*}}}*/
/*{{{  interrupt pins*/
#define   IRQA_PINNO                  (PWM_OUTPUTA_INT)
#define   IRQB_PINNO                  (PWM_OUTPUTB_INT)
#define   IRQC_PINNO                  (PWM_OUTPUTC_INT)
#define   IRQD_PINNO                  (PWM_OUTPUTD_INT)

/*}}}*/

/*{{{  output to GPIO mapping*/
#define PWM_OUTPUTA_PWM  	GPIO_TO_PIN(2, 14 )
#define PWM_OUTPUTA_DIR0  	GPIO_TO_PIN(3, 15 )
#define PWM_OUTPUTA_DIR1  	GPIO_TO_PIN(3,  6 )
#define PWM_OUTPUTA_INT  	GPIO_TO_PIN(5, 11 )
#define PWM_OUTPUTA_DIRA  	GPIO_TO_PIN(0,  4 )
#define PWM_OUTPUTA_SLEEPAB  	(-1)
#define PWM_OUTPUTA_FAULTAB  	(-1)

#define PWM_OUTPUTB_PWM  	GPIO_TO_PIN(2, 15 )
#define PWM_OUTPUTB_DIR0  	GPIO_TO_PIN(2,  1 )
#define PWM_OUTPUTB_DIR1  	GPIO_TO_PIN(0,  3 )
#define PWM_OUTPUTB_INT  	GPIO_TO_PIN(5,  8 )
#define PWM_OUTPUTB_DIRA  	GPIO_TO_PIN(2,  9 )
#define PWM_OUTPUTB_SLEEPAB  	(-1)
#define PWM_OUTPUTB_FAULTAB  	(-1)

#define PWM_OUTPUTC_PWM  	GPIO_TO_PIN(8,  7 )
#define PWM_OUTPUTC_DIR0  	GPIO_TO_PIN(6,  8 )
#define PWM_OUTPUTC_DIR1  	GPIO_TO_PIN(5,  9 )
#define PWM_OUTPUTC_INT  	GPIO_TO_PIN(5, 13 )
#define PWM_OUTPUTC_DIRA  	GPIO_TO_PIN(3, 14 )
#define PWM_OUTPUTC_SLEEPCD  	(-1)
#define PWM_OUTPUTC_FAULTCD  	(-1)

#define PWM_OUTPUTD_PWM  	GPIO_TO_PIN(0,  0 )
#define PWM_OUTPUTD_DIR0  	GPIO_TO_PIN(5,  3 )
#define PWM_OUTPUTD_DIR1  	GPIO_TO_PIN(5, 10 )
#define PWM_OUTPUTD_INT  	GPIO_TO_PIN(6,  9 )
#define PWM_OUTPUTD_DIRA  	GPIO_TO_PIN(2,  8 )
#define PWM_OUTPUTD_SLEEPCD  	(-1)
#define PWM_OUTPUTD_FAULTCD  	(-1)


#define EV3_OUTPUTA_PWM   EV3_EPWM1B
#define EV3_OUTPUTA_DIR0  EV3_GPIO3_15
#define EV3_OUTPUTA_DIR1  EV3_GPIO3_6
#define EV3_OUTPUTA_INT   EV3_GPIO5_11
#define EV3_OUTPUTA_DIRA  EV3_GPIO0_4

#define EV3_OUTPUTB_PWM   EV3_EPWM1A
#define EV3_OUTPUTB_DIR0  EV3_GPIO2_1
#define EV3_OUTPUTB_DIR1  EV3_GPIO0_3
#define EV3_OUTPUTB_INT   EV3_GPIO5_8
#define EV3_OUTPUTB_DIRA  EV3_GPIO2_9

#define EV3_OUTPUTC_PWM   EV3_APWM0
#define EV3_OUTPUTC_DIR0  EV3_GPIO6_8
#define EV3_OUTPUTC_DIR1  EV3_GPIO5_9
#define EV3_OUTPUTC_INT   EV3_GPIO5_13
#define EV3_OUTPUTC_DIRA  EV3_GPIO3_14

#define EV3_OUTPUTD_PWM   EV3_APWM1
#define EV3_OUTPUTD_DIR0  EV3_GPIO5_3
#define EV3_OUTPUTD_DIR1  EV3_GPIO5_10
#define EV3_OUTPUTD_INT   EV3_GPIO6_9
#define EV3_OUTPUTD_DIRA  EV3_GPIO2_8

static const int legoev3_pwm_gpio[][OUTPUT_PORT_PINS] = {
	{PWM_OUTPUTA_PWM	// Output port A
	 , PWM_OUTPUTA_DIR0, PWM_OUTPUTA_DIR1, PWM_OUTPUTA_INT, PWM_OUTPUTA_DIRA, PWM_OUTPUTA_SLEEPAB, PWM_OUTPUTA_FAULTAB}
	, {PWM_OUTPUTB_PWM	// Output port B
	   , PWM_OUTPUTB_DIR0, PWM_OUTPUTB_DIR1, PWM_OUTPUTB_INT, PWM_OUTPUTB_DIRA, PWM_OUTPUTB_SLEEPAB, PWM_OUTPUTB_FAULTAB}
	, {PWM_OUTPUTC_PWM	// Output port C
	   , PWM_OUTPUTC_DIR0, PWM_OUTPUTC_DIR1, PWM_OUTPUTC_INT, PWM_OUTPUTC_DIRA, PWM_OUTPUTC_SLEEPCD, PWM_OUTPUTC_FAULTCD}
	, {PWM_OUTPUTD_PWM	// Output port D
	   , PWM_OUTPUTD_DIR0, PWM_OUTPUTD_DIR1, PWM_OUTPUTD_INT, PWM_OUTPUTD_DIRA, PWM_OUTPUTD_SLEEPCD, PWM_OUTPUTD_FAULTCD}
};

static const short legoev3_outputA_pins[] = {
	EV3_OUTPUTA_PWM		// Output port A
		, EV3_OUTPUTA_DIR0, EV3_OUTPUTA_DIR1, EV3_OUTPUTA_INT, EV3_OUTPUTA_DIRA, (-1)
};

static const short legoev3_outputB_pins[] = {
	EV3_OUTPUTB_PWM		// Output port B
		, EV3_OUTPUTB_DIR0, EV3_OUTPUTB_DIR1, EV3_OUTPUTB_INT, EV3_OUTPUTB_DIRA, (-1)
};

static const short legoev3_outputC_pins[] = {
	EV3_OUTPUTC_PWM		// Output port C
		, EV3_OUTPUTC_DIR0, EV3_OUTPUTC_DIR1, EV3_OUTPUTC_INT, EV3_OUTPUTC_DIRA, (-1)
};

static const short legoev3_outputD_pins[] = {
	EV3_OUTPUTD_PWM		// Output port D
		, EV3_OUTPUTD_DIR0, EV3_OUTPUTD_DIR1, EV3_OUTPUTD_INT, EV3_OUTPUTD_DIRA, (-1)
};

static const short *legoev3_output_pins[] = {
	legoev3_outputA_pins, legoev3_outputB_pins, legoev3_outputC_pins, legoev3_outputD_pins
};

#warning "Remember to check if this direction pin is correct!"
// #define   OutputReadDir(port,pin)     ((gpio_get_value( legoev3_pwm_gpio[port][pin] )))
#define   OutputReadDir(port,pin)     (!(gpio_get_value( legoev3_pwm_gpio[port][pin] )))

#define   READDirA                      OutputReadDir(0,DIR)
#define   READDirB                      OutputReadDir(1,DIR)
#define   READDirC                      OutputReadDir(2,DIR)
#define   READDirD                      OutputReadDir(3,DIR)


/*}}}*/

/*
 * Variables
 */

/*{{{  global vars*/
static void **ev3mem = NULL;

#if 0
static struct pwm_device *ehrpwm_1_0 = NULL;		/* eHRPWM1A for motor A */
static struct pwm_device *ehrpwm_1_1 = NULL;		/* eHRPWM1B for motor B */
#endif

static MOTOR Motor[NO_OF_OUTPUT_PORTS];
static SLONG *(StepPowerSteps[NO_OF_OUTPUT_PORTS]);
static SLONG *(TachoSteps[NO_OF_OUTPUT_PORTS]) = {
&(Motor[0].TachoCnt), &(Motor[1].TachoCnt), &(Motor[2].TachoCnt), &(Motor[3].TachoCnt)};

static SLONG *(TimerSteps[NO_OF_OUTPUT_PORTS]) = {
&(Motor[0].TimeCnt), &(Motor[1].TimeCnt), &(Motor[2].TimeCnt), &(Motor[3].TimeCnt)};

static void (*SetDuty[NO_OF_OUTPUT_PORTS]) (ULONG) = {
SetDutyMA, SetDutyMB, SetDutyMC, SetDutyMD};

static TACHOSAMPLES TachoSamples[NO_OF_OUTPUT_PORTS];
static UBYTE ReadyStatus = 0;
static UBYTE TestStatus = 0;

static MOTORDATA MotorData[NO_OF_OUTPUT_PORTS];
static MOTORDATA *pMotor = MotorData;

static ULONG TimeOutSpeed0[NO_OF_OUTPUT_PORTS];
static UBYTE MinRegEnabled[NO_OF_OUTPUT_PORTS];

static UBYTE SyncMNos[MAX_SYNC_MOTORS];
static SBYTE MaxSyncSpeed;

static struct hrtimer Device1Timer;
static ktime_t Device1Time;

static UBYTE PrgStopTimer[NO_OF_OUTPUT_PORTS];


static ULONG CountsPerPulse[4] = { COUNTS_PER_PULSE_LM, COUNTS_PER_PULSE_LM, COUNTS_PER_PULSE_LM, COUNTS_PER_PULSE_LM };

/*}}}*/
/*{{{  docs*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 */
/*! \brief    Number of average samples for each motor
 *
 *  - Medium motor = 1, 2,  4,  8
 *  - Large motor  = 2, 8, 16, 32
 *
 *  Medium motor has not the jitter on the tacho when as large motor because
 *  it has one gear wheel less that the large motor.
 *
 *  Medium motor reaction time is much faster than large motor due to smaller motor
 *  and smaller gearbox
 */


/*}}}*/
/*{{{  global vars*/
static UBYTE SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = { 2, 4, 8, 16 };
static UBYTE SamplesLargeMotor[NO_OF_SAMPLE_STEPS] = { 4, 16, 32, 64 };

//static    UBYTE    SamplesMediumMotor[NO_OF_SAMPLE_STEPS] = {1, 2,  4,  8};
//static    UBYTE    SamplesLargeMotor[NO_OF_SAMPLE_STEPS]  = {2, 8, 16, 32};
static UBYTE *SamplesPerSpeed[NO_OF_OUTPUT_PORTS] = { SamplesLargeMotor, SamplesLargeMotor, SamplesLargeMotor, SamplesLargeMotor };

static UBYTE AVG_TACHO_COUNTS[NO_OF_OUTPUT_PORTS] = { 2, 2, 2, 2 };
static ULONG AVG_COUNTS[NO_OF_OUTPUT_PORTS] = { (2 * COUNTS_PER_PULSE_LM), (2 * COUNTS_PER_PULSE_LM), (2 * COUNTS_PER_PULSE_LM), (2 * COUNTS_PER_PULSE_LM) };

/*}}}*/

/*
 * Macros
 */

/*{{{  FLOATFaultPins ()*/
#define   FLOATFaultPins                {\
                                          if ( legoev3_pwm_gpio[0][FAULT] != -1)\
                                          {\
                                            (gpio_direction_input( legoev3_pwm_gpio[0][FAULT] ));\
                                          }\
                                          if ( legoev3_pwm_gpio[2][FAULT] != -1)\
                                          {\
                                            (gpio_direction_input( legoev3_pwm_gpio[2][FAULT] ));\
                                          }\
                                        }
/*}}}*/
/*{{{  SetSleepPins ()*/
#define   SETSleepPins                  {\
                                          if ( legoev3_pwm_gpio[0][SLEEP] != -1)\
                                          {\
                                            (gpio_direction_output( legoev3_pwm_gpio[0][SLEEP], 1 ));\
                                          }\
                                          if ( legoev3_pwm_gpio[2][SLEEP] != -1)\
                                          {\
                                            (gpio_direction_output( legoev3_pwm_gpio[2][SLEEP], 1 ));\
                                          }\
                                        }
/*}}}*/
/*{{{  SETMotorType (Port, NewType)*/
#define   SETMotorType(Port, NewType)   {\
                                          Motor[Port].Type = NewType;\
                                          if (TYPE_MINITACHO == NewType)\
                                          {\
                                            CountsPerPulse[Port]      =  COUNTS_PER_PULSE_MM;\
                                            SamplesPerSpeed[Port]     = SamplesMediumMotor;\
                                          }\
                                          else\
                                          {\
                                            CountsPerPulse[Port]      =  COUNTS_PER_PULSE_LM;\
                                            SamplesPerSpeed[Port]     =  SamplesLargeMotor;\
                                          }\
                                        }
/*}}}*/
/*{{{  SETAvgTachoCount (Port, Speed)*/
#define   SETAvgTachoCount(Port, Speed) {\
                                          if (Speed > 80)\
                                          {\
                                            AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][3];\
                                            AVG_COUNTS[Port]       = SamplesPerSpeed[Port][3] * CountsPerPulse[Port];\
                                          }\
                                          else\
                                          {\
                                            if (Speed > 60)\
                                            {\
                                              AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][2];\
                                              AVG_COUNTS[Port]       = SamplesPerSpeed[Port][2] * CountsPerPulse[Port];\
                                            }\
                                            else\
                                            {\
                                              if (Speed > 40)\
                                              {\
                                                AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][1];\
                                                AVG_COUNTS[Port]       = SamplesPerSpeed[Port][1] * CountsPerPulse[Port];\
                                              }\
                                              else\
                                              {\
                                                AVG_TACHO_COUNTS[Port] = SamplesPerSpeed[Port][0];\
                                                AVG_COUNTS[Port]       = SamplesPerSpeed[Port][0] * CountsPerPulse[Port];\
                                              }\
                                            }\
                                          }\
                                        }

/*}}}*/
/*{{{  FREERunning24bittimer ()*/
#define   FREERunning24bittimer         (EV3IO_READ32(ev3mem[EV3IO_TIMER64P3], TIM34) >> 8)

/*}}}*/
/*{{{  CLEARTachoArray (No)*/
#define   CLEARTachoArray(No)           {\
                                          Motor[No].DirChgPtr = 0;\
                                          memset(&TachoSamples[No], 0, sizeof(TACHOSAMPLES));\
                                        }
/*}}}*/
/*{{{  OutputFloat (port, pin)*/
#define   OutputFloat(port,pin)         {\
	                                  gpio_direction_output(legoev3_pwm_gpio[port][pin], 0);\
	                                  gpio_direction_input(legoev3_pwm_gpio[port][pin]);\
                                        }
/*}}}*/
/*{{{  OutputRead (port, pin)*/
#define   OutputRead(port,pin)          (gpio_get_value( legoev3_pwm_gpio[port][pin] ))
/*}}}*/
/*{{{  OutputHigh (port, pin)*/
#define   OutputHigh(port,pin)          {\
	                                  gpio_direction_output(legoev3_pwm_gpio[port][pin], 1);\
                                        }
/*}}}*/
/*{{{  OutputLow (port, pin)*/
#define   OutputLow(port,pin)           {\
	                                  gpio_direction_output(legoev3_pwm_gpio[port][pin], 0);\
                                        }
/*}}}*/
/*{{{  EHRPWMClkDis () -- PWM clock disable*/
#define   EHRPWMClkDis                  {\
					  EV3IO_WRITE16((TB_UP | TB_DISABLE | TB_SHADOW | TB_SYNC_DISABLE | TB_HDIV1 | TB_DIV1 | TB_COUNT_FREEZE), \
                                          	ev3mem[EV3IO_EHRPWM1], TBCTL); \
                                        }
/*}}}*/
/*{{{  EHRPWMClkEna () -- PWM clock enable*/
#define   EHRPWMClkEna                  {\
                                          EV3IO_WRITE16((TB_UP | TB_DISABLE | TB_SHADOW | TB_SYNC_DISABLE | TB_HDIV1 | TB_DIV1 | TB_COUNT_UP), \
					  	ev3mem[EV3IO_EHRPWM1], TBCTL); \
                                          EV3IO_WRITE32(EV3IO_READ32 (ev3mem[EV3IO_SYSCFG0], CFGCHIP1) | TBCLKSYNC, ev3mem[EV3IO_SYSCFG0], CFGCHIP1); /*This is the clock to all eHRPWM's*/\
                                        }
/*}}}*/
/*{{{  OLDCODE*/
//                                        REGUnlock;
//                                        iowrite32((ioread32(&SYSCFG0[CFGCHIP1]) | TBCLKSYNC),&SYSCFG0[CFGCHIP1]); /*This is the clock to all eHRPWM's*/
//                                        REGLock;
//                                      }
/*}}}*/
/*{{{  STOPPwm ()*/
#define   STOPPwm                       {\
                                             EV3IO_WRITE16(0x00, ev3mem[EV3IO_ECAP0], 0x15);\
                                             EV3IO_WRITE16(0x00, ev3mem[EV3IO_ECAP1], 0x15);\
                                             EV3IO_WRITE32(0x00000000, ev3mem[EV3IO_TIMER64P3], TGCR);\
                                             EV3IO_WRITE16(0x00, ev3mem[EV3IO_EHRPWM1], TBCTL);\
                                             EV3IO_WRITE16(0x00, ev3mem[EV3IO_EHRPWM1], CMPCTL);\
                                             EHRPWMClkDis;\
                                        }
/*}}}*/
/*{{{  SETPwmFreqKHz (KHz)*/
#define   SETPwmFreqKHz(KHz)            {\
                                          EV3IO_WRITE16(KHz, ev3mem[EV3IO_EHRPWM1], TBPRD); /* For Motor A and Motor B */\
                                          EV3IO_WRITE16(KHz, ev3mem[EV3IO_ECAP0], CAP1); /* For Motor C             */\
                                          EV3IO_WRITE16(KHz, ev3mem[EV3IO_ECAP1], CAP1); /* For Motor D             */\
                                        }
/*}}}*/
/*{{{  SETUPPwmModules ()*/
#define   SETUPPwmModules               { \
                                             /* eHRPWM Module */\
                                             EHRPWMClkDis;\
                                             EV3IO_WRITE16 (0x0000, ev3mem[EV3IO_EHRPWM1], TBPHS);\
                                             EV3IO_WRITE16 (0x0000, ev3mem[EV3IO_EHRPWM1], TBCNT);\
                                             EV3IO_WRITE16 ((CC_A_SHADOW | CC_B_SHADOW | CC_CTR_A_ZERO | CC_CTR_B_ZERO), \
					     		ev3mem[EV3IO_EHRPWM1], CMPCTL);\
                                             EV3IO_WRITE16 (0x0021, ev3mem[EV3IO_EHRPWM1], AQCTLA);\
                                             EV3IO_WRITE16 (0x0201, ev3mem[EV3IO_EHRPWM1], AQCTLB);\
                                             EHRPWMClkEna;\
                                             \
                                             /* eCAP modules - APWM */\
                                             EV3IO_WRITE16 (0, ev3mem[EV3IO_ECAP0], TSCTR);\
                                             EV3IO_WRITE16 (0, ev3mem[EV3IO_ECAP1], TSCTR);\
                                             EV3IO_WRITE16 (0, ev3mem[EV3IO_ECAP0], CTRPHS);\
                                             EV3IO_WRITE16 (0, ev3mem[EV3IO_ECAP1], CTRPHS);\
                                             EV3IO_WRITE16 (0x0690, ev3mem[EV3IO_ECAP0], ECCTL2);\
                                             EV3IO_WRITE16 (0x0690, ev3mem[EV3IO_ECAP1], ECCTL2);\
					     \
                                             EV3IO_WRITE32 (0x00003304, ev3mem[EV3IO_TIMER64P3], TGCR);\
                                             EV3IO_WRITE32 (EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], TGCR) | 0x00000002, \
					     		ev3mem[EV3IO_TIMER64P3], TGCR);\
                                             EV3IO_WRITE32 (0xFFFFFFFF, ev3mem[EV3IO_TIMER64P3], PRD34);\
                                             EV3IO_WRITE32 (0x00800000, ev3mem[EV3IO_TIMER64P3], TCR);\
                                                                              \
                                             /* Setup PWM */\
                                             SetDutyMA(0);\
                                             SetDutyMB(0);\
                                             SetDutyMC(0);\
                                             SetDutyMD(0);\
                                             SETPwmFreqKHz(MAX_PWM_CNT-1);\
                                             FLOATFaultPins;\
                                             SETSleepPins;\
                                      }
/*}}}*/
/*{{{  OLDCODE*/
//                                           /* Setup PWM */
//                                           SetDutyMA(0);
//                                           SetDutyMB(0);
//                                           SetDutyMC(0);
//                                           SetDutyMD(0);
//                                           SETPwmFreqKHz(MAX_PWM_CNT-1);
//                                           FLOATFaultPins;
//                                           SETSleepPins;
//                                      }
/*}}}*/

/*{{{  motor flag reading*/
#define   READIntA                      OutputRead(0,INT)
#define   READIntB                      OutputRead(1,INT)
#define   READIntC                      OutputRead(2,INT)
#define   READIntD                      OutputRead(3,INT)

/*}}}*/

/*
 * Functions
 */

/*{{{  void CheckSpeedPowerLimits (SBYTE *pCheckVal)*/
/*
 *	checks that *pCheckVal is within specified speed limits [-MAX_SPEED .. MAX_SPEED].
 */
void CheckSpeedPowerLimits (SBYTE *pCheckVal)
{
	if (MAX_SPEED < *pCheckVal) {
		*pCheckVal = MAX_SPEED;
	}
	if (-MAX_SPEED > *pCheckVal) {
		*pCheckVal = -MAX_SPEED;
	}
}
/*}}}*/
/*{{{  UBYTE CheckLessThanSpecial (SLONG Param1, SLONG Param2, SLONG Inv)*/
/*
 *	test.
 */
UBYTE CheckLessThanSpecial (SLONG Param1, SLONG Param2, SLONG Inv)
{
	UBYTE RtnVal = FALSE;

	if ((Param1 * Inv) < (Param2 * Inv)) {
		RtnVal = TRUE;
	}
	return (RtnVal);
}
/*}}}*/
/*{{{  void IncSpeed (SBYTE Dir, SBYTE *pSpeed)*/
/*
 *	increment speed.
 */
void IncSpeed (SBYTE Dir, SBYTE *pSpeed)
{
	if ((NON_INV == Dir) && (MAX_SPEED > *pSpeed)) {
		(*pSpeed)++;
	} else {
		if ((INV == Dir) && (-MAX_SPEED < *pSpeed)) {
			(*pSpeed)--;
		}
	}
}
/*}}}*/
/*{{{  void DecSpeed (SBYTE Dir, SBYTE *pSpeed)*/
/*
 *	decrement speed.
 */
void DecSpeed (SBYTE Dir, SBYTE *pSpeed)
{
	if ((NON_INV == Dir) && (0 < *pSpeed)) {
		(*pSpeed)--;
	} else {
		if ((INV == Dir) && (0 > *pSpeed)) {
			(*pSpeed)++;
		}
	}
}
/*}}}*/
/*{{{  void SetDirRwd (UBYTE Port)*/
/*
 *	set reverse direction.
 */
void SetDirRwd (UBYTE Port)
{
	OutputFloat (Port, DIR0);
	OutputHigh (Port, DIR1);
}
/*}}}*/
/*{{{  void SetDirFwd (UBYTE Port)*/
/*
 *	set forward direction.
 */
void SetDirFwd (UBYTE Port)
{
	OutputHigh (Port, DIR0);
	OutputFloat (Port, DIR1);
}
/*}}}*/
/*{{{  void SetBrake (UBYTE Port)*/
/*
 *	set motor brake.
 */
void SetBrake (UBYTE Port)
{
	OutputHigh (Port, DIR0);
	OutputHigh (Port, DIR1);
}
/*}}}*/
/*{{{  void SetCoast (UBYTE Port)*/
/*
 *	set motor float (coast).
 */
void SetCoast (UBYTE Port)
{
	OutputLow (Port, DIR0);
	OutputLow (Port, DIR1);
}
/*}}}*/
/*{{{  UWORD GetTachoDir (UBYTE Port)*/
/*
 *	gets the tachometer direction.
 */
UWORD GetTachoDir (UBYTE Port)
{
	return (OutputRead (Port, DIR));
}
/*}}}*/
/*{{{  UWORD GetTachoInt (UBYTE Port)*/
/*
 *	gets the tachometer value.
 */
UWORD GetTachoInt (UBYTE Port)
{
	return (OutputRead (Port, INT));
}
/*}}}*/
/*{{{  void SetPower (UBYTE Port, SLONG Power)*/
/*
 *	set motor power.
 */
void SetPower (UBYTE Port, SLONG Power)
{
	if (MAX_PWM_CNT < Power) {
		Power = MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}
	if (-MAX_PWM_CNT > Power) {
		Power = -MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}

	if (TYPE_MINITACHO == Motor[Port].Type) {
		// Medium Motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd (Port);
			} else {
				SetDirRwd (Port);
				Power = 0 - Power;
			}
			Power = ((Power * 8000) / 10000) + 2000;
		}
	} else {
		// Large motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd (Port);
			} else {
				SetDirRwd (Port);
				Power = 0 - Power;
			}
			Power = ((Power * 9500) / 10000) + 500;
		}
	}
	SetDuty[Port] (Power);
}
/*}}}*/
/*{{{  void SetRegulationPower (UBYTE Port, SLONG Power)*/
/*
 *	sets motor power (bit different to the above in duty-cycle calculation).
 */
void SetRegulationPower (UBYTE Port, SLONG Power)
{
	if (MAX_PWM_CNT < Power) {
		Power = MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}
	if (-MAX_PWM_CNT > Power) {
		Power = -MAX_PWM_CNT;
		Motor[Port].Power = Power;
	}

	if (TYPE_MINITACHO == Motor[Port].Type) {
		// Medium Motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd (Port);
			} else {
				SetDirRwd (Port);
				Power = 0 - Power;
			}
			Power = ((Power * 9000) / 10000) + 1000;
		}
	} else {
		// Large motor
		if (0 != Power) {
			if (0 < Power) {
				SetDirFwd (Port);
			} else {
				SetDirRwd (Port);
				Power = 0 - Power;
			}
			Power = ((Power * 10000) / 10000) + 0;
		}
	}
	SetDuty[Port] (Power);
}
/*}}}*/
/*{{{  void SetDutyMA (ULONG Duty)*/
/*
 *	sets motor A duty cycle
 */
void SetDutyMA (ULONG Duty)
{
	EV3IO_WRITE16 ((UWORD)Duty, ev3mem[EV3IO_EHRPWM1], CMPA);
}
/*}}}*/
/*{{{  void SetDutyMB (ULONG Duty)*/
/*
 *	sets motor B duty cycle
 */
void SetDutyMB (ULONG Duty)
{
	EV3IO_WRITE16 ((UWORD)Duty, ev3mem[EV3IO_EHRPWM1], CMPB);
}
/*}}}*/
/*{{{  void SetDutyMC (ULONG Duty)*/
/*
 *	sets motor C duty cycle
 */
void SetDutyMC (ULONG Duty)
{
	EV3IO_WRITE16 ((UWORD)Duty, ev3mem[EV3IO_ECAP1], CAP2);
}
/*}}}*/
/*{{{  void SetDutyMD (ULONG Duty)*/
/*
 *	sets motor D duty cycle.
 */
void SetDutyMD (ULONG Duty)
{
	EV3IO_WRITE16 ((UWORD)Duty, ev3mem[EV3IO_ECAP0], CAP2);
}
/*}}}*/

/*{{{  void ClearPIDParameters (UBYTE No)*/
/*
 *	clears PID parameters.
 */
void ClearPIDParameters (UBYTE No)
{
	Motor[No].PVal = 0;
	Motor[No].IVal = 0;
	Motor[No].DVal = 0;
	Motor[No].OldSpeedErr = 0;
}
/*}}}*/
/*{{{  void StepPowerStopMotor (UBYTE No, SLONG AdjustTachoValue)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepPowerStopMotor
 *
 *  Helper function
 *  Checks how the motor should stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 *
 */
void StepPowerStopMotor (UBYTE No, SLONG AdjustTachoValue)
{
	*StepPowerSteps[No] -= AdjustTachoValue;
	SetPower (No, 0);	// Don't destroy power level if next command needs to use it
	if (Motor[No].TargetBrake) {
		if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
			// To be able to brake at tacho = 0 if this was timed command
			Motor[No].TachoCnt = 0;
		}
		Motor[No].State = BRAKED;
	} else {
		Motor[No].State = IDLE;
		SetCoast (No);
	}
	Motor[No].TargetState = UNLIMITED_UNREG;	// Default motor startup

	// If this was a synced motor then clear it
	if (SyncMNos[0] == No) {
		SyncMNos[0] = UNUSED_SYNC_MOTOR;
	}
	if (SyncMNos[1] == No) {
		SyncMNos[1] = UNUSED_SYNC_MOTOR;
	}
	ReadyStatus &= ~(0x01 << No);	// Clear ready flag
	TestStatus &= ~(0x01 << No);	// Clear ready flag
}

/*}}}*/
/*{{{  UBYTE StepSpeedCheckTachoCntUp (UBYTE No)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepSpeedCheckTachoCntUp
 *
 *  Helper function
 *  Checks if step power should have a Count up sequence
 *
 *  Parameters:
 *  No: The motor number
 *
 */
UBYTE StepSpeedCheckTachoCntUp (UBYTE No)
{
	UBYTE Return;
	SLONG CorrPower;

	Return = FALSE;
	if (Motor[No].TachoCntUp) {	// RampUp enabled

		Motor[No].State = LIMITED_REG_STEPUP;

		if (0 != Motor[No].Speed) {
			CorrPower = Motor[No].TargetPower - (SLONG) (Motor[No].Speed);
		} else {
			CorrPower = Motor[No].TargetPower;
		}

		// Number of powersteps per tacho count
		Motor[No].RampUpFactor = ((CorrPower * RAMP_FACTOR) + (Motor[No].TachoCntUp / 2)) / Motor[No].TachoCntUp;

		if (0 == Motor[No].RampUpFactor) {
			Motor[No].RampUpFactor = 1;
		}
		Motor[No].RampUpOffset = Motor[No].Speed;

		if (0 == Motor[No].Speed) {
			ClearPIDParameters (No);
		}

		if ((0 == Motor[No].TachoCntConst) && (0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		Return = TRUE;
	}
	return (Return);
}

/*}}}*/
/*{{{  UBYTE StepSpeedCheckTachoCntConst (UBYTE No, SLONG AdjustTachoValue)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepPowerCheckTachoCntConst
 *
 *  Helper function
 *  Checks if step power should have a constant power sequence
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 *
 */
UBYTE StepSpeedCheckTachoCntConst (UBYTE No, SLONG AdjustTachoValue)
{
	UBYTE ReturnState;

	ReturnState = FALSE;

	if (Motor[No].TachoCntConst) {
		*StepPowerSteps[No] -= AdjustTachoValue;
		Motor[No].TargetSpeed = Motor[No].TargetPower;	//(Motor[No].TargetPower / SPEED_PWMCNT_REL);
		Motor[No].State = LIMITED_REG_STEPCONST;
		ClearPIDParameters (No);

		if ((0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		ReturnState = TRUE;
	}
	return (ReturnState);
}

/*}}}*/
/*{{{  void StepSpeedCheckTachoCntDown (UBYTE No, SLONG AdjustTachoValue)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepSpeedCheckTachoCntDown
 *
 *  Helper function
 *  Checks if step power should ramp down or stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 *
 */
void StepSpeedCheckTachoCntDown (UBYTE No, SLONG AdjustTachoValue)
{
	if (Motor[No].TachoCntDown) {
		Motor[No].RampDownFactor = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntDown / 2)) / Motor[No].TachoCntDown;

		*StepPowerSteps[No] -= AdjustTachoValue;
		Motor[No].State = LIMITED_REG_STEPDOWN;
		MinRegEnabled[No] = FALSE;
		ClearPIDParameters (No);
	} else {
		StepPowerStopMotor (No, AdjustTachoValue);
	}
}

/*}}}*/
/*{{{  UBYTE StepPowerCheckTachoCntUp (UBYTE No)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepPowerCheckTachoCntUp
 *
 *  Helper function
 *  Checks if step power should have a Count up sequence
 *
 *  Parameters:
 *  No: The motor number
 *
 */
UBYTE StepPowerCheckTachoCntUp (UBYTE No)
{
	UBYTE Return;

	Return = FALSE;

	if (Motor[No].TachoCntUp) {	// RampUp enabled

		Motor[No].State = LIMITED_UNREG_STEPUP;
		Motor[No].RampUpFactor = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntUp / 2)) / Motor[No].TachoCntUp;
		ClearPIDParameters (No);

		if ((0 == Motor[No].TachoCntConst) && (0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		Return = TRUE;
	}
	return (Return);
}

/*}}}*/
/*{{{  UBYTE StepPowerCheckTachoCntConst (UBYTE No, SLONG AdjustTachoValue)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepPowerCheckTachoCntConst
 *
 *  Helper function
 *  Checks if step power should have a constant power sequense
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 *
 */
UBYTE StepPowerCheckTachoCntConst (UBYTE No, SLONG AdjustTachoValue)
{
	UBYTE ReturnState;

	ReturnState = FALSE;

	if (Motor[No].TachoCntConst) {
		*StepPowerSteps[No] -= AdjustTachoValue;
		Motor[No].Power = Motor[No].TargetPower;
		Motor[No].State = LIMITED_UNREG_STEPCONST;
		SetPower (No, Motor[No].Power);
		ClearPIDParameters (No);

		if ((0 == Motor[No].TachoCntDown) && (1 == Motor[No].TargetBrake)) {
			Motor[No].BrakeAfter = TRUE;
		} else {
			Motor[No].BrakeAfter = FALSE;
		}
		Motor[No].LockRampDown = FALSE;
		ReturnState = TRUE;
	}
	return (ReturnState);
}
/*}}}*/
/*{{{  void StepPowerCheckTachoCntDown (UBYTE No, SLONG AdjustTachoValue)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    StepPowerCheckTachoCntDown
 *
 *  Helper function
 *  Checks if step power should ramp down or stop
 *
 *  Parameters:
 *  No: The motor number
 *  AdjustTachoValue: The value that the tacho should be decremented
 *
 */
void StepPowerCheckTachoCntDown (UBYTE No, SLONG AdjustTachoValue)
{
	if (Motor[No].TachoCntDown) {

		Motor[No].RampDownFactor = ((Motor[No].TargetPower * RAMP_FACTOR) + (Motor[No].TachoCntDown / 2)) / Motor[No].TachoCntDown;
		*StepPowerSteps[No] -= AdjustTachoValue;
		Motor[No].State = LIMITED_UNREG_STEPDOWN;
		MinRegEnabled[No] = FALSE;
		ClearPIDParameters (No);
	} else {
		StepPowerStopMotor (No, AdjustTachoValue);
	}
}
/*}}}*/
/*{{{  void dRegulateSpeed (UBYTE No)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 */
/*! \brief    dRegulateSpeed
 *
 *  - Calculates the new motor power value (duty cycle)
 *
 *  - Regulation method:
 *    To be defined
 *
 *  - Parameters:
 *    - Input:
 *      - No:       Motor output number
 *
 *    -Output:
 *      - None
 */
void dRegulateSpeed (UBYTE No)
{

	SLONG SpeedErr;

	if (TYPE_MINITACHO == Motor[No].Type) {
		SpeedErr = ((SLONG) (Motor[No].TargetSpeed) - (SLONG) (Motor[No].Speed));
		Motor[No].PVal = SpeedErr * 4;
		Motor[No].IVal = ((Motor[No].IVal * 9) / 10) + (SpeedErr / 3);
		Motor[No].DVal = (((SpeedErr - (Motor[No].OldSpeedErr)) * 4) / 2) * 40;
	} else {
		SpeedErr = ((SLONG) (Motor[No].TargetSpeed) - (SLONG) (Motor[No].Speed));
		Motor[No].PVal = SpeedErr * 2;
		Motor[No].IVal = ((Motor[No].IVal * 9) / 10) + (SpeedErr / 4);
		Motor[No].DVal = (((SpeedErr - (Motor[No].OldSpeedErr)) * 4) / 2) * 40;
	}

	Motor[No].Power = Motor[No].Power + ((Motor[No].PVal + Motor[No].IVal + Motor[No].DVal));

	if (Motor[No].Power > MAX_PWM_CNT) {
		Motor[No].Power = MAX_PWM_CNT;
	}

	if (Motor[No].Power < -MAX_PWM_CNT) {
		Motor[No].Power = -MAX_PWM_CNT;
	}
	Motor[No].OldSpeedErr = SpeedErr;


	if (((Motor[No].TargetSpeed) > 0) && (Motor[No].Power < 0)) {
		Motor[No].Power = 0;
	}

	if (((Motor[No].TargetSpeed) < 0) && (Motor[No].Power > 0)) {
		Motor[No].Power = 0;
	}

	SetRegulationPower (No, Motor[No].Power);
}
/*}}}*/
/*{{{  void BrakeMotor (UBYTE No, SLONG TachoCnt)*/
/*
 *	brakes the specified motor.
 */
void BrakeMotor (UBYTE No, SLONG TachoCnt)
{
	SLONG TmpTacho;

	TmpTacho = TachoCnt * 100;

	TmpTacho <<= 2;

	if (TmpTacho > MAX_PWM_CNT) {
		TmpTacho = MAX_PWM_CNT;
	}
	if (TmpTacho < -MAX_PWM_CNT) {
		TmpTacho = -MAX_PWM_CNT;
	}

	Motor[No].Power = 0 - TmpTacho;
	SetRegulationPower (No, Motor[No].Power);
}

/*}}}*/
/*{{{  UBYTE RampDownToBrake (UBYTE No, SLONG CurrentCnt, SLONG TargetCnt, SLONG Dir)*/
/*
 *	ramp down speed and brake when done.
 */
UBYTE RampDownToBrake (UBYTE No, SLONG CurrentCnt, SLONG TargetCnt, SLONG Dir)
{
	UBYTE Status;

	Status = FALSE;

	if (TRUE == CheckLessThanSpecial (CurrentCnt, TargetCnt, Dir)) {

//    Motor[No].TargetSpeed = (SBYTE)((Motor[No].TachoCntConst - CurrentCnt));
		Motor[No].TargetSpeed = (SBYTE) (TargetCnt - CurrentCnt);

		if ((Motor[No].TargetSpeed > 5) || (Motor[No].TargetSpeed < -5)) {
			if (TRUE == CheckLessThanSpecial (Motor[No].TargetSpeed, Motor[No].Speed, Dir)) {
				Motor[No].TargetSpeed = 1 * Dir;
			}
		}

		dRegulateSpeed (No);
	} else {
		// Done ramping down to brake
		Status = TRUE;
	}
	return (Status);
}

/*}}}*/
/*{{{  void CalcRampDownFactor (UBYTE No, ULONG CurrentPower, ULONG Counts)*/
/*
 *	calculates ramp-down factor.
 */
void CalcRampDownFactor (UBYTE No, ULONG CurrentPower, ULONG Counts)
{

	if (Counts != 0) {
		Motor[No].RampDownFactor = ((CurrentPower * RAMP_FACTOR) + (Counts / 2)) / Counts;
	}
}

/*}}}*/
/*{{{  void GetCompareCounts (UBYTE No, SLONG *Counts, SLONG *CompareCounts)*/
/*
 *	gets counters for comparison.
 */
void GetCompareCounts (UBYTE No, SLONG *Counts, SLONG *CompareCounts)
{
	*Counts = *StepPowerSteps[No];

	if (TRUE == Motor[No].BrakeAfter) {
		if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
			// Run for time
			*CompareCounts = *Counts + (Motor[No].Speed);
		} else {
			// Run for tacho
			if (TYPE_TACHO == Motor[No].Type) {
				*CompareCounts = *Counts + ((Motor[No].Speed) / 2);
			} else {
				*CompareCounts = *Counts + ((Motor[No].Speed / 3));
			}
		}
	} else {
		*CompareCounts = *Counts;
	}
}

/*}}}*/
/*{{{  void StopAndBrakeMotor (UBYTE MotorNo)*/
/*
 *	stop and brake motor.
 */
void StopAndBrakeMotor (UBYTE MotorNo)
{
	ReadyStatus &= ~(0x01 << MotorNo);
	TestStatus &= ~(0x01 << MotorNo);
	Motor[MotorNo].Power = 0;
	Motor[MotorNo].Speed = 0;
	Motor[MotorNo].TachoCnt = 0;
	Motor[MotorNo].State = BRAKED;
	CLEARTachoArray (MotorNo);
	SetPower (MotorNo, Motor[MotorNo].Power);
	SetBrake (MotorNo);
}

/*}}}*/
/*{{{  void StopAndFloatMotor (UBYTE MotorNo)*/
/*
 *	stop and float motor.
 */
void StopAndFloatMotor (UBYTE MotorNo)
{
	ReadyStatus &= ~(0x01 << MotorNo);
	TestStatus &= ~(0x01 << MotorNo);
	Motor[MotorNo].Power = 0;
	Motor[MotorNo].Speed = 0;
	Motor[MotorNo].TachoCnt = 0;
	Motor[MotorNo].State = IDLE;
	CLEARTachoArray (MotorNo);
	SetPower (MotorNo, Motor[MotorNo].Power);
	SetCoast (MotorNo);
}

/*}}}*/
/*{{{  void FloatSyncedMotors (void)*/
/*
 *	float synchronised motors.
 */
void FloatSyncedMotors (void)
{
	UBYTE TmpNo0, TmpNo1;

	TmpNo0 = SyncMNos[0];
	TmpNo1 = SyncMNos[1];

	Motor[TmpNo0].Mutex = TRUE;
	Motor[TmpNo1].Mutex = TRUE;

	StopAndFloatMotor (TmpNo0);
	StopAndFloatMotor (TmpNo1);
	SyncMNos[0] = UNUSED_SYNC_MOTOR;
	SyncMNos[1] = UNUSED_SYNC_MOTOR;

	MaxSyncSpeed = 0;
	Motor[TmpNo0].TargetSpeed = 0;
	Motor[TmpNo1].TargetSpeed = 0;

	Motor[TmpNo0].Mutex = FALSE;
	Motor[TmpNo1].Mutex = FALSE;
}

/*}}}*/
/*{{{  void TestAndFloatSyncedMotors (UBYTE MotorBitField, UBYTE SyncCmd)*/
/*
 *	float synchronised motors, but only if synchronised.
 */
void TestAndFloatSyncedMotors (UBYTE MotorBitField, UBYTE SyncCmd)
{
//  UBYTE   TmpNo0, TmpNo1;

	// Only if motors are already sync'ed
	if ((SyncMNos[0] != UNUSED_SYNC_MOTOR) && (SyncMNos[1] != UNUSED_SYNC_MOTOR)) {
		if (TRUE == SyncCmd) {
			//This is new sync'ed motor command
			if (((MotorBitField & (0x01 << SyncMNos[0])) && (MotorBitField & (0x01 << SyncMNos[1])))) {
				// Same motors used -> don't stop motors
			} else {
				// Only 2 motors can be sync'ed at a time -> stop sync'ed command
				FloatSyncedMotors ();
			}
		} else {
			if ((MotorBitField & (0x01 << SyncMNos[0])) || (MotorBitField & (0x01 << SyncMNos[1]))) {
				// Same motors used -> stop sync'ed command
				FloatSyncedMotors ();
			}
		}



/*

    // Check if new sync'ed motor command uses same motors as previous sync cmd
    if ((MotorBitField & (0x01 << SyncMNos[0])) && (MotorBitField & (0x01 << SyncMNos[1])))
    {
      // Check if only one of the sync'ed motors are affected by the new motor cmd
      if (((MotorBitField & (0x01 << SyncMNos[0])) || (MotorBitField & (0x01 << SyncMNos[1]))) || (TRUE == SyncCmd))
      {
        // The motor is in sync'ed mode -> float both sync'ed motors
        TmpNo0 = SyncMNos[0];
        TmpNo1 = SyncMNos[1];

        Motor[TmpNo0].Mutex  =  TRUE;
        Motor[TmpNo1].Mutex  =  TRUE;

        StopAndFloatMotor(TmpNo0);
        StopAndFloatMotor(TmpNo1);
        SyncMNos[0] = UNUSED_SYNC_MOTOR;
        SyncMNos[1] = UNUSED_SYNC_MOTOR;

        MaxSyncSpeed              = 0;
        Motor[TmpNo0].TargetSpeed = 0;
        Motor[TmpNo1].TargetSpeed = 0;

        Motor[TmpNo0].Mutex  =  FALSE;
        Motor[TmpNo1].Mutex  =  FALSE;
      }
    }*/
	}
}
/*}}}*/
/*{{{  static enum hrtimer_restart Device1TimerInterrupt1 (struct hrtimer *pTimer)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    Device1TimerInterrupt1
 *
 *  Motor timer interrupt function
 *
 *  Handles all motor regulation and timing related functionality
 *
 */
static enum hrtimer_restart Device1TimerInterrupt1 (struct hrtimer *pTimer)
{
	UBYTE No;
	UBYTE Test;

	static SLONG volatile TmpTacho;
	static SLONG volatile Tmp;

	hrtimer_forward_now (pTimer, Device1Time);
	for (No = 0; No < NO_OF_OUTPUT_PORTS; No++)
//while( 0 )
//  for (No = 0; No < 1; No++)
	{
		TmpTacho = Motor[No].IrqTacho;
		Tmp = (TmpTacho - Motor[No].OldTachoCnt);

		Motor[No].TachoCnt += Tmp;
		Motor[No].TachoSensor += Tmp;
		Motor[No].OldTachoCnt = TmpTacho;

		/* Update shared memory */
		pMotor[No].TachoCounts = Motor[No].TachoCnt;
		pMotor[No].Speed = Motor[No].Speed;
		pMotor[No].TachoSensor = Motor[No].TachoSensor;

		Motor[No].TimeCnt += Motor[No].TimeInc;	// Add or sub so that TimerCnt is 1 mS resolution

		if (FALSE == Motor[No].Mutex) {

			Test = dCalculateSpeed (No, &(Motor[No].Speed));
			switch (Motor[No].State) {
			case UNLIMITED_UNREG: /*{{{*/
				{
					if (Motor[No].TargetPower != Motor[No].Power) {
						Motor[No].Power = Motor[No].TargetPower;
						SetPower (No, Motor[No].Power);
					}
					Motor[No].TachoCnt = 0;
					Motor[No].TimeCnt = 0;

				}
				break;
				/*}}}*/
			case UNLIMITED_REG: /*{{{*/
				{
					dRegulateSpeed (No);
					Motor[No].TachoCnt = 0;
					Motor[No].TimeCnt = 0;
				}
				break;
				/*}}}*/
			case LIMITED_REG_STEPUP: /*{{{*/
				{
					UBYTE Status;
					SLONG StepCnt;
					SLONG StepCntTst;

					// Status used to check if ramp up has completed
					Status = FALSE;

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					GetCompareCounts (No, &StepCnt, &StepCntTst);

					if ((TRUE == CheckLessThanSpecial (StepCntTst, Motor[No].TachoCntUp, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
						Motor[No].TargetSpeed = ((StepCnt * (Motor[No].RampUpFactor)) / RAMP_FACTOR) + Motor[No].RampUpOffset;
						if (TRUE == CheckLessThanSpecial ((SLONG) Motor[No].TargetSpeed, ((SLONG) 6 * Motor[No].Dir), Motor[No].Dir)) {
							//Ensure minimum speed
							Motor[No].TargetSpeed = 6 * Motor[No].Dir;
						}
						dRegulateSpeed (No);
					} else {
						if (TRUE == Motor[No].BrakeAfter) {

							Motor[No].LockRampDown = TRUE;
							Status = RampDownToBrake (No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);
						} else {
							Status = TRUE;
						}
					}

					if (TRUE == Status) {
						// Ramp up completed check for next step
						if (FALSE == StepSpeedCheckTachoCntConst (No, Motor[No].TachoCntUp)) {
							StepSpeedCheckTachoCntDown (No, Motor[No].TachoCntUp);
						}
					}
				}
				break;
				/*}}}*/
			case LIMITED_REG_STEPCONST: /*{{{*/
				{
					SLONG StepCnt, StepCntTst;

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					GetCompareCounts (No, &StepCnt, &StepCntTst);

					if ((TRUE == CheckLessThanSpecial (StepCntTst, Motor[No].TachoCntConst, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
						dRegulateSpeed (No);
					} else {
						if (TRUE == Motor[No].BrakeAfter) {
							Motor[No].LockRampDown = TRUE;
							if (TRUE == RampDownToBrake (No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir)) {
								StepSpeedCheckTachoCntDown (No, Motor[No].TachoCntConst);
							}
						} else {
							StepSpeedCheckTachoCntDown (No, Motor[No].TachoCntConst);
						}
					}
				}
				break;
				/*}}}*/
			case LIMITED_REG_STEPDOWN: /*{{{*/
				{
					SLONG StepCnt;
					SBYTE NewSpeed;

					StepCnt = *StepPowerSteps[No];

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					if (TRUE == CheckLessThanSpecial (StepCnt, Motor[No].TachoCntDown, Motor[No].Dir)) {
						NewSpeed = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor)) / RAMP_FACTOR);
						if (TRUE == CheckLessThanSpecial ((4 * Motor[No].Dir), NewSpeed, Motor[No].Dir)) {
							Motor[No].TargetSpeed = NewSpeed;
						}
						dRegulateSpeed (No);
					} else {
						StepPowerStopMotor (No, Motor[No].TachoCntDown);
					}
				}
				break;
				/*}}}*/
			case LIMITED_UNREG_STEPUP: /*{{{*/
				{

					UBYTE Status;
					SLONG StepCnt;
					SLONG StepCntTst;

					// Status used to check if ramp up has completed
					Status = FALSE;

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					GetCompareCounts (No, &StepCnt, &StepCntTst);

					if ((TRUE == CheckLessThanSpecial (StepCntTst, Motor[No].TachoCntUp, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
						if (0 != Motor[No].Speed) {
							if (TRUE == CheckLessThanSpecial (Motor[No].Power, (StepCnt * (Motor[No].RampUpFactor)) /*/RAMP_FACTOR */ , Motor[No].Dir)) {
								// if very slow ramp up then power could be calculated as 0 for a while
								// avoid starting and stopping
								Motor[No].Power = (StepCnt * (Motor[No].RampUpFactor)) / RAMP_FACTOR;

							}

							if (TRUE == CheckLessThanSpecial (Motor[No].Power, 0, Motor[No].Dir)) {
								// Avoid motor turning the wrong way
								Motor[No].Power = Motor[No].Power * -1;
							}
						} else {
							Motor[No].Power += (20 * Motor[No].Dir);
						}
						SetPower (No, Motor[No].Power);
					} else {
						// Done Stepping up
						Motor[No].LockRampDown = TRUE;
						if (TRUE == Motor[No].BrakeAfter) {
							Status = RampDownToBrake (No, StepCnt, Motor[No].TachoCntUp, Motor[No].Dir);
						} else {
							Status = TRUE;
						}
					}

					if (TRUE == Status) {
						// Ramp up completed check for next step
						if (FALSE == StepPowerCheckTachoCntConst (No, Motor[No].TachoCntUp)) {
							StepPowerCheckTachoCntDown (No, Motor[No].TachoCntUp);
						}
					}
				}
				break;
				/*}}}*/
			case LIMITED_UNREG_STEPCONST: /*{{{*/
				{
					SLONG StepCnt, StepCntTst;

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					GetCompareCounts (No, &StepCnt, &StepCntTst);

					if ((TRUE == CheckLessThanSpecial (StepCntTst, Motor[No].TachoCntConst, Motor[No].Dir)) && (FALSE == Motor[No].LockRampDown)) {
					} else {
						if (TRUE == Motor[No].BrakeAfter) {

							Motor[No].LockRampDown = TRUE;
							if (TRUE == RampDownToBrake (No, StepCnt, Motor[No].TachoCntConst, Motor[No].Dir)) {
								StepPowerCheckTachoCntDown (No, Motor[No].TachoCntConst);
							}
						} else {
							StepPowerCheckTachoCntDown (No, Motor[No].TachoCntConst);
						}
					}
				}
				break;
				/*}}}*/
			case LIMITED_UNREG_STEPDOWN: /*{{{*/
				{
					SLONG StepCnt;

					StepCnt = *StepPowerSteps[No];

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					if (TRUE == CheckLessThanSpecial (StepCnt, Motor[No].TachoCntDown, Motor[No].Dir)) {
						if ((TRUE == CheckLessThanSpecial ((2 * Motor[No].Dir), Motor[No].Speed, Motor[No].Dir)) && (FALSE == MinRegEnabled[No])) {
							if (TRUE ==
							    CheckLessThanSpecial ((4 * Motor[No].Dir),
										  (Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor)) / RAMP_FACTOR)),
										  Motor[No].Dir)) {
								Motor[No].Power = Motor[No].TargetPower - ((StepCnt * (Motor[No].RampDownFactor)) / RAMP_FACTOR);
							}
							SetPower (No, Motor[No].Power);
						} else {

							MinRegEnabled[No] = TRUE;
							Motor[No].TargetSpeed = (2 * Motor[No].Dir);
							dRegulateSpeed (No);
						}
					} else {
						StepPowerStopMotor (No, Motor[No].TachoCntDown);
					}
				}
				break;
				/*}}}*/
			case LIMITED_STEP_SYNC:/*{{{*/
				{
					// Here motor are syncronized and supposed to drive straight

					UBYTE Cnt;
					UBYTE Status;
					SLONG StepCnt;

					StepCnt = *StepPowerSteps[No];

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					Status = FALSE;

					if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100))) {
						// Regulation is stretched to the limit....... Checked in both directions
						Status = TRUE;
						if (TRUE ==
						    CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[0]].Speed), (SLONG) (Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir)) {
							if (TRUE == CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[0]].Speed), (SLONG) (MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
								// Check for running the same direction as target speed
								if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
								    ((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0))) {
									MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
								}
							}
						}
					}
					if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100))) {
						// Regulation is stretched to the limit....... Checked in both directions
						Status = TRUE;
						if (TRUE ==
						    CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[1]].Speed), (SLONG) (Motor[SyncMNos[1]].TargetSpeed),
									  (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir))) {
							if (TRUE ==
							    CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[1]].Speed), (SLONG) (MaxSyncSpeed),
										  (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir))) {
								// Check for running the same direction as target speed
								if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
								    ((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0))) {
									MaxSyncSpeed = Motor[SyncMNos[1]].Speed;
								}
							}
						}
					}

					if (FALSE == Status) {
						if (TRUE == CheckLessThanSpecial ((SLONG) (MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir)) {
							// TargetSpeed has been reduced but now there is room to increase
							IncSpeed (Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
						}
					}

					for (Cnt = 0; Cnt < MAX_SYNC_MOTORS; Cnt++) {
						//Update all motors to the max sync speed
						Motor[SyncMNos[Cnt]].TargetSpeed = MaxSyncSpeed;
					}

					if (TRUE == CheckLessThanSpecial (Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
						DecSpeed (Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
					}

					if (TRUE == CheckLessThanSpecial (Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[1]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
						DecSpeed (Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
					}

					dRegulateSpeed (SyncMNos[0]);
					dRegulateSpeed (SyncMNos[1]);

					CheckforEndOfSync ();

				}
				break;
				/*}}}*/
			case SYNCED_SLAVE:/*{{{*/
				{
					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}
				}
				break;
				/*}}}*/
			case LIMITED_DIFF_TURN_SYNC: /*{{{*/
				{
					UBYTE Status;
					SLONG StepCnt;

					StepCnt = *StepPowerSteps[No];

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					Status = FALSE;

					if ((Motor[SyncMNos[0]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[0]].Power < (-MAX_PWM_CNT + 100))) {
						// Regulation is stretched to the limit....... in both directions
						Status = TRUE;
						if (TRUE ==
						    CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[0]].Speed), (SLONG) (Motor[SyncMNos[0]].TargetSpeed), Motor[SyncMNos[0]].Dir)) {
							if (TRUE == CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[0]].Speed), (SLONG) (MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
								// Check for running the same direction as target speed
								if (((Motor[SyncMNos[0]].Speed <= 0) && (Motor[SyncMNos[0]].TargetSpeed <= 0)) ||
								    ((Motor[SyncMNos[0]].Speed >= 0) && (Motor[SyncMNos[0]].TargetSpeed >= 0))) {
									MaxSyncSpeed = Motor[SyncMNos[0]].Speed;
								}
							}
						}
					}

					if ((Motor[SyncMNos[1]].Power > (MAX_PWM_CNT - 100)) || (Motor[SyncMNos[1]].Power < (-MAX_PWM_CNT + 100))) {
						// Regulation is stretched to the limit....... in both directions
						Status = TRUE;
						if (TRUE ==
						    CheckLessThanSpecial ((SLONG) (Motor[SyncMNos[1]].Speed), (SLONG) (Motor[SyncMNos[1]].TargetSpeed),
									  (Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir))) {
							if (TRUE ==
							    CheckLessThanSpecial ((SLONG)
										  ((Motor[SyncMNos[1]].Speed * 100) / (Motor[SyncMNos[1]].TurnRatio * Motor[SyncMNos[1]].Dir)),
										  (SLONG) (MaxSyncSpeed), Motor[SyncMNos[0]].Dir)) {
								if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == Motor[SyncMNos[1]].Speed)) {
									MaxSyncSpeed = 0;
								} else {
									// Check for running the same direction as target speed
									if (((Motor[SyncMNos[1]].Speed <= 0) && (Motor[SyncMNos[1]].TargetSpeed <= 0)) ||
									    ((Motor[SyncMNos[1]].Speed >= 0) && (Motor[SyncMNos[1]].TargetSpeed >= 0))) {
										MaxSyncSpeed =
											((Motor[SyncMNos[1]].Speed * 100) / Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir;
									}
								}
							}
						}
					}

					if (FALSE == Status) {
						if (TRUE == CheckLessThanSpecial ((SLONG) (MaxSyncSpeed), Motor[SyncMNos[0]].TargetPower, Motor[SyncMNos[0]].Dir)) {
							// TargetSpeed has been reduced but now there is room to increase
							IncSpeed (Motor[SyncMNos[0]].Dir, &MaxSyncSpeed);
						}
					}
					// Set the new
					Motor[SyncMNos[0]].TargetSpeed = MaxSyncSpeed;
					if ((0 == Motor[SyncMNos[1]].TurnRatio) || (0 == MaxSyncSpeed)) {
						Motor[SyncMNos[1]].TargetSpeed = 0;
					} else {
						Motor[SyncMNos[1]].TargetSpeed = ((MaxSyncSpeed * Motor[SyncMNos[1]].TurnRatio) / 100) * Motor[SyncMNos[1]].Dir;
					}

					if (0 != Motor[SyncMNos[1]].TurnRatio) {
						if (TRUE ==
						    CheckLessThanSpecial ((((Motor[SyncMNos[1]].TachoCnt * 100) / Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir),
									  Motor[SyncMNos[0]].TachoCnt, Motor[SyncMNos[0]].Dir)) {
							DecSpeed (Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[0]].TargetSpeed));
						}
					}

					if (0 != Motor[SyncMNos[1]].TurnRatio) {
						if (TRUE ==
						    CheckLessThanSpecial (Motor[SyncMNos[0]].TachoCnt,
									  ((Motor[SyncMNos[1]].TachoCnt * 100) / (Motor[SyncMNos[1]].TurnRatio) * Motor[SyncMNos[1]].Dir),
									  Motor[SyncMNos[0]].Dir)) {
							DecSpeed (Motor[SyncMNos[1]].Dir * Motor[SyncMNos[0]].Dir, &(Motor[SyncMNos[1]].TargetSpeed));
						}
					}

					dRegulateSpeed (SyncMNos[0]);
					dRegulateSpeed (SyncMNos[1]);

					CheckforEndOfSync ();

				}
				break;
				/*}}}*/
			case RAMP_DOWN_SYNC:  /*{{{*/
				{

					SLONG Count0, Count1;

					if (StepPowerSteps[No] != &(Motor[No].TachoCnt)) {
						Motor[No].TachoCnt = 0;
					} else {
						Motor[No].TimeCnt = 0;
					}

					// Duration is either dependent on timer ticks or tacho counts
					GetSyncDurationCnt (&Count0, &Count1);

					if (TRUE == CheckLessThanSpecial (Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir)) {

						RampDownToBrake (SyncMNos[0], Count0, Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir);

						if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
							// Needs to adjust second synchronised in time mode - both motor needs to run for the same
							// amount of time but not at the same speed
							RampDownToBrake (SyncMNos[1], ((Count1 * Motor[SyncMNos[1]].TurnRatio) / 100),
									 ((Motor[SyncMNos[1]].TachoCntConst * Motor[SyncMNos[1]].TurnRatio) / 100),
									 (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
						} else {
							RampDownToBrake (SyncMNos[1], Count1, Motor[SyncMNos[1]].TachoCntConst, (Motor[SyncMNos[0]].Dir * Motor[SyncMNos[1]].Dir));
						}
					} else {
						MaxSyncSpeed = 0;
						Motor[SyncMNos[0]].TargetSpeed = 0;
						Motor[SyncMNos[1]].TargetSpeed = 0;
						StepPowerStopMotor (SyncMNos[0], Motor[SyncMNos[0]].TachoCntConst);
						StepPowerStopMotor (SyncMNos[1], Motor[SyncMNos[1]].TachoCntConst);
					}
				}
				break;
				/*}}}*/
			case STOP_MOTOR: /*{{{*/
				{
					if (PrgStopTimer[No]) {
						PrgStopTimer[No]--;
					} else {
						Motor[No].State = IDLE;
						Motor[No].TargetState = UNLIMITED_UNREG;
						SetCoast (No);
					}
				}
				break;
				/*}}}*/
			case BRAKED: /*{{{*/
				{
					BrakeMotor (No, Motor[No].TachoCnt);
				}
				break;
				/*}}}*/
			case IDLE: /*{{{*/
				{	/* Intentionally left empty */
				}
				break;
				/*}}}*/
			default: /*{{{*/
				{	/* Intentionally left empty */
				}
				break;
				/*}}}*/
			}
		}
	}
	return (HRTIMER_RESTART);
}
/*}}}*/
/*{{{  void GetSyncDurationCnt (SLONG *pCount0, SLONG *pCount1)*/
/*
 *	gets counters for synchronised motors (?).
 */
void GetSyncDurationCnt (SLONG *pCount0, SLONG *pCount1)
{
	if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
		*pCount0 = *TimerSteps[SyncMNos[0]];
		*pCount1 = *TimerSteps[SyncMNos[1]];
	} else {
		*pCount0 = *TachoSteps[SyncMNos[0]];
		*pCount1 = *TachoSteps[SyncMNos[1]];
	}
}
/*}}}*/
/*{{{  void CheckforEndOfSync (void)*/
/*
 *	checks for end-of-synchronisation in ramp-down (?).
 */
void CheckforEndOfSync (void)
{
	SLONG Count0, Count1;
	SBYTE Speed;

	// Duration is either dependent on timer ticks or tacho counts
	GetSyncDurationCnt (&Count0, &Count1);

	if (StepPowerSteps[SyncMNos[0]] == TimerSteps[SyncMNos[0]]) {
		Speed = (Motor[SyncMNos[0]].Speed);
	} else {
		Speed = (Motor[SyncMNos[0]].Speed / 2);
	}

	if ((TRUE == CheckLessThanSpecial (Count0 + (SLONG) (Speed), Motor[SyncMNos[0]].TachoCntConst, Motor[SyncMNos[0]].Dir)) || (0 == Motor[SyncMNos[0]].TachoCntConst)) {
	} else {
		Motor[SyncMNos[0]].State = RAMP_DOWN_SYNC;
	}
}
/*}}}*/
/*{{{  static void pwm_command_handler (const signed char *Buf)*/
/*! \page PWMModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 *
 */
/*! \brief    pwm_command_handler
 *
 *  VALID COMMANDS:
 *
 *  opOUTPUT_SET_TYPE:
 *  opOUTPUT_GET_TYPE
 *  opOUTPUT_SET_TYPE
 *  opOUTPUT_RESET:       Resets the output tacho counters
 *  opOUTPUT_STOP:        Stops the motor - either Braked or coasted
 *  opOUTPUT_POWER:       Sets the power - Duty -                        Do not start the motor
 *  opOUTPUT_SPEED:       Sets the Speed - setpoint for regulation -     Do not start the motor
 *  opOUTPUT_START:       Starts the motor if not started
 *  opOUTPUT_POLARITY:    Sets the polarity of the motor -               Do not start the motor
 *  opOUTPUT_READ:
 *  opOUTPUT_TEST:
 *  opOUTPUT_READY:
 *  opOUTPUT_POSITION:    Runs the motor to the absolute tacho positon - Starts the motor
 *  opOUTPUT_STEP_POWER:  Runs the motor un-regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_POWER:  Runs the motor un-regulated with ramp up const and down according to time
 *  opOUTPUT_STEP_SPEED:  Runs the motor regulated with ramp up const and down according to the tacho
 *  opOUTPUT_TIME_SPEED:  Runs the motor regulated with ramp up const and down according to the time
 *  opOUTPUT_STEP_SYNC:   Runs two motors regulated and syncronized, duration as specified by tacho cnts
 *  opOUTPUT_TIME_SYNC:   Runs two motors regulated and syncronized, duration as specified by time
 *  opOUTPUT_CLR_COUNT:   Resets the tacho count related to when motor is used as a sensor
 *
 *
 *  Default state:        TBD
 */
static void pwm_command_handler (const signed char *Buf)
{
	switch ((UBYTE) (Buf[0])) {
	case opPROGRAM_STOP: /*{{{*/
		{
			UBYTE Tmp;

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {

				Motor[Tmp].Mutex = TRUE;
				ReadyStatus &= ~(0x01 << Tmp);	// Clear Ready flag
				TestStatus &= ~(0x01 << Tmp);	// Clear Test flag

				Motor[Tmp].Power = 0;
				Motor[Tmp].Speed = 0;
				MaxSyncSpeed = 0;
				Motor[Tmp].TargetSpeed = 0;

				if (((IDLE == Motor[Tmp].State) && ((OutputRead (Tmp, DIR0)) && (OutputRead (Tmp, DIR1)))) || (BRAKED == Motor[Tmp].State)) {
					PrgStopTimer[Tmp] = 100 / SOFT_TIMER_MS;
					Motor[Tmp].State = STOP_MOTOR;
					Motor[Tmp].TargetState = UNLIMITED_UNREG;
					SetBrake (Tmp);
				} else {
					Motor[Tmp].State = IDLE;
					Motor[Tmp].TargetState = UNLIMITED_UNREG;
					SetCoast (Tmp);
				}
				Motor[Tmp].Mutex = FALSE;
			}
		}
		/*}}}*/
	case opPROGRAM_START: /*{{{*/
		{
			UBYTE Tmp;

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				Motor[Tmp].Pol = 1;
			}
			SyncMNos[0] = UNUSED_SYNC_MOTOR;
			SyncMNos[1] = UNUSED_SYNC_MOTOR;
		}
		break;
		/*}}}*/
	case opOUTPUT_SET_TYPE: /*{{{*/
		{
			UBYTE Tmp;

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[Tmp + 1] != Motor[Tmp].Type) {
					Motor[Tmp].Mutex = TRUE;
					if ((TYPE_TACHO == Buf[Tmp + 1]) || (TYPE_MINITACHO == Buf[Tmp + 1])) {
						// There is a motor attached the port
						CLEARTachoArray (Tmp);
						SETMotorType (Tmp, Buf[Tmp + 1]);	//  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
						SETAvgTachoCount (Tmp, 0);	//  Switching motor => speed = 0
					} else {
						// No motor is connected
						Motor[Tmp].State = IDLE;
						SetCoast (Tmp);
					}
					Motor[Tmp].Type = Buf[Tmp + 1];

					// All counts are reset when motor type changes
					Motor[Tmp].TachoCnt = 0;
					pMotor[Tmp].TachoCounts = 0;
					Motor[Tmp].TimeCnt = 0;
					pMotor[Tmp].TachoSensor = 0;
					Motor[Tmp].TachoSensor = 0;
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_RESET: /*{{{*/
		{
			UBYTE Tmp;

			TestAndFloatSyncedMotors (Buf[1], FALSE);

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					Motor[Tmp].TachoCnt = 0;
					pMotor[Tmp].TachoCounts = 0;
					Motor[Tmp].TimeCnt = 0;
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_CLR_COUNT: /*{{{*/
		{
			UBYTE Tmp;
			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					pMotor[Tmp].TachoSensor = 0;
					Motor[Tmp].TachoSensor = 0;
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_STOP: /*{{{*/
		{
			UBYTE Tmp;

			TestAndFloatSyncedMotors (Buf[1], FALSE);

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					if (Buf[2]) {
						StopAndBrakeMotor (Tmp);
					} else {
						StopAndFloatMotor (Tmp);
					}

					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_POWER: /*{{{*/
		{
			UBYTE Tmp;

			TestAndFloatSyncedMotors (Buf[1], FALSE);

			CheckSpeedPowerLimits (&(Buf[2]));

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					Motor[Tmp].TargetPower = (SLONG) (Buf[2]) * (SLONG) (Motor[Tmp].Pol) * (SLONG) SPEED_PWMCNT_REL;

					if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
						Motor[Tmp].TargetState = UNLIMITED_UNREG;
					}
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_SPEED: /*{{{*/
		{
			UBYTE Tmp;

			TestAndFloatSyncedMotors (Buf[1], FALSE);

			CheckSpeedPowerLimits (&(Buf[2]));

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					Motor[Tmp].TargetSpeed = (Buf[2]) * (Motor[Tmp].Pol);
					if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
						Motor[Tmp].TargetState = UNLIMITED_REG;
					}
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_START: /*{{{*/
		{
			UBYTE Tmp;
			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					if ((IDLE == Motor[Tmp].State) || (BRAKED == Motor[Tmp].State)) {
						Motor[Tmp].State = Motor[Tmp].TargetState;
					}
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_POLARITY: /*{{{*/
		{
			UBYTE Tmp;

			TestAndFloatSyncedMotors (Buf[1], FALSE);

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (Buf[1] & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;
					if (0 == (SBYTE) Buf[2]) {
						/* 0 == Toggle */
						Motor[Tmp].TargetPower = (Motor[Tmp].TargetPower) * -1;
						Motor[Tmp].TargetSpeed = (Motor[Tmp].TargetSpeed) * -1;

						if (1 == Motor[Tmp].Pol) {
							Motor[Tmp].Pol = -1;
						} else {
							Motor[Tmp].Pol = 1;
						}
					} else {

						if (Motor[Tmp].Pol != (SBYTE) Buf[2]) {
							Motor[Tmp].TargetPower = (Motor[Tmp].TargetPower) * -1;
							Motor[Tmp].TargetSpeed = (Motor[Tmp].TargetSpeed) * -1;
							Motor[Tmp].Pol = Buf[2];
						}
					}
					SetPower (Tmp, Motor[Tmp].TargetPower);
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_POSITION: /*{{{*/
		{
		}
		break;
		/*}}}*/
	case opOUTPUT_STEP_POWER: /*{{{*/
		{
			UBYTE Tmp;
			STEPPOWER StepPower;

			memcpy ((UBYTE *) (&(StepPower.Cmd)), &Buf[0], sizeof (StepPower));

			TestAndFloatSyncedMotors (StepPower.Nos, FALSE);

			CheckSpeedPowerLimits (&(StepPower.Power));

			// Adjust if there is inconsistency between power and steps
			if (((StepPower.Power < 0) && ((StepPower.Step1 > 0) || (StepPower.Step2 > 0) || (StepPower.Step3 > 0))) ||
			    ((StepPower.Power > 0) && ((StepPower.Step1 < 0) || (StepPower.Step2 < 0) || (StepPower.Step3 < 0)))) {
				StepPower.Step1 *= -1;
				StepPower.Step2 *= -1;
				StepPower.Step3 *= -1;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (StepPower.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					StepPowerSteps[Tmp] = TachoSteps[Tmp];

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag

					Motor[Tmp].TargetPower = StepPower.Power * SPEED_PWMCNT_REL * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntUp = StepPower.Step1 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntConst = StepPower.Step2 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntDown = StepPower.Step3 * (Motor[Tmp].Pol);
					Motor[Tmp].TargetBrake = StepPower.Brake;
					if (Motor[Tmp].TargetPower >= 0) {
						Motor[Tmp].Dir = 1;
					} else {
						Motor[Tmp].Dir = -1;
					}

					TimeOutSpeed0[Tmp] = FREERunning24bittimer;

					if (FALSE == StepPowerCheckTachoCntUp (Tmp)) {
						SLONG AdjustTachoValue = 0;
						if (FALSE == StepPowerCheckTachoCntConst (Tmp, AdjustTachoValue)) {
							StepPowerCheckTachoCntDown (Tmp, AdjustTachoValue);
						}
					}
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_TIME_POWER: /*{{{*/
		{
			UBYTE Tmp;
			TIMEPOWER TimePower;
			SLONG Inc;

			memcpy ((UBYTE *) (&(TimePower.Cmd)), &Buf[0], sizeof (TimePower));

			TestAndFloatSyncedMotors (TimePower.Nos, FALSE);

			// Adjust if there is inconsistency between power and Time
			if (((TimePower.Power < 0) && ((TimePower.Time1 > 0) || (TimePower.Time2 > 0) || (TimePower.Time3 > 0))) ||
			    ((TimePower.Power > 0) && ((TimePower.Time1 < 0) || (TimePower.Time2 < 0) || (TimePower.Time3 < 0)))) {
				TimePower.Time1 *= -1;
				TimePower.Time2 *= -1;
				TimePower.Time3 *= -1;
			}

			if ((TimePower.Time1 > 0) || (TimePower.Time2 > 0) || (TimePower.Time3 > 0)) {
				Inc = SOFT_TIMER_MS;
			} else {
				Inc = -SOFT_TIMER_MS;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (TimePower.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					StepPowerSteps[Tmp] = TimerSteps[Tmp];
					Motor[Tmp].TimeInc = Inc * (Motor[Tmp].Pol);
					*StepPowerSteps[Tmp] = 0;

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag

					Motor[Tmp].TargetPower = TimePower.Power * SPEED_PWMCNT_REL * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntUp = TimePower.Time1 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntConst = TimePower.Time2 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntDown = TimePower.Time3 * (Motor[Tmp].Pol);
					Motor[Tmp].TargetBrake = TimePower.Brake;

					if (Motor[Tmp].TargetPower >= 0) {
						Motor[Tmp].Dir = 1;
					} else {
						Motor[Tmp].Dir = -1;
					}

					TimeOutSpeed0[Tmp] = FREERunning24bittimer;

					if (FALSE == StepPowerCheckTachoCntUp (Tmp)) {
						SLONG AdjustTachoValue = 0;
						if (FALSE == StepPowerCheckTachoCntConst (Tmp, AdjustTachoValue)) {
							StepPowerCheckTachoCntDown (Tmp, AdjustTachoValue);
						}
					}
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_STEP_SPEED: /*{{{*/
		{
			UBYTE Tmp;
			STEPSPEED StepSpeed;

			memcpy ((UBYTE *) (&(StepSpeed.Cmd)), &Buf[0], sizeof (StepSpeed));

			TestAndFloatSyncedMotors (StepSpeed.Nos, FALSE);

			CheckSpeedPowerLimits (&(StepSpeed.Speed));

			// Adjust if there is inconsistency between Speed and Steps
			if (((StepSpeed.Speed < 0) && ((StepSpeed.Step1 > 0) || (StepSpeed.Step2 > 0) || (StepSpeed.Step3 > 0))) ||
			    ((StepSpeed.Speed > 0) && ((StepSpeed.Step1 < 0) || (StepSpeed.Step2 < 0) || (StepSpeed.Step3 < 0)))) {
				StepSpeed.Step1 *= -1;
				StepSpeed.Step2 *= -1;
				StepSpeed.Step3 *= -1;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (StepSpeed.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					StepPowerSteps[Tmp] = TachoSteps[Tmp];

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag

					Motor[Tmp].TargetPower = StepSpeed.Speed * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntUp = StepSpeed.Step1 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntConst = StepSpeed.Step2 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntDown = StepSpeed.Step3 * (Motor[Tmp].Pol);
					Motor[Tmp].TargetBrake = StepSpeed.Brake;

					if (Motor[Tmp].TargetPower >= 0) {
						Motor[Tmp].Dir = 1;
					} else {
						Motor[Tmp].Dir = -1;
					}

					TimeOutSpeed0[Tmp] = FREERunning24bittimer;

					if (FALSE == StepSpeedCheckTachoCntUp (Tmp)) {
						SLONG AdjustTachoValue = 0;
						if (FALSE == StepSpeedCheckTachoCntConst (Tmp, AdjustTachoValue)) {
							StepSpeedCheckTachoCntDown (Tmp, AdjustTachoValue);
						}
					}
					Motor[Tmp].TargetSpeed = Motor[Tmp].TargetPower;
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_TIME_SPEED: /*{{{*/
		{
			UBYTE Tmp;
			SLONG Inc;
			TIMESPEED TimeSpeed;


			memcpy ((UBYTE *) (&(TimeSpeed.Cmd)), &Buf[0], sizeof (TimeSpeed));

			TestAndFloatSyncedMotors (TimeSpeed.Nos, FALSE);

			CheckSpeedPowerLimits (&(TimeSpeed.Speed));

			// Adjust if there is inconsistency between Speed and Time
			if (((TimeSpeed.Speed < 0) && ((TimeSpeed.Time1 > 0) || (TimeSpeed.Time2 > 0) || (TimeSpeed.Time3 > 0))) ||
			    ((TimeSpeed.Speed > 0) && ((TimeSpeed.Time1 < 0) || (TimeSpeed.Time2 < 0) || (TimeSpeed.Time3 < 0)))) {
				TimeSpeed.Time1 *= -1;
				TimeSpeed.Time2 *= -1;
				TimeSpeed.Time3 *= -1;
			}

			if ((TimeSpeed.Time1 > 0) || (TimeSpeed.Time2 > 0) || (TimeSpeed.Time3 > 0)) {
				Inc = SOFT_TIMER_MS;
			} else {
				Inc = -SOFT_TIMER_MS;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (TimeSpeed.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					StepPowerSteps[Tmp] = TimerSteps[Tmp];
					Motor[Tmp].TimeInc = Inc * (Motor[Tmp].Pol);
					*StepPowerSteps[Tmp] = 0;

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag

					Motor[Tmp].TargetPower = TimeSpeed.Speed * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntUp = TimeSpeed.Time1 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntConst = TimeSpeed.Time2 * (Motor[Tmp].Pol);
					Motor[Tmp].TachoCntDown = TimeSpeed.Time3 * (Motor[Tmp].Pol);
					Motor[Tmp].TargetBrake = TimeSpeed.Brake;

					if (Motor[Tmp].TargetPower >= 0) {
						Motor[Tmp].Dir = 1;
					} else {
						Motor[Tmp].Dir = -1;
					}

					TimeOutSpeed0[Tmp] = FREERunning24bittimer;

					if (FALSE == StepSpeedCheckTachoCntUp (Tmp)) {
						SLONG AdjustTachoValue = 0;
						if (FALSE == StepSpeedCheckTachoCntConst (Tmp, AdjustTachoValue)) {
							StepSpeedCheckTachoCntDown (Tmp, AdjustTachoValue);
						}
					}
					Motor[Tmp].TargetSpeed = Motor[Tmp].TargetPower;
					Motor[Tmp].Mutex = FALSE;
				}
			}
		}
		break;
		/*}}}*/
	case opOUTPUT_STEP_SYNC: /*{{{*/
		{
			UBYTE No = 0;
			UBYTE Tmp;
			STEPSYNC StepSync;

			memcpy ((UBYTE *) (&(StepSync.Cmd)), &Buf[0], sizeof (StepSync));

			TestAndFloatSyncedMotors (StepSync.Nos, TRUE);

			//Check if exceeding speed limits
			CheckSpeedPowerLimits (&(StepSync.Speed));

			if (((StepSync.Speed < 0) && (StepSync.Step > 0)) || ((StepSync.Speed > 0) && (StepSync.Step < 0))) {
				StepSync.Step *= -1;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {

				if (StepSync.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag
					StepPowerSteps[Tmp] = TachoSteps[Tmp];
					Motor[Tmp].TargetBrake = StepSync.Brake;

					if (0 == StepSync.Step) {
						// If run forever
						*StepPowerSteps[Tmp] = 0;
					}

					if (0 == StepSync.Turn) {
						// Synced motors are going straight
						Motor[Tmp].TargetPower = StepSync.Speed;
						Motor[Tmp].TurnRatio = 100;
						Motor[Tmp].TachoCntConst = StepSync.Step;
						SyncMNos[No] = Tmp;
						TimeOutSpeed0[Tmp] = FREERunning24bittimer;
						MaxSyncSpeed = StepSync.Speed;

						// Find the direction the main motor will drive
						if (0 <= StepSync.Speed) {
							Motor[Tmp].Dir = NON_INV;
						} else {
							Motor[Tmp].Dir = INV;
						}

						if (0 == No) {
							Motor[Tmp].State = LIMITED_STEP_SYNC;
						} else {
							Motor[Tmp].State = SYNCED_SLAVE;
							Motor[Tmp].Dir = NON_INV;
						}
					} else {
						// Turning
						UBYTE MotorIndex;

						if (0 < StepSync.Turn) {
							MotorIndex = No;
						} else {
							// Invert if turning left (right motor runs fastest)
							MotorIndex = 1 - No;
						}

						if (0 == MotorIndex) {
							if (StepSync.Speed > 0) {
								Motor[Tmp].Dir = NON_INV;
							} else {
								Motor[Tmp].Dir = INV;
							}
							Motor[Tmp].TurnRatio = StepSync.Turn;
						} else {
							if ((StepSync.Turn < 100) && (StepSync.Turn > -100)) {
								Motor[Tmp].Dir = NON_INV;

								if (StepSync.Turn > 0)	// Invert the ratio in the first quarter
								{
									Motor[Tmp].TurnRatio = 100 - StepSync.Turn;
								} else {
									Motor[Tmp].TurnRatio = -100 - StepSync.Turn;
								}
							} else {
								if ((StepSync.Turn >= 100) || (StepSync.Turn <= -100)) {
									// Both motors are turning
									Motor[Tmp].Dir = INV;
									if (StepSync.Turn <= -100) {
										Motor[Tmp].TurnRatio = (StepSync.Turn + 100);
									} else {
										Motor[Tmp].TurnRatio = (StepSync.Turn - 100);
									}
								}
							}
						}

						SyncMNos[MotorIndex] = Tmp;

						if (0 > StepSync.Turn) {
							Motor[Tmp].TurnRatio *= -1;
						}

						if (0 == MotorIndex) {
							Motor[Tmp].TargetPower = StepSync.Speed;
							Motor[Tmp].TachoCntConst = StepSync.Step;
							MaxSyncSpeed = StepSync.Speed;
							Motor[Tmp].State = LIMITED_DIFF_TURN_SYNC;
						} else {
							Motor[Tmp].TargetPower =
								(SLONG) (((SLONG) (StepSync.Speed) * (SLONG) (Motor[Tmp].TurnRatio)) / ((SLONG) 100 * Motor[Tmp].Dir));
							Motor[Tmp].TachoCntConst = ((StepSync.Step * (SLONG) (Motor[Tmp].TurnRatio)) / 100) * Motor[Tmp].Dir;
							Motor[Tmp].State = SYNCED_SLAVE;
						}

						TimeOutSpeed0[Tmp] = FREERunning24bittimer;
					}
					No++;
				}
			}
			Motor[SyncMNos[0]].Mutex = FALSE;
			Motor[SyncMNos[1]].Mutex = FALSE;
		}
		break;
		/*}}}*/
	case opOUTPUT_TIME_SYNC: /*{{{*/
		{
			SLONG Inc;
			UBYTE No = 0;
			UBYTE Tmp;
			TIMESYNC TimeSync;

			memcpy ((UBYTE *) (&(TimeSync.Cmd)), &Buf[0], sizeof (TimeSync));

			TestAndFloatSyncedMotors (TimeSync.Nos, TRUE);

			//Check if exceeding speed limits
			CheckSpeedPowerLimits (&(TimeSync.Speed));

			if (((TimeSync.Speed < 0) && (TimeSync.Time > 0)) || ((TimeSync.Speed > 0) && (TimeSync.Time < 0))) {
				TimeSync.Time *= -1;
			}

			if (TimeSync.Time > 0) {
				Inc = SOFT_TIMER_MS;
			} else {
				Inc = -SOFT_TIMER_MS;
			}

			for (Tmp = 0; Tmp < OUTPUTS; Tmp++) {
				if (TimeSync.Nos & (1 << Tmp)) {
					Motor[Tmp].Mutex = TRUE;

					ReadyStatus |= (0x01 << Tmp);	// Set Ready flag
					TestStatus |= (0x01 << Tmp);	// Set Test flag

					StepPowerSteps[Tmp] = TimerSteps[Tmp];
					Motor[Tmp].TimeInc = Inc;
					*TimerSteps[Tmp] = 0;

					Motor[Tmp].TargetBrake = TimeSync.Brake;

					if (0 == TimeSync.Turn) {
						// Synced motors are going straight
						Motor[Tmp].TargetPower = TimeSync.Speed;
						Motor[Tmp].TurnRatio = 100;
						Motor[Tmp].TachoCntConst = TimeSync.Time;
						SyncMNos[No] = Tmp;
						TimeOutSpeed0[Tmp] = FREERunning24bittimer;
						MaxSyncSpeed = TimeSync.Speed;

						// Find the direction the main motor will drive
						if (0 <= TimeSync.Speed) {
							Motor[Tmp].Dir = NON_INV;
						} else {
							Motor[Tmp].Dir = INV;
						}

						if (0 == No) {
							Motor[Tmp].State = LIMITED_STEP_SYNC;
						} else {
							Motor[Tmp].State = SYNCED_SLAVE;
							Motor[Tmp].Dir = NON_INV;
						}
					} else {
						//   Turning
						UBYTE MotorIndex;

						if (0 < TimeSync.Turn) {
							MotorIndex = No;
						} else {
							// Invert if turning left (right motor runs fastest)
							MotorIndex = 1 - No;
						}

						if (0 == MotorIndex) {
							if (TimeSync.Speed > 0) {
								Motor[Tmp].Dir = NON_INV;
							} else {
								Motor[Tmp].Dir = INV;
							}
							Motor[Tmp].TurnRatio = TimeSync.Turn;
						} else {
							if ((TimeSync.Turn < 100) && (TimeSync.Turn > -100)) {
								Motor[Tmp].Dir = NON_INV;

								if (TimeSync.Turn > 0)	// Invert the ratio in the first quarter
								{
									Motor[Tmp].TurnRatio = 100 - TimeSync.Turn;
								} else {
									Motor[Tmp].TurnRatio = -100 - TimeSync.Turn;
								}
							} else {
								if ((TimeSync.Turn >= 100) || (TimeSync.Turn <= -100)) {
									// Both motors are turning
									Motor[Tmp].Dir = INV;
									if (TimeSync.Turn <= -100) {
										Motor[Tmp].TurnRatio = (TimeSync.Turn + 100);
									} else {
										Motor[Tmp].TurnRatio = (TimeSync.Turn - 100);
									}
								}
							}
						}

						SyncMNos[MotorIndex] = Tmp;

						if (0 > TimeSync.Turn) {
							Motor[Tmp].TurnRatio *= -1;
						}

						if (0 == MotorIndex) {
							Motor[Tmp].TargetPower = TimeSync.Speed;
							Motor[Tmp].TachoCntConst = TimeSync.Time;
							MaxSyncSpeed = TimeSync.Speed;
							Motor[Tmp].State = LIMITED_DIFF_TURN_SYNC;
						} else {
							Motor[Tmp].TargetPower =
								(SLONG) (((SLONG) (TimeSync.Speed) * (SLONG) (Motor[Tmp].TurnRatio)) / ((SLONG) 100 * Motor[Tmp].Dir));
							Motor[Tmp].TachoCntConst = TimeSync.Time * Motor[Tmp].Dir;
							Motor[Tmp].TimeInc *= Motor[Tmp].Dir;
							Motor[Tmp].State = SYNCED_SLAVE;
						}
						TimeOutSpeed0[Tmp] = FREERunning24bittimer;
					}
					No++;
				}
			}
			Motor[SyncMNos[0]].Mutex = FALSE;
			Motor[SyncMNos[1]].Mutex = FALSE;
		}
		break;
		/*}}}*/
	default: /*{{{*/
		{
		}
		break;
		/*}}}*/
	}

}

/*}}}*/
/*{{{  static ssize_t Device1Write (struct file *f, const char *buffer, size_t count, loff_t *dptr)*/
/*
 *	called to write bytes to the motor device.
 */
static ssize_t Device1Write (struct file *f, const char *buffer, size_t count, loff_t *dptr)
{
	SBYTE kbuf[16];

	memset (kbuf, 0, 16);
	if (count > 16) {
		count = 16;
	}
	copy_from_user (kbuf, buffer, count);
	pwm_command_handler (kbuf);

	return count;
}
/*}}}*/

/*{{{  OLDCODE*/
// static ssize_t Device1Read(struct file *File,char *Buffer,size_t Count,loff_t *Offset)
/*}}}*/
/*{{{  OLDCODE (struct miscdevice Device1 & fileops*/
// static    const struct file_operations Device1Entries =
// {
//   .owner        = THIS_MODULE,
//   .read         = Device1Read,
//   .write        = Device1Write
// };
// 
// 
// static    struct miscdevice Device1 =
// {
//   MISC_DYNAMIC_MINOR,
//   DEVICE_NAME,
//   &Device1Entries
// };
/*}}}*/

/*{{{  struct miscdevice Device1 & fileops*/
static const struct file_operations Device1Entries = {
	.owner = THIS_MODULE,
	.write = Device1Write
};

static struct miscdevice Device1 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &Device1Entries,
	.mode = 0222
};
/*}}}*/

#if 0
/*{{{  void GetPeripheralBasePtr (ULONG Address, ULONG Size, ULONG **Ptr)*/
// /*! \page PwmModule
//  *
//  *  <hr size="1"/>
//  *  <b>     write </b>
//  *
//  */
// /*! \brief    GetPeripheralBasePtr
//  *
//  *  Helper function for getting the peripheral HW base address
//  *
//  */
#warning "Use proper kernel methods for this - see the pullup update in d_ui.c"

void GetPeripheralBasePtr (ULONG Address, ULONG Size, ULONG **Ptr)
{
	/* eCAP0 pointer */
//if (request_mem_region(Address,Size,MODULE_NAME) >= 0)
	if (request_mem_region (Address, Size, "ev3dev_pwm") >= 0) {

		*Ptr = (ULONG *) ioremap (Address, Size);

		if (*Ptr != NULL) {
// #ifdef DEBUG
//    printk("%s memory Remapped from 0x%08lX to 0x%08lX\n",DEVICE_NAME,Address,(unsigned long)*Ptr);
			printk ("ev3dev_pwm: memory Remapped from 0x%08lX to 0x%08lX\n", Address, (unsigned long) *Ptr);
// #endif
		} else {
			printk ("Memory remap ERROR");
		}
	} else {
		printk ("Region request error");
	}
}
/*}}}*/
#endif
/*{{{  static int Device1Init (void)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    Device1Init
 *
 */
static int Device1Init (void)
{
	int Result = -1;
	UBYTE Tmp;

	Result = misc_register (&Device1);
	if (Result) {
		printk ("  failed to register device %s\n", DEVICE_NAME);
		goto out_err;
	}

#ifdef DEBUG
	printk ("  %s device register succes\n", DEVICE_NAME);
#endif

	EV3IO_WRITE32 (0x00000003, ev3mem[EV3IO_PSC1], 0x291);	/* Setup ePWM module power on  */
	EV3IO_WRITE32 (0x00000003, ev3mem[EV3IO_PSC1], 0x48);	/* Eval the NEXT field         */

	EV3IO_WRITE32 ((EV3IO_READ32 (ev3mem[EV3IO_PSC1], 0x294) | 0x00000003), ev3mem[EV3IO_PSC1], 0x294);	/* Turn PSC on for the eCAP module */
	EV3IO_WRITE32 ((EV3IO_READ32 (ev3mem[EV3IO_PSC1], 0x48) | 0x00000003), ev3mem[EV3IO_PSC1], 0x48);	/* Execute the next step           */

	for (Tmp = 0; Tmp < NO_OF_OUTPUT_PORTS; Tmp++) {
		memset (&Motor[Tmp], 0, sizeof (MOTOR));

		Motor[Tmp].TargetBrake = COAST;
		Motor[Tmp].Pol = 1;
		Motor[Tmp].Direction = FORWARD;
		Motor[Tmp].Type = TYPE_NONE;
		Motor[Tmp].State = IDLE;
		Motor[Tmp].TargetState = UNLIMITED_UNREG;	//default startup state
		Motor[Tmp].Mutex = FALSE;
		Motor[Tmp].BrakeAfter = FALSE;

		CLEARTachoArray (Tmp);
		SETMotorType (Tmp, TYPE_NONE);	//  Motor types can be: TYPE_TACHO, TYPE_NONE, TYPE_MINITACHO
		SETAvgTachoCount (Tmp, 0);	//  At initialisation speed is assumed to be zero
	}

	/* Float the tacho inputs */
	for (Tmp = 0; Tmp < NO_OF_OUTPUT_PORTS; Tmp++) {
		OutputFloat (Tmp, INT);
		OutputFloat (Tmp, DIR);
		SetCoast (Tmp);
	}

	SyncMNos[0] = UNUSED_SYNC_MOTOR;
	SyncMNos[1] = UNUSED_SYNC_MOTOR;

	/* Setup the PWM peripherals */
	printk ("TIMER64P3[TGCR]  = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], TGCR));
	printk ("TIMER64P3[PRD34] = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], PRD34));
	printk ("TIMER64P3[TCR]   = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], TCR));

	SETUPPwmModules;

	printk ("TIMER64P3[TGCR]  = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], TGCR));
	printk ("TIMER64P3[PRD34] = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], PRD34));
	printk ("TIMER64P3[TCR]   = %lx\n", EV3IO_READ32 (ev3mem[EV3IO_TIMER64P3], TCR));

	/* Setup interrupt for the tacho int pins */
// FIXME if you ever want interrupts for IRQs
	SetGpioAnyEdgeIrq (IRQA_PINNO, IntA);
	SetGpioAnyEdgeIrq (IRQB_PINNO, IntB);
	SetGpioAnyEdgeIrq (IRQC_PINNO, IntC);
	SetGpioAnyEdgeIrq (IRQD_PINNO, IntD);
//  }

	return 0;
out_err:
	/* better release the I/O memory regions, or leaves that subsystem in a mess */
	ev3dev_release_mmio_regions ();

	return (Result);
}
/*}}}*/
/*{{{  static void Device1Exit (void)*/
/*
 *	shut-down device 1 (FIXME: use proper kernel methods, not I/O mapping)
 */
static void Device1Exit (void)
{
	hrtimer_cancel (&Device1Timer);
	misc_deregister (&Device1);

#ifdef DEBUG
	printk ("  %s device unregistered\n", DEVICE_NAME);
#endif
	return;
}
/*}}}*/
/*{{{  OLDCODE*/
// static    void  __iomem *GpioBase;
// 
// void      SetGpio(int Pin)
// {
//   int     Tmp = 0;
//   void    __iomem *Reg;
// 
//   if (Pin >= 0)
//   {
//     while ((MuxRegMap[Tmp].Pin != -1) && (MuxRegMap[Tmp].Pin != Pin))
//     {
//       Tmp++;
//     }
//     if (MuxRegMap[Tmp].Pin == Pin)
//     {
//       Reg   =  da8xx_syscfg0_base + 0x120 + (MuxRegMap[Tmp].MuxReg << 2);
// 
//       *(u32*)Reg &=  MuxRegMap[Tmp].Mask;
//       *(u32*)Reg |=  MuxRegMap[Tmp].Mode;
// 
//       if (Pin < NO_OF_GPIOS)
//       {
// #ifdef DEBUG
//         printk("    GP%d_%-2d   0x%08X and 0x%08X or 0x%08X\n",(Pin >> 4),(Pin & 0x0F),(u32)Reg, MuxRegMap[Tmp].Mask, MuxRegMap[Tmp].Mode);
// #endif
//       }
//       else
//       {
// #ifdef DEBUG
//         printk("   OUTPUT FUNCTION 0x%08X and 0x%08X or 0x%08X\n",(u32)Reg, MuxRegMap[Tmp].Mask, MuxRegMap[Tmp].Mode);
// #endif
//       }
//     }
//     else
//     {
//       printk("    GP%d_%-2d Not found (Const no. %d, Tmp = %d)\n",(Pin >> 4),(Pin & 0x0F), Pin, Tmp);
//     }
//   }
// }

/*}}}*/

/*{{{  void InitGpio (void)*/
/*
 *	initialises the various GPIO regions (pins) used by the motors.
 */
void InitGpio (void)
{
	int Port;
	int Pin;
	int ret = 0;
	// unlock
//  REGUnlock;

#warning "Move this to the board level init later!"

	ret = davinci_cfg_reg_list (legoev3_outputA_pins);
	if (ret) {
		pr_warning ("d_pwm: legoev3_outputA_pins setup failed: %d\n", ret);
	}
	ret = davinci_cfg_reg_list (legoev3_outputB_pins);
	if (ret) {
		pr_warning ("d_pwm: legoev3_outputB_pins setup failed: %d\n", ret);
	}
	ret = davinci_cfg_reg_list (legoev3_outputC_pins);
	if (ret) {
		pr_warning ("d_pwm: legoev3_outputC_pins setup failed: %d\n", ret);
	}
	ret = davinci_cfg_reg_list (legoev3_outputD_pins);
	if (ret) {
		pr_warning ("d_pwm: legoev3_outputD_pins setup failed: %d\n", ret);
	}

	for (Port = 0; Port < NO_OF_OUTPUT_PORTS; ++Port) {
#ifdef DEBUG
		printk ("  Output port %d\n", Port + 1);
#endif

		for (Pin = 0; Pin < OUTPUT_PORT_PINS; Pin++) {
//      if ((pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin) >= 0)
			if (legoev3_pwm_gpio[Port][Pin] >= 0) {
				gpio_request (legoev3_pwm_gpio[Port][Pin], "ev3dev_pwm");
				gpio_direction_input (legoev3_pwm_gpio[Port][Pin]);
// 
//        pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].pGpio  =  (struct gpio_controller *__iomem)(GpioBase + ((pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin >> 5) * 0x28) + 0x10);
//        pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Mask   =  (1 << (pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin & 0x1F));
//
//        SetGpio(pOutputPortPin[Hw][(Port * OUTPUT_PORT_PINS) + Pin].Pin);
			}
		}
	}

	// lock
//  REGLock;
}
/*}}}*/
/*{{{  void FreeGpio (void)*/
/*
 *	frees GPIO regions.
 */
void FreeGpio (void)
{
	int Port;
	int Pin;

	for (Port = 0; Port < NO_OF_OUTPUT_PORTS; ++Port) {
		for (Pin = 0; Pin < OUTPUT_PORT_PINS; Pin++) {
			if (legoev3_pwm_gpio[Port][Pin] >= 0) {
				gpio_free (legoev3_pwm_gpio[Port][Pin]);
			}
		}
	}
}
/*}}}*/
/*{{{  void SetGpioAnyEdgeIrq (UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *))*/
/*
 *	sets the any-edge IRQ for a GPIO pin.
 */
void SetGpioAnyEdgeIrq (UBYTE PinNo, irqreturn_t (*IntFuncPtr)(int, void *))
{
	UWORD Status;

	Status = request_irq (gpio_to_irq (PinNo), IntFuncPtr, 0, "PWM_DEVICE", NULL);
	if (Status < 0) {
		printk ("error %d requesting GPIO IRQ %d\n", Status, PinNo);
	}
	irq_set_irq_type (gpio_to_irq (PinNo), IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
//#ifdef DEBUG
	printk (".. done\n");
//#endif
}
/*}}}*/

/*{{{  static irqreturn_t IntA (int irq, void *dev)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 *
 */
/*! \brief    IntA
 *
 *  Tacho A interrupt function
 *
 *  Tacho count is incremented or decremented on both positive
 *  and negative edges of the INT signal.
 *
 *  For each positive and negative edge of the INT tacho signal
 *  a timer is sampled. this is used to calculate the speed later on.
 *
 *  DirChgPtr is implemented for ensuring that there is enough
 *  samples in the same direction to calculate a speed.
 *
 */
static irqreturn_t IntA (int irq, void *dev)
{
	UBYTE TmpPtr;
	ULONG IntAState;
	ULONG DirAState;
	ULONG Timer;

	// Sample all necessary items as fast as possible
	IntAState = READIntA;
	DirAState = READDirA;

	Timer = FREERunning24bittimer;

	TmpPtr = (TachoSamples[0].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES - 1);
	TachoSamples[0].TachoArray[TmpPtr] = Timer;
	TachoSamples[0].ArrayPtr = TmpPtr;

	if ((35 < Motor[0].Speed) || (-35 > Motor[0].Speed)) {
		if (FORWARD == Motor[0].Direction) {
			(Motor[0].IrqTacho)++;
		} else {
			(Motor[0].IrqTacho)--;
		}
		if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
			Motor[0].DirChgPtr++;
		}
	} else {
		if (IntAState) {
			if (DirAState) {
				if (FORWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)++;
				Motor[0].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[0].Direction) {
					TachoSamples[0].TachoArray[TmpPtr] = Timer;
					TachoSamples[0].ArrayPtr = TmpPtr;
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)--;
				Motor[0].Direction = BACKWARD;
			}
		} else {
			if (DirAState) {
				if (BACKWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)--;
				Motor[0].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[0].Direction) {
					if (Motor[0].DirChgPtr < SamplesPerSpeed[0][SAMPLES_ABOVE_SPEED_75]) {
						Motor[0].DirChgPtr++;
					}
				} else {
					Motor[0].DirChgPtr = 0;
				}
				(Motor[0].IrqTacho)++;
				Motor[0].Direction = FORWARD;
			}
		}
	}
	return IRQ_HANDLED;
}

/*}}}*/
/*{{{  static irqreturn_t IntB (int irq, void *dev)*/
/*
 *	Ditto, for 2nd motor.
 */
static irqreturn_t IntB (int irq, void *dev)
{
	UBYTE volatile TmpPtr;
	ULONG volatile IntBState;
	ULONG volatile DirBState;
	ULONG volatile Timer;

	// Sample all necessary items as fast as possible
	IntBState = READIntB;
	DirBState = READDirB;
	Timer = FREERunning24bittimer;

	TmpPtr = (TachoSamples[1].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES - 1);
	TachoSamples[1].TachoArray[TmpPtr] = Timer;
	TachoSamples[1].ArrayPtr = TmpPtr;

	if ((35 < Motor[1].Speed) || (-35 > Motor[1].Speed)) {
		if (FORWARD == Motor[1].Direction) {
			(Motor[1].IrqTacho)++;
		} else {
			(Motor[1].IrqTacho)--;
		}

		if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
			Motor[1].DirChgPtr++;
		}
	} else {
		if (IntBState) {
			if (DirBState) {
				if (FORWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)++;
				Motor[1].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)--;
				Motor[1].Direction = BACKWARD;
			}
		} else {
			if (DirBState) {
				if (BACKWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)--;
				Motor[1].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[1].Direction) {
					if (Motor[1].DirChgPtr < SamplesPerSpeed[1][SAMPLES_ABOVE_SPEED_75]) {
						Motor[1].DirChgPtr++;
					}
				} else {
					Motor[1].DirChgPtr = 0;
				}
				(Motor[1].IrqTacho)++;
				Motor[1].Direction = FORWARD;
			}
		}
	}
	return IRQ_HANDLED;
}
/*}}}*/
/*{{{  static irqreturn_t IntC (int irq, void *dev)*/
/*
 *	Ditto, for 3rd motor.
 */
static irqreturn_t IntC (int irq, void *dev)
{
	UBYTE TmpPtr;
	ULONG IntCState;
	ULONG DirCState;
	ULONG Timer;

	// Sample all necessary items as fast as possible
	IntCState = READIntC;
	DirCState = READDirC;
	Timer = FREERunning24bittimer;

	TmpPtr = (TachoSamples[2].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES - 1);
	TachoSamples[2].TachoArray[TmpPtr] = Timer;
	TachoSamples[2].ArrayPtr = TmpPtr;

	if ((35 < Motor[2].Speed) || (-35 > Motor[2].Speed)) {
		if (FORWARD == Motor[2].Direction) {
			(Motor[2].IrqTacho)++;
		} else {
			(Motor[2].IrqTacho)--;
		}
		if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
			Motor[2].DirChgPtr++;
		}
	} else {
		if (IntCState) {
			if (DirCState) {
				if (FORWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)++;
				Motor[2].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)--;
				Motor[2].Direction = BACKWARD;
			}
		} else {
			if (DirCState) {
				if (BACKWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)--;
				Motor[2].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[2].Direction) {
					if (Motor[2].DirChgPtr < SamplesPerSpeed[2][SAMPLES_ABOVE_SPEED_75]) {
						Motor[2].DirChgPtr++;
					}
				} else {
					Motor[2].DirChgPtr = 0;
				}
				(Motor[2].IrqTacho)++;
				Motor[2].Direction = FORWARD;
			}
		}
	}
	return IRQ_HANDLED;
}

/*}}}*/
/*{{{  static irqreturn_t IntD (int irq, void *dev)*/
/*
 *	Ditto, for 4th motor.
 */
static irqreturn_t IntD (int irq, void *dev)
{
	UBYTE TmpPtr;
	ULONG IntDState;
	ULONG DirDState;
	ULONG Timer;

	// Sample all necessary items as fast as possible
	IntDState = READIntD;
	DirDState = READDirD;
	Timer = FREERunning24bittimer;

	TmpPtr = (TachoSamples[3].ArrayPtr + 1) & (NO_OF_TACHO_SAMPLES - 1);
	TachoSamples[3].TachoArray[TmpPtr] = Timer;
	TachoSamples[3].ArrayPtr = TmpPtr;

	if ((35 < Motor[3].Speed) || (-35 > Motor[3].Speed)) {
		if (FORWARD == Motor[3].Direction) {
			(Motor[3].IrqTacho)++;
		} else {
			(Motor[3].IrqTacho)--;
		}
		if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
			Motor[3].DirChgPtr++;
		}
	} else {
		if (IntDState) {
			if (DirDState) {
				if (FORWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)++;
				Motor[3].Direction = FORWARD;
			} else {
				if (BACKWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)--;
				Motor[3].Direction = BACKWARD;
			}
		} else {
			if (DirDState) {
				if (BACKWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)--;
				Motor[3].Direction = BACKWARD;
			} else {
				if (FORWARD == Motor[3].Direction) {
					if (Motor[3].DirChgPtr < SamplesPerSpeed[3][SAMPLES_ABOVE_SPEED_75]) {
						Motor[3].DirChgPtr++;
					}
				} else {
					Motor[3].DirChgPtr = 0;
				}
				(Motor[3].IrqTacho)++;
				Motor[3].Direction = FORWARD;
			}
		}
	}
	return IRQ_HANDLED;
}
/*}}}*/
/*{{{  UBYTE dCalculateSpeed (UBYTE No, SBYTE *pSpeed)*/
/*! \page PwmModule
 *
 *  <hr size="1"/>
 *  <b>     write </b>
 */
/*! \brief    dCalculateSpeed
 *
 *  - Calculates the actual speed for a motor
 *
 *  - Returns TRUE when a new speed has been calculated, FALSE if otherwise
 *
 *  - Time is sampled every edge on the tacho
 *      - Timer used is 64bit timer plus (P3) module (dual 32bit un-chained mode)
 *      - 64bit timer is running 33Mhz (24Mhz (Osc) * 22 (Multiplier) / 2 (Post divider) / 2 (DIV2)) / 4 (T64 prescaler)
 *      - When reading the timer it is divided by 256 => timer is a factor 256 slower
 *
 *
 *  - Tacho counter is updated on every edge of the tacho INTx pin signal
 *  - Time capture is updated on every edge of the tacho INTx pin signal
 *
 *
 *  - Speed is calculated from the following parameters
 *
 *      - Time is measured edge to edge of the tacho interrupt pin. Average of time is always minimum 2 pulses
 *        (1 high + 1 low period or 1 low + 1 high period) because the duty of the high and low period of the
 *        tacho pulses are not always 50%.
 *        - Average of the large motor
 *          - Above speed 80 it is:          64 samples
 *          - Between speed 60 - 80 it is:   32 samples
 *          - Between speed 40 - 60 it is:   16 samples
 *          - below speed 40 it is:           4 samples
 *        - Average of the medium motor
 *          - Above speed 80 it is:          16 samples
 *          - Between speed 60 - 80 it is:    8 samples
 *          - Between speed 40 - 60 it is:    4 samples
 *          - below speed 40 it is:           2 sample
 *
 *      - Number of samples is always determined based on 1 sample meaning 1 low period nor 1 high period,
 *        this is to enable fast adoption to changes in speed. Medium motor has the critical timing because
 *        it can change speed and direction very fast.
 *
 *      - Large Motor
 *        - Maximum speed of the Large motor is approximately 2mS per tacho pulse (low + high period)
 *          resulting in minimum timer value of: 2mS / (1/(33MHz / 256)) = 256 T64 timer ticks.
 *          Because 1 sample is based on only half a period minimum speed is 256/2 = 128.
 *        - Minimum speed of the large motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 256 * 100 = 25600 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 25600/2 = 12800.
 *
 *
 *      - Medium Motor
 *        - Maximum speed of the medium motor is approximately 1,25mS per tacho pulse (low + high period)
 *          resulting in minimum timer value og: 1,25mS / (1/(33MHz / 256)) = 162 (approximately)
 *          Because 1 sample is based on only half a period minimum speed is 162/2 = 81.
 *        - Minimum speed of the medium motor is a factor of 100 less than max. speed
 *          max. speed timer ticks * 100 => 162 * 100 = 16200 T64 timer ticks
 *          Because 1 sample is based on only half a period minimum speed is 16200/2 = 8100.
 *
 *      - Actual speed is then calculated as:
 *        - Medium motor:
 *          8100 * number of samples / actual time elapsed for number of samples
 *        - Large motor:
 *          12800 * number of samples / actual time elapsed for number of samples
 *
 *
 *  - Parameters:
 *    - Input:
 *      - No        : Motor output number
 *      - *pSpeed   : Pointer to the speed value
 *
 *    - Output:
 *      - Status    : Indication of new speed available or not
 *
 *
 *  - Tacho pulse examples:
 *
 *
 *    - Normal
 *
 *      ----       ------       ------       ------
 *          |     |      |     |      |     |      |
 *          |     |      |     |      |     |      |
 *          -------      -------      -------      --- DIRx signal
 *
 *
 *         ----       ------       ------       ------ INTx signal
 *             |     |      |     |      |     |
 *             |     |      |     |      |     |
 *             -------      -------      -------
 *
 *             ^     ^      ^     ^      ^     ^
 *             |     |      |     |      |     |
 *             |   Timer    |   Timer    |   Timer
 *             |     +      |     +      |     +
 *             |  Counter   |  Counter   |  Counter
 *             |            |            |
 *           Timer        Timer        Timer
 *             +            +            +
 *          Counter      Counter      Counter
 *
 *
 *
 *    - Direction change
 *
 *      DirChgPtr variable is used to indicate how many timer samples have been sampled
 *      since direction has been changed. DirChgPtr is set to 0 when tacho interrupt detects
 *      direction change and then it is counted up for every timer sample. So when DirChgPtr
 *      has the value of 2 then there must be 2 timer samples in the the same direction
 *      available.
 *
 *      ----       ------       ------       ------       ---
 *          |     |      |     |      |     |      |     |
 *          |     |      |     |      |     |      |     |
 *          -------      -------      -------      -------   DIRx signal
 *
 *
 *       ------       -------------       ------       ------INTx signal
 *             |     |             |     |      |     |
 *             |     |             |     |      |     |
 *             -------             -------      -------
 *
 *             ^     ^             ^     ^      ^     ^
 *             |     |             |     |      |     |
 *           Timer   |           Timer   |    Timer   |
 *             +     |             +     |      +     |
 *          Counter  |          Counter  |   Counter  |
 *             +     |             +     |      +     |
 *       DirChgPtr++ |       DirChgPtr=0 |DirChgPtr++ |
 *                 Timer               Timer        Timer
 *                   +                   +            +
 *                Counter             Counter      Counter
 *                   +                   +            +
 *               DirChgPtr++         DirChgPtr++  DirChgPtr++
 *
 *
 *
 *
 *      ----       ------             ------        ----
 *          |     |      |           |      |      |
 *          |     |      |           |      |      |
 *          -------      -------------       -------          DIRx signal
 *
 *
 *       ------       ------       ------       ------        INTx signal
 *             |     |      |     |      |     |      |
 *             |     |      |     |      |     |      |
 *             -------      -------      -------       ----
 *
 *             ^     ^      ^     ^      ^     ^      ^
 *             |     |      |     |      |     |      |
 *           Timer   |    Timer   |    Timer   |    Timer
 *             +     |      +     |      +     |      +
 *          Counter  |   Counter  |   Counter  |   Counter
 *             +     |      +     |      +     |      +
 *        DirChgPtr++| DirChgPtr++| DirChgPtr++| DirChgPtr++
 *                 Timer        Timer        Timer
 *                   +            +            +
 *                Counter      Counter      Counter
 *                   +            +            +
 *               DirChgPtr++  DirChgPtr=0  DirChgPtr++
 *
 *
 *
 */

UBYTE dCalculateSpeed (UBYTE No, SBYTE *pSpeed)
{

	ULONG Tmp1, Tmp2;
	ULONG Diff;
	UBYTE Ptr, Status;
	SWORD Speed;

	Status = FALSE;

	Ptr = TachoSamples[No].ArrayPtr;

	if (Motor[No].DirChgPtr >= 1) {
		Diff = (((TachoSamples[No].TachoArray[Ptr]) - (TachoSamples[No].TachoArray[((Ptr - 1) & (NO_OF_TACHO_SAMPLES - 1))])) & 0x00FFFFFF);
		if (Diff) {
			SETAvgTachoCount (No, (ULONG) ((CountsPerPulse[No] / Diff) & 0x00FFFFFF));
		} else {
			SETAvgTachoCount (No, (ULONG) 1);
		}
	}

	Speed = *pSpeed;	// Maintain old speed if not changed
	Tmp1 = TachoSamples[No].TachoArray[Ptr];
	Tmp2 = TachoSamples[No].TachoArray[((Ptr - AVG_TACHO_COUNTS[No]) & (NO_OF_TACHO_SAMPLES - 1))];

	if ((Ptr != TachoSamples[No].ArrayPtrOld) && (Motor[No].DirChgPtr >= AVG_TACHO_COUNTS[No])) {

		Status = TRUE;
		TimeOutSpeed0[No] = Tmp1;
		TachoSamples[No].ArrayPtrOld = Ptr;

		Diff = ((Tmp1 - Tmp2) & 0x00FFFFFF);
		if (Diff) {
			Speed = (SWORD) ((ULONG) (AVG_COUNTS[No]) / (ULONG) Diff);
		} else {
			// Safety check
			Speed = 1;
		}

		if (Speed > 100) {
			Speed = 100;
		}

		if (BACKWARD == Motor[No].Direction) {
			Speed = 0 - Speed;
		}
	} else {
		// No new Values check for speed 0
		if ((CountsPerPulse[No]) < ((FREERunning24bittimer - TimeOutSpeed0[No]) & 0x00FFFFFF)) {
			TimeOutSpeed0[No] = FREERunning24bittimer;
			Speed = 0;
			Motor[No].DirChgPtr = 0;
			Status = TRUE;
		}
	}

	*pSpeed = (SBYTE) Speed;

	return (Status);
}

/*}}}*/

// MODULE *********************************************************************


/*{{{  static int ev3dev_pwm_doinit (void)*/
/*
 *	initialises the PWM module for driving motors on the EV3.
 *	return 0 on success, non-zero on failure.
 */
static int ev3dev_pwm_doinit (void)
{
	int ret = 0;

#if 0
	ehrpwm_1_0 = pwm_request_byname (EV3DEV_PWM_EHRPWM1A_NAME, "ev3dev_pwm");
	if (IS_ERR (ehrpwm_1_0)) {
		printk (KERN_ERR "ev3dev_pwm: failed to find PWM device %s\n", EV3DEV_PWM_EHRPWM1A_NAME);
		ret = PTR_ERR (ehrpwm_1_0);
		goto out_err0;
	}

	ehrpwm_1_1 = pwm_request_byname (EV3DEV_PWM_EHRPWM1B_NAME, "ev3dev_pwm");
	if (IS_ERR (ehrpwm_1_1)) {
		printk (KERN_ERR "ev3dev_pwm: failed to find PWM device %s\n", EV3DEV_PWM_EHRPWM1B_NAME);
		ret = PTR_ERR (ehrpwm_1_1);
		goto out_err1;
	}
#endif

	/* Motor PWM outputs has been switched in EP2 MA<->MB and MC<->MD */
	SetDuty[0] = SetDutyMB;
	SetDuty[1] = SetDutyMA;
	SetDuty[2] = SetDutyMD;
	SetDuty[3] = SetDutyMC;

	ev3mem = ev3dev_get_mmio_regions ();
	if (!ev3mem) {
		printk (KERN_ERR "ev3dev_pwm: failed to get mapped I/O regions from ev3dev\n");
		ret = -EIO;
		goto out_err2;
	}

	InitGpio ();

	Device1Init ();

	/* clear state (used to be in Device2Init) */
	memset (MotorData, 0, sizeof (MotorData));

	/* Setup timer irq */
	Device1Time = ktime_set (0, SOFT_TIMER_SETUP);
	hrtimer_init (&Device1Timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	Device1Timer.function = Device1TimerInterrupt1;
	hrtimer_start (&Device1Timer, Device1Time, HRTIMER_MODE_REL);

	return ret;

out_err2:
#if 0
	pwm_release (ehrpwm_1_1);
	ehrpwm_1_1 = NULL;
out_err1:
	pwm_release (ehrpwm_1_0);
	ehrpwm_1_0 = NULL;
out_err0:
#endif
	return ret;
}

/*}}}*/
/*{{{  static void ev3dev_pwm_doexit (void)*/
/*
 *	tidies up resources for module exit.
 */
static void ev3dev_pwm_doexit (void)
{
	printk ("ev3dev_pwm exit started\n");
	STOPPwm;
// FIXME if you want to unload irqs when module is unloaded
	free_irq (gpio_to_irq (IRQA_PINNO), NULL);
	free_irq (gpio_to_irq (IRQB_PINNO), NULL);
	free_irq (gpio_to_irq (IRQC_PINNO), NULL);
	free_irq (gpio_to_irq (IRQD_PINNO), NULL);

	Device1Exit ();
// 1  procfile_exit();

	FreeGpio ();

//  iounmap(GpioBase);
#if 0
	pwm_release (ehrpwm_1_0);
	pwm_release (ehrpwm_1_1);
	ehrpwm_1_0 = NULL;
	ehrpwm_1_1 = NULL;
#endif

	if (ev3mem) {
		ev3dev_release_mmio_regions ();
		ev3mem = NULL;
	}


}
/*}}}*/

// -----------------------------------------------------------------------

#warning "The value 16 is the max number of bytes in a MOTORDATA, STEPPOWER, TIMEPOWER, STEPSPEED, STEPSYNC, or TIMESYNC command"
#warning "See if we can se the buffer size programmatically!"

/*{{{  struct ev3dev_pwm_command_attribute and macro*/
struct ev3dev_pwm_command_attribute {
	struct device_attribute dev_attr;
	void *var;
	signed char buf[16];
};

#define to_ev3dev_pwm_command_attr(x) container_of(x, struct ev3dev_pwm_command_attribute, dev_attr)

/*}}}*/

/*{{{  static ssize_t show_pwm_command (struct device *dev, struct device_attribute *attr, char *buf)*/
/*
 *	shows the PWM command buffer for a motor.
 */
static ssize_t show_pwm_command (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ev3dev_pwm_command_attribute *ea = to_ev3dev_pwm_command_attr (attr);

	return snprintf (buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", ea->buf[0]
			 , ea->buf[1]
			 , ea->buf[2]
			 , ea->buf[3]
			 , ea->buf[4]
			 , ea->buf[5]
			 , ea->buf[6]
			 , ea->buf[7]
			 , ea->buf[8]
			 , ea->buf[9]
			 , ea->buf[10]
			 , ea->buf[11]
			 , ea->buf[12]
			 , ea->buf[13]
			 , ea->buf[14]
			 , ea->buf[15]);
}
/*}}}*/
/*{{{  static ssize_t store_pwm_command (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)*/
/*
 *	stores a string of command bytes (parsed with strtol).
 */
static ssize_t store_pwm_command (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	long val;

	struct ev3dev_pwm_command_attribute *ea = to_ev3dev_pwm_command_attr (attr);

	const char *start = buf;
	char *end = (char *)buf;
	const char *last = buf + count;

	memset (ea->buf, 0, 16);

	for (i = 0; i < 16; i++) {

		start = skip_spaces (end);

		if (last <= start) {
			break;
		}

		val = simple_strtol (start, &end, 0);

		if (end == start) {
			return -EINVAL;
		}

		ea->buf[i] = val & 0xFF;
	}

	pwm_command_handler ((const char *) (ea->buf));

	return (count);
}
/*}}}*/

#define EV3DEV_PWM_COMMAND_ATTR(_name, _fname, _mode, _var ) \
        struct ev3dev_pwm_command_attribute dev_attr_##_name =   \
                { __ATTR(_fname, _mode, show_pwm_command, store_pwm_command), .var = &(_var) }


// -----------------------------------------------------------------------

/*{{{  global vars*/
static struct platform_device *ev3dev_pwm_motor[4];

static unsigned char *motor_name[4] = { "motorA", "motorB", "motorC", "motorD" };

/*}}}*/
/*{{{  motor_data[..] struct array: holds values seen in sysfs*/
static struct {
	int enable;
	int type;
	int command;
	int power;
	int speed;
	int polarity;
	int counts;
	int time;
	int sync;
} motor_data[4];

/*}}}*/

#warning "Add #defines to limit min/max range for these attributes ie TYPE_TACHO to TYPE_NEWTACHO"

/*{{{  constant attributes*/
static EV3DEV_MINMAX_ATTR (enableA, enable, 0666, motor_data[0].enable, 0, 1);
static EV3DEV_MINMAX_ATTR (typeA, type, 0666, motor_data[0].type, 7, 9);
static EV3DEV_MINMAX_ATTR (powerA, power, 0666, motor_data[0].power, -100, 100);
static EV3DEV_MINMAX_ATTR (speedA, speed, 0666, motor_data[0].speed, -100, 100);
static EV3DEV_MINMAX_ATTR (polarityA, polarity, 0666, motor_data[0].polarity, 0, 1);
static EV3DEV_MINMAX_ATTR (countsA, counts, 0666, motor_data[0].counts, INT_MIN, INT_MAX);
static EV3DEV_MINMAX_ATTR (timeA, time, 0666, motor_data[0].time, 0, INT_MAX);
static EV3DEV_MINMAX_ATTR (syncA, sync, 0666, motor_data[0].sync, 0, 0);

static EV3DEV_MINMAX_ATTR (enableB, enable, 0666, motor_data[1].enable, 0, 1);
static EV3DEV_MINMAX_ATTR (typeB, type, 0666, motor_data[1].type, 7, 9);
static EV3DEV_MINMAX_ATTR (powerB, power, 0666, motor_data[1].power, -100, 100);
static EV3DEV_MINMAX_ATTR (speedB, speed, 0666, motor_data[1].speed, -100, 100);
static EV3DEV_MINMAX_ATTR (polarityB, polarity, 0666, motor_data[1].polarity, 0, 1);
static EV3DEV_MINMAX_ATTR (countsB, counts, 0666, motor_data[1].counts, INT_MIN, INT_MAX);
static EV3DEV_MINMAX_ATTR (timeB, time, 0666, motor_data[1].time, 0, INT_MAX);
static EV3DEV_MINMAX_ATTR (syncB, sync, 0666, motor_data[1].sync, 0, 0);

static EV3DEV_MINMAX_ATTR (enableC, enable, 0666, motor_data[2].enable, 0, 1);
static EV3DEV_MINMAX_ATTR (typeC, type, 0666, motor_data[2].type, 7, 9);
static EV3DEV_MINMAX_ATTR (powerC, power, 0666, motor_data[2].power, -100, 100);
static EV3DEV_MINMAX_ATTR (speedC, speed, 0666, motor_data[2].speed, -100, 100);
static EV3DEV_MINMAX_ATTR (polarityC, polarity, 0666, motor_data[2].polarity, 0, 1);
static EV3DEV_MINMAX_ATTR (countsC, counts, 0666, motor_data[2].counts, INT_MIN, INT_MAX);
static EV3DEV_MINMAX_ATTR (timeC, time, 0666, motor_data[2].time, 0, INT_MAX);
static EV3DEV_MINMAX_ATTR (syncC, sync, 0666, motor_data[2].sync, 0, 0);

static EV3DEV_MINMAX_ATTR (enableD, enable, 0666, motor_data[3].enable, 0, 1);
static EV3DEV_MINMAX_ATTR (typeD, type, 0666, motor_data[3].type, 7, 9);
static EV3DEV_MINMAX_ATTR (powerD, power, 0666, motor_data[3].power, -100, 100);
static EV3DEV_MINMAX_ATTR (speedD, speed, 0666, motor_data[3].speed, -100, 100);
static EV3DEV_MINMAX_ATTR (polarityD, polarity, 0666, motor_data[3].polarity, 0, 1);
static EV3DEV_MINMAX_ATTR (countsD, counts, 0666, motor_data[3].counts, INT_MIN, INT_MAX);
static EV3DEV_MINMAX_ATTR (timeD, time, 0666, motor_data[3].time, 0, INT_MAX);
static EV3DEV_MINMAX_ATTR (syncD, sync, 0666, motor_data[3].sync, 0, 0);

/*}}}*/
/*{{{  command attributes*/
static EV3DEV_PWM_COMMAND_ATTR (commandA, command, 0666, motor_data[0].command);
static EV3DEV_PWM_COMMAND_ATTR (commandB, command, 0666, motor_data[1].command);
static EV3DEV_PWM_COMMAND_ATTR (commandC, command, 0666, motor_data[2].command);
static EV3DEV_PWM_COMMAND_ATTR (commandD, command, 0666, motor_data[3].command);

/*}}}*/
/*{{{  more device attribute stuff*/
static struct attribute *ev3dev_pwm_motorA_attrs[] = {
	&dev_attr_enableA.dev_attr.attr,
	&dev_attr_typeA.dev_attr.attr,
	&dev_attr_commandA.dev_attr.attr,
	&dev_attr_powerA.dev_attr.attr,
	&dev_attr_speedA.dev_attr.attr,
	&dev_attr_polarityA.dev_attr.attr,
	&dev_attr_countsA.dev_attr.attr,
	&dev_attr_timeA.dev_attr.attr,
	&dev_attr_syncA.dev_attr.attr,
	NULL,
};

static struct attribute *ev3dev_pwm_motorB_attrs[] = {
	&dev_attr_enableB.dev_attr.attr,
	&dev_attr_typeB.dev_attr.attr,
	&dev_attr_commandB.dev_attr.attr,
	&dev_attr_powerB.dev_attr.attr,
	&dev_attr_speedB.dev_attr.attr,
	&dev_attr_polarityB.dev_attr.attr,
	&dev_attr_countsB.dev_attr.attr,
	&dev_attr_timeB.dev_attr.attr,
	&dev_attr_syncB.dev_attr.attr,
	NULL,
};

static struct attribute *ev3dev_pwm_motorC_attrs[] = {
	&dev_attr_enableC.dev_attr.attr,
	&dev_attr_typeC.dev_attr.attr,
	&dev_attr_commandC.dev_attr.attr,
	&dev_attr_powerC.dev_attr.attr,
	&dev_attr_speedC.dev_attr.attr,
	&dev_attr_polarityC.dev_attr.attr,
	&dev_attr_countsC.dev_attr.attr,
	&dev_attr_timeC.dev_attr.attr,
	&dev_attr_syncC.dev_attr.attr,
	NULL,
};

static struct attribute *ev3dev_pwm_motorD_attrs[] = {
	&dev_attr_enableD.dev_attr.attr,
	&dev_attr_typeD.dev_attr.attr,
	&dev_attr_commandD.dev_attr.attr,
	&dev_attr_powerD.dev_attr.attr,
	&dev_attr_speedD.dev_attr.attr,
	&dev_attr_polarityD.dev_attr.attr,
	&dev_attr_countsD.dev_attr.attr,
	&dev_attr_timeD.dev_attr.attr,
	&dev_attr_syncD.dev_attr.attr,
	NULL
};

static const struct attribute_group ev3dev_pwm_motorA_attr_group = {
	.attrs = ev3dev_pwm_motorA_attrs,
};

static const struct attribute_group ev3dev_pwm_motorB_attr_group = {
	.attrs = ev3dev_pwm_motorB_attrs,
};

static const struct attribute_group ev3dev_pwm_motorC_attr_group = {
	.attrs = ev3dev_pwm_motorC_attrs,
};

static const struct attribute_group ev3dev_pwm_motorD_attr_group = {
	.attrs = ev3dev_pwm_motorD_attrs,
};

static const struct attribute_group *ev3dev_pwm_motor_attr_groups[] = {
	&ev3dev_pwm_motorA_attr_group,
	&ev3dev_pwm_motorB_attr_group,
	&ev3dev_pwm_motorC_attr_group,
	&ev3dev_pwm_motorD_attr_group,
	NULL,
};
/*}}}*/
/*{{{  definitions for EV3 and PWM platform devices*/
extern struct platform_device *ev3dev;
static struct platform_device *pwm;

/*}}}*/

/*{{{  static int ev3dev_pwm_init (void)*/
/*
 *	initialises the EV3 PWM module (first entry-point).
 */
static int ev3dev_pwm_init (void)
{
	int ret;
	int i;

	if (!ev3dev) {
		printk ("ev3dev node is not yet registered, load the ev3dev module\n");
		ret = -ENODEV;
		goto err0;
	}

	ret = -ENOMEM;

	pwm = platform_device_alloc ("pwm", -1);
	if (!pwm) {
		printk (KERN_CRIT "failed to allocate ev3dev/pwm device\n");
		goto err1;
	}

	printk ("  pwm_init pwm allocated  %08x\n", (unsigned int) (pwm));

	pwm->dev.parent = &ev3dev->dev;

	ret = platform_device_add (pwm);
	if (ret) {
		printk ("failed to register pwm device\n");
		goto err4;
	}

	for (i = 0; i < 4; ++i) {
		ev3dev_pwm_motor[i] = platform_device_alloc (motor_name[i], -1);

		if (!ev3dev_pwm_motor[i]) {
			printk ("failed to allocate motor device\n");
			goto err5;
		}

	}

	for (i = 0; i < 4; ++i) {
		ev3dev_pwm_motor[i]->dev.parent = &pwm->dev;

		ret = platform_device_add (ev3dev_pwm_motor[i]);

		if (ret) {
			printk ("failed to add pwm/motor device\n");
			goto err6;
		}
	}

	for (i = 0; i < 4; ++i) {
		ret = sysfs_create_group (&ev3dev_pwm_motor[i]->dev.kobj, ev3dev_pwm_motor_attr_groups[i]);
		if (ret) {
			printk ("failed to add pwm/motor attributes\n");
			goto err7;
		}
	}

	ret = ev3dev_pwm_doinit ();
	if (ret) {
		printk (KERN_ERR "ev3dev_pwm: failed to initialise module specific code\n");
		goto err8;
	}


	return 0;
err8:
	i = 4;

err7:
	while (--i >= 0) {
		sysfs_remove_group (&ev3dev_pwm_motor[i]->dev.kobj, ev3dev_pwm_motor_attr_groups[i]);
	}

	i = 4;			// Make sure we unroll the device adds when we fall through!

err6:
	while (--i >= 0) {
		platform_device_del (ev3dev_pwm_motor[i]);
	}

	i = 4;			// Make sure we unroll the allocates when we fall through!

err5:
	while (--i >= 0) {
		platform_device_put (ev3dev_pwm_motor[i]);
	}

err4:
	platform_device_put (pwm);

err1:
err0:
	return ret;
}
/*}}}*/
/*{{{  static void ev3dev_pwm_exit (void)*/
/*
 *	called when shutting-down the device (to clean-up).
 */
static void ev3dev_pwm_exit (void)
{
	int i;

	ev3dev_pwm_doexit ();

	for (i = 0; i < 4; ++i) {
		sysfs_remove_group (&ev3dev_pwm_motor[i]->dev.kobj, ev3dev_pwm_motor_attr_groups[i]);
		platform_device_del (ev3dev_pwm_motor[i]);
		platform_device_put (ev3dev_pwm_motor[i]);
	}

	platform_device_del (pwm);
	platform_device_put (pwm);
}
/*}}}*/

module_init (ev3dev_pwm_init);
module_exit (ev3dev_pwm_exit);


MODULE_AUTHOR ("Ralph Hempel <rhempel@hempeldesigngroup.com>");
MODULE_DESCRIPTION ("Driver for LEGO MINDSTORMS EV3 pwm Device");
MODULE_LICENSE ("GPL");


