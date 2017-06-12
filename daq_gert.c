/*
 *     comedi/drivers/daq_gert.c
 * 
 *	Also for the TI ADS1220 SD ADC converter chip (and MCP3911 later) for low voltage sensing and
 *	solar panel panel light detection. +- 2.048, 1.024 and 0.512 voltage ranges @ 20 bits of usable resolution
 *	ADC is in single-shot conversion mode @20SPS, PGA disabled and gain from 1, 2 and 4 in differential
 *	signal detection mode, 50/60Hz rejection enabled. 500kHz SPI clock with direct RPi2 connection
 *	Analog +- 2.5VDC from Zener regulators for the bipolar input stage with external 2.5VDC Zener input
 *	signal protection.
 * 
 *	LEDs: + supply, - supply, DRDY__ LOW
 * 
 *	Board jumpers J1 Left to Right
 *	1 3.3VDC digital supply for direct connection to RPi2 board
 *	2 5.0VDC digital supply for optical interconnects for 5VDC or 3.3VDC SPI interfaces
 *	3 Enable 5VDC power
 * 
 * ADS8330 SD chip driver
 *
 * DIP8 Pins for MCP3002 header
 * 25K22	RPi DIP8 header		IDC 10 pin connector header	ADS1220
 * Pin 21   RB0	SPI Chip-Select	Pin 1		8	CS		2
 * Pin 22   RB1	SPI Clock	Pin 7		7	SCK		1
 * Pin 23   RB2	SPI Data In	Pin 5		6	SDI		16
 * Pin 24   RB3	SPI Data Out	Pin 6		5	SDO		15
 * Pin 8    Vss			Pin 4		10	GND		4
 * Pin 20   Vdd			Pin 8		9	Vdd 3.3/5.0VDC	13
 * Pin 2    RA0	ANA0		Pin 2		1	nc
 * Pin 3    RA1	ANA1		Pin 3		2	nc
 * 
 *	PIC 8722 SPI slave connect
 *	TRISDbits.TRISD6 = 1; // SCK SSP2 pins in SLAVE mode
 *	TRISDbits.TRISD5 = 1; // SDI
 *	TRISDbits.TRISD4 = 0; // SDO
 *	TRISDbits.TRISD7 = 1; // SS2
 *
 *     COMEDI - Linux Control and Measurement Device Interface
 *     Copyright (C) 1998 David A. Schleef <ds@schleef.org>
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 2 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program; if not, write to the Free Software
 *     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * TODO: Refactor sample put get code to reduce the amount of build up/down time
 * 
Driver: "experimental" daq_gert in progress ... 
 * for 4.4+ kernels with device-tree enabled
 * see README.md for install instructions
 * 
Description: GERTBOARD daq_gert
Author: Fred Brooks <spam@sma2.rain.com>
 * 
Most of the actual GPIO setup code was copied from
 * 
WiringPI 
 *      https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 * Driver for Broadcom BCM2835 SPI Controller masters
 *
 * 

Devices: [] GERTBOARD (daq_gert)
Status: inprogress (DIO 95%) (AI 95%) AO (96%) (My code cleanup 95%)
Updated: Sept 2016 12:07:20 +0000

The DAQ-GERT appears in Comedi as a  digital I/O subdevice (0) with
17 or 21 or 30 channels, 
a analog input subdevice (1) with a Mux for bipolar input voltages
2 differential-ended channels: (0) inputs 0-1, (1) inputs 2-3
2 single-ended channels: (2) (3) for inputs 2 or 3 
a internal short of inputs for a offset reading: (4) with the ADC1220 adc, OR
a analog input subdevice (1) with 2 single-ended channels with onboard adc, OR
a analog input subdevice (1) with single-ended channels set by the SPI slave device
and a analog output subdevice(2) with 2 channels with onboard dac
 * 
 * Caveats:
 * 

Digital:  The comedi channel 0 corresponds to the GPIO WPi table order
channel numbers [0..7] will be outputs, [8..16/20/29] will be inputs
 * 0/2
 * 1/3
 * 4
 * 7    SPI CE1
 * 8    SPI CE0
 * 9    SPI SO
 * 10   SPI SI
 * 11   SPI CLK
 * 14   UART
 * 15   UART
 * 17
 * 18   PWM
 * 21/27
 * 22
 * 23
 * 24
 * 25
 * 
 The BCM2835 has 54 GPIO pins.
      BCM2835 data sheet, Page 90 onwards.
      There are 6 control registers, each control the functions of a block
      of 10 pins.
      Each control register has 10 sets of 3 bits per GPIO pin:

      000 = GPIO Pin X is an input
      001 = GPIO Pin X is an output
      100 = GPIO Pin X takes alternate function 0
      101 = GPIO Pin X takes alternate function 1
      110 = GPIO Pin X takes alternate function 2
      111 = GPIO Pin X takes alternate function 3
      011 = GPIO Pin X takes alternate function 4
      010 = GPIO Pin X takes alternate function 5

 So the 3 bits for port X are:
      X / 10 + ((X % 10) * 3)

Digital direction configuration: [0..1] are input only due to pullups,
 * all other ports can be input or outputs

Analog: The type and resolution of the onboard ADC/DAC chips are set
by the module option variable daqgert_conf in the /etc/modprobe.d directory
 * options daq_gert daqgert_conf=1
 * 
0 = Factory Gertboard configuratin of MCP3002 ADC and MCP4802 ADC: 10bit in/8bit out
1 = MCP3202 ADC and MCP4822 DAC: 12bit in/12bit out 
2 = MCP3002 ADC and MCP4822 DAC: 10bit in/12bit out
3 = MCP3202 ADC and MCP4802 DAC: 12bit in/8bit out
4 = ADS1220 ADC and MCP4822 DAC: 24bit in/12bit out
14 = ADS1220 ADC and MCP4802 DAC: 24bit in/8bit out
5 = force PIC slave P8722 mode
6 = force PIC slave P25k22 mode
16 = ADS8330 ADC and MCP4822 DAC: 16bit in/12bit out
17 = ADS8330 ADC and MCP4802 DAC: 16bit in/8bit out
99 = special ADC test device and MCP4822 DAC
 * 
 * Module parameters are found in the /sys/modules/daq_gert/parameters directory
 * 
 * The input  range is 0 to 1023/4095 for 0.0 to 3.3(Vdd) onboard devices, 2.048 volts/Vdd for PIC slaves 
 * or +-0.512, +-1.024, +-2.048 for the ADS1220 device with usable range in double %2.6fV format for output
 * The output range is 0 to 4095 for 0.0 to 2.048 onboard devices (output resolution depends on the device)
 * In the async command mode transfers can be handled in HUNK mode by creating a SPI message
 * of many conversion sequences into one message, this allows for close to native driver wire-speed 
 * HUNK_LEN data samples into the Comedi read buffer with a special mix_mode for 
 * sampling both ADC devices in an alt sequence for programs like xoscope at 
 * full speed (48828 ns per conversion over a 10 second period). 
 * comedi analog output testing command: ./ao_waveform -v -c 0 -f /dev/comedi0_subd2 -n1
 * The transfer array is currently static but can easily be made into
 * a config size parameter runtime value if needed with kmalloc for the required space

 *  PIC Slave Info:
 * 
 * bit 7 high for commands sent from the MASTER
 * bit 6 0 send lower or 1 send upper byte ADC result first
 * bits 3..0 port address
 * all zeros sent to the PIC slave will return 8 bits from the ADC buffer
 *
 * bit 7 low  for config data sent in CMD_DUMMY per uC type
 * bits 6 config bit code always 1
 * bit	5 0=ADC ref VDD, 1=ADC rec FVR=2.048
 * bit  4 0=10bit adc, 1=12bit adc
 * bits 3..0 number of ADC channels
 * 
 */

#include "../comedidev.h" 
#include <linux/interrupt.h> 
#include <linux/kernel.h>
#include <linux/module.h> 
#include <linux/kthread.h> 
#include <linux/sched.h>
#include <linux/spi/spi.h> 
#include <linux/delay.h> 
#include <linux/device.h> 
#include <linux/timer.h> 
#include <linux/list.h>  
#include "comedi_8254.h"  
//#include <mach/platform.h> /* for GPIO_BASE and ST_BASE */
//#define PERI_BASE   0x20000000
#define PERI_BASE   0x3F000000
#define BCM2708_PERI_BASE        PERI_BASE

#define ST_BASE                  (BCM2708_PERI_BASE + 0x3000) 
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO */
#define CHECKMARK 0x1957

/* Command Definitions */
#define ADS8330_CMR_DEFAULT 0b1111000000000000ul /* reset to device default */
#define ADS8330_CMR_WCFR    0b1110000000000000ul /* write device commands */
#define ADS8330_CMR_RCFR    0b1100000000000000ul /* read device commands */
#define ADS8330_CMR_WAKE    0b1011000000000000ul /* wake device from nap */
#define ADS8330_CMR_RDATA   0b1101000000000000ul /* read ADC data */
#define ADS8330_CMR_CH0     0b0000000000000000ul /* select ADC channel 0 */
#define ADS8330_CMR_CH1     0b0001000000000000ul /* select ADC channel 1 */

#define ADS8330_CFR_D11     0b0000100000000000ul /* auto channel */
#define ADS8330_CFR_D10     0b0000010000000000ul /* internal conversion clock */
#define ADS8330_CFR_D9      0b0000001000000000ul /* manual conversion */
#define ADS8330_CFR_D6_7    0b0000000011000000ul /* EOC low */
#define ADS8330_CFR_D5      0b0000000000100000ul /* EOC output */
#define ADS8330_CFR_D2_3_4_MANU  0b0000000000010100ul /* power save */
#define ADS8330_CFR_D2_3_4_AUTO  0b0000000000011100ul /* power save */
#define ADS8330_CFR_D1      0b0000000000000010ul /* tag bit */
#define ADS8330_CFR_D0      0b0000000000000001ul /* device run normal/reset */

#define ADS8330_CMR_CONF_MANU    ADS8330_CMR_WCFR | ADS8330_CFR_D10 | ADS8330_CFR_D9
#define ADS8330_CMR_CONF_AUTO    ADS8330_CMR_WCFR | ADS8330_CFR_D10
#define ADS8330_CFR_CONF_MANU    ADS8330_CFR_D6_7 | ADS8330_CFR_D5 | ADS8330_CFR_D2_3_4_MANU | ADS8330_CFR_D0
#define ADS8330_CFR_CONF_AUTO    ADS8330_CFR_D6_7 | ADS8330_CFR_D5 | ADS8330_CFR_D2_3_4_AUTO | ADS8330_CFR_D0

/* Error Return Values */
#define ADS1220_NO_ERROR           0
#define ADS1220_ERROR   

/* Command Definitions */
#define ADS1220_CMD_RDATA 0x10
#define ADS1220_CMD_RREG 0x20
#define ADS1220_CMD_WREG 0x40
#define ADS1220_CMD_SYNC 0x08
#define ADS1220_CMD_SHUTDOWN    0x02
#define ADS1220_CMD_RESET 0x06

/* ADS1220 Register Definitions */
#define ADS1220_0_REGISTER 0x00
#define ADS1220_1_REGISTER      0x01
#define ADS1220_2_REGISTER      0x02
#define ADS1220_3_REGISTER 0x03

/* ADS1220 Register 0 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                     MUX [3:0]                 |             GAIN[2:0]             | PGA_BYPASS
//
// Define MUX
#define ADS1220_MUX_0_1    0x00
#define ADS1220_MUX_0_2    0x10
#define ADS1220_MUX_0_3    0x20
#define ADS1220_MUX_1_2    0x30
#define ADS1220_MUX_1_3    0x40
#define ADS1220_MUX_2_3    0x50
#define ADS1220_MUX_1_0    0x60
#define ADS1220_MUX_3_2    0x70
#define ADS1220_MUX_0_G    0x80
#define ADS1220_MUX_1_G    0x90
#define ADS1220_MUX_2_G    0xa0
#define ADS1220_MUX_3_G    0xb0
#define ADS1220_MUX_EX_VREF 0xc0
#define ADS1220_MUX_AVDD    0xd0
#define ADS1220_MUX_DIV2    0xe0

// Define GAIN
#define ADS1220_GAIN_1      0x00
#define ADS1220_GAIN_2      0x02
#define ADS1220_GAIN_4      0x04
#define ADS1220_GAIN_8      0x06
#define ADS1220_GAIN_16     0x08
#define ADS1220_GAIN_32     0x0a
#define ADS1220_GAIN_64     0x0c
#define ADS1220_GAIN_128    0x0e

// Define PGA_BYPASS
#define ADS1220_PGA_BYPASS  0x01

/* ADS1220 Register 1 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                DR[2:0]            |      MODE[1:0]        |     CM    |     TS    |    BCS
//
// Define DR (data rate)
#define ADS1220_DR_20  0x00
#define ADS1220_DR_45  0x20
#define ADS1220_DR_90  0x40
#define ADS1220_DR_175  0x60
#define ADS1220_DR_330  0x80
#define ADS1220_DR_600  0xa0
#define ADS1220_DR_1000  0xc0

// Define MODE of Operation
#define ADS1220_MODE_NORMAL 0x00
#define ADS1220_MODE_DUTY 0x08
#define ADS1220_MODE_TURBO  0x10
#define ADS1220_MODE_DCT 0x18

// Define CM (conversion mode)
#define ADS1220_CC  0x04

// Define TS (temperature sensor)
#define ADS1220_TEMP_SENSOR 0x02

// Define BCS (burnout current source)
#define ADS1220_BCS  0x01

/* ADS1220 Register 2 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//         VREF[1:0]     |        50/60[1:0]     |    PSW    |             IDAC[2:0]
//
// Define VREF
#define ADS1220_VREF_INT 0x00
#define ADS1220_VREF_EX_DED 0x40
#define ADS1220_VREF_EX_AIN 0x80
#define ADS1220_VREF_SUPPLY 0xc0

// Define 50/60 (filter response)
#define ADS1220_REJECT_OFF 0x00
#define ADS1220_REJECT_BOTH 0x10
#define ADS1220_REJECT_50 0x20
#define ADS1220_REJECT_60 0x30

// Define PSW (low side power switch)
#define ADS1220_PSW_SW  0x08

// Define IDAC (IDAC current)
#define ADS1220_IDAC_OFF 0x00
#define ADS1220_IDAC_10  0x01
#define ADS1220_IDAC_50  0x02
#define ADS1220_IDAC_100 0x03
#define ADS1220_IDAC_250 0x04
#define ADS1220_IDAC_500 0x05
#define ADS1220_IDAC_1000 0x06
#define ADS1220_IDAC_2000 0x07

/* ADS1220 Register 3 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//               I1MUX[2:0]          |               I2MUX[2:0]          |   DRDYM   | RESERVED
//
// Define I1MUX (current routing)
#define ADS1220_IDAC1_OFF 0x00
#define ADS1220_IDAC1_AIN0 0x20
#define ADS1220_IDAC1_AIN1 0x40
#define ADS1220_IDAC1_AIN2 0x60
#define ADS1220_IDAC1_AIN3 0x80
#define ADS1220_IDAC1_REFP0 0xa0
#define ADS1220_IDAC1_REFN0 0xc0

// Define I2MUX (current routing)
#define ADS1220_IDAC2_OFF 0x00
#define ADS1220_IDAC2_AIN0 0x04
#define ADS1220_IDAC2_AIN1 0x08
#define ADS1220_IDAC2_AIN2 0x0c
#define ADS1220_IDAC2_AIN3 0x10
#define ADS1220_IDAC2_REFP0 0x14
#define ADS1220_IDAC2_REFN0 0x18

/*
 *  define DRDYM (DOUT/DRDY behavior)
 */
#define ADS1220_DRDY_MODE 0x02

/*
 * for optional SPI framework patch
 */
#define CS_CHANGE_USECS

/* 
 * SPI transfer buffer size 
 * must be a define to init buffer sizes
 * normally 1024
 */
#define HUNK_LEN 1024
#define SPECIAL_LEN 64

/* 
 * branch macros for ARM7 
 */
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

/*
 * core node state bits
 */
enum daqgert_state_bits {
	AI_CMD_RUNNING = 0,
	AO_CMD_RUNNING,
	SPI_AI_RUN,
	SPI_AO_RUN,
	CMD_TIMER,
	CMD_RUN,
};

/* 
 * this is the Comedi SPI device queue 
 */
static LIST_HEAD(device_list);

/*
 * ads1220 daq configuration
 */
static const uint8_t ads1220_r0 = ADS1220_MUX_0_1 | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS;
static const uint8_t ads1220_r0_for_mux_gain = ADS1220_PGA_BYPASS;
static const uint8_t ads1220_r1 = ADS1220_DR_20 | ADS1220_MODE_TURBO;
static const uint8_t ads1220_r2 = ADS1220_REJECT_OFF;
static const uint8_t ads1220_r3 = ADS1220_IDAC_OFF | ADS1220_DRDY_MODE;

/* analog chip types  index to daqgert_device array */
#define defdev0  0
#define mcp3002  1
#define mcp3202  2
#define mcp4802  3
#define mcp4812  4
#define mcp4822  5
#define picsl10  6
#define picsl12  7
#define ads1220  8
#define ads8330  9
#define special  10

static const uint32_t PIC18_CONVD_25K22 = 24;
static const uint32_t PIC18_CMDD_25K22 = 4;
static const uint32_t SPI_BUFF_SIZE = 128000; // normally 5000
static const uint32_t SPI_BUFF_SIZE_NOHUNK = 64000; // normally 64
static const uint32_t MAX_CHANLIST_LEN = 256;
static const uint32_t CONV_SPEED = 5000; /* 10s of nsecs: the true rate is ~3000/5000 so we need a fixup,  two conversions per mix scan */
static const uint32_t CONV_SPEED_FIX = 19; /* usecs: round it up to ~50usecs total with this */
static const uint32_t CONV_SPEED_FIX_FREERUN = 1; /* usecs: round it up to ~30usecs total with this */
static const uint32_t CONV_SPEED_FIX_FAST = 9; /* used for the MCP3002 ADC */
static const uint32_t CONV_ADS8330 = 0; /* used for the ADS8330 ADC */
static const uint32_t MAX_BOARD_RATE = 1000000000;
static const uint32_t CS_CHANGE_DELAY_USECS = 1; // spi transfer spacing 1 equals zero microseconds of delay, 0 equals the default of 10us
static const uint32_t CSnA = 0; /* GPIO 8  Gertboard ADC */
static const uint32_t CSnB = 1; /* GPIO 7  Gertboard DAC */

/* 
 * PIC Slave commands 
 */
static const uint8_t CMD_ZERO = 0x0;
static const uint8_t CMD_ADC_GO = 0x80;
static const uint8_t CMD_PORT_GO = 0xa0; /* send data LO_NIBBLE to port buffer */
static const uint8_t CMD_CHAR_GO = 0xb0; /* send data LO_NIBBLE to TX buffer */
static const uint8_t CMD_ADC_DATA = 0xc0;
static const uint8_t CMD_PORT_DATA = 0xd0; /* send data HI_NIBBLE to port buffer ->PORT and return input PORT data in received SPI data byte */
static const uint8_t CMD_CHAR_DATA = 0xe0; /* send data HI_NIBBLE to TX buffer and return RX buffer in received SPI data byte */
static const uint8_t CMD_XXXX = 0xf0; /* ??? */
static const uint8_t CMD_CHAR_RX = 0x10; /* Get RX buffer */
static const uint8_t CMD_DUMMY_CFG = 0x40; /* stuff config data in SPI buffer */
static const uint8_t CMD_DEAD = 0xff; /* This is usually a bad response */

/*
 * WPI constants
 */
static const uint32_t PIN_SAFE_MASK_WPI = 0x7f00;
static const uint32_t PIN_SAFE_MASK_GPIO1 = 0xf83;
static const uint32_t PIN_SAFE_MASK_GPIO2 = 0xf8c;
static const uint32_t INPUT = 0;
static const uint32_t OUTPUT = 1;
static const uint32_t PWM_OUTPUT = 2;
static const uint32_t LOW = 0;
static const uint32_t HIGH = 1;

/* GPPUD:
 * GPIO Pin pull up/down register
 */
static const uint32_t GPPUD = 37;
static const uint32_t PUD_OFF = 0;
static const uint32_t PUD_DOWN = 1;
static const uint32_t PUD_UP = 2;

/* 
 * driver hardware numbers 
 */
static const uint32_t NUM_DIO_CHAN = 17;
static const uint32_t NUM_DIO_CHAN_REV2 = 17;
static const uint32_t NUM_DIO_CHAN_REV3 = 17;
static const uint32_t NUM_DIO_OUTPUTS = 8;
static const uint32_t DIO_PINS_DEFAULT = 0xff;

/* 
 * Globals for the RPi board rev 
 */
extern uint32_t system_rev; /* from the kernel symbol table exports */
extern uint32_t system_serial_low;
extern uint32_t system_serial_high;

/* 
 * module configuration and data variables
 * found at /sys/modules/daq_gert/parameters 
 */
static int32_t daqgert_conf = 0;
module_param(daqgert_conf, int, S_IRUGO);
MODULE_PARM_DESC(daqgert_conf, "hardware configuration: default 0=gertboard factory standard");
static int32_t pullups = 2;
module_param(pullups, int, S_IRUGO);
MODULE_PARM_DESC(pullups, "gpio pins 0: none 1: down 2: up: 2=default");
static int32_t gpiosafe = 1;
module_param(gpiosafe, int, S_IRUGO);
static int32_t dio_conf = 0;
module_param(dio_conf, int, S_IRUGO);
MODULE_PARM_DESC(dio_conf, "internal board rev code");
static uint32_t ai_count = 0;
module_param(ai_count, uint, S_IRUGO);
MODULE_PARM_DESC(ai_count, "total adc samples");
static uint32_t ao_count = 0;
module_param(ao_count, uint, S_IRUGO);
MODULE_PARM_DESC(ao_count, "total dac samples");
static uint32_t hunk_count = 0;
module_param(hunk_count, uint, S_IRUGO);
static int32_t hunk_len = HUNK_LEN;
module_param(hunk_len, int, S_IRUGO);
static int32_t gert_autoload = 1;
module_param(gert_autoload, int, S_IRUGO);
MODULE_PARM_DESC(gert_autoload, "boot autoload: default 1=load module");
static int32_t gert_type = 0;
module_param(gert_type, int, S_IRUGO);
MODULE_PARM_DESC(gert_type, "i/o board type: default 0=gertboard");
static int32_t speed_test = 0;
module_param(speed_test, int, S_IRUGO);
MODULE_PARM_DESC(speed_test, "sample timing test: 1=enable");
static int32_t special_test = 0;
module_param(special_test, int, S_IRUGO);
MODULE_PARM_DESC(special_test, "special timing test: 1=enable");
static int32_t lsamp_size = 0;
module_param(lsamp_size, int, S_IRUGO);
MODULE_PARM_DESC(lsamp_size, "16 or 32 bit lsampl size: 0=16 bit");
static int32_t wiringpi = 1;
module_param(wiringpi, int, S_IRUGO);
static int32_t use_hunking = 1;
module_param(use_hunking, int, S_IRUGO);

struct daqgert_device {
	const char *name;
	int32_t ai_subdev_flags;
	int32_t ao_subdev_flags;
	uint32_t min_acq_ns;
	uint32_t rate_min;
	uint32_t max_speed_hz;
	uint32_t spi_mode;
	uint32_t spi_bpw;
	uint32_t n_chan_bits;
	uint32_t n_chan;
	uint32_t n_transfers;
};

static const struct daqgert_device daqgert_devices[] = {
	{
		.name = "defdev0",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 500000,
		.min_acq_ns = 22160,
		.rate_min = 20000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_transfers = 3,
	},
	{
		.name = "mcp3002",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 1000000,
		.min_acq_ns = 20000,
		.rate_min = 30000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 2,
		.n_transfers = 2,
	},
	{
		.name = "mcp3202",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 1000000,
		.min_acq_ns = 28800,
		.rate_min = 30000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 0,
		.n_transfers = 3,
	},
	{
		.name = "mcp4802",
		.ao_subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_CMD_WRITE,
		.max_speed_hz = 16000000,
		.min_acq_ns = 5400,
		.rate_min = 6000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 4,
		.n_transfers = 2,
	},
	{
		.name = "mcp4812",
		.ao_subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_CMD_WRITE,
		.max_speed_hz = 16000000,
		.min_acq_ns = 5400,
		.rate_min = 6000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 2,
		.n_transfers = 2,
	},
	{
		.name = "mcp4822",
		.ao_subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_CMD_WRITE,
		.max_speed_hz = 16000000,
		.min_acq_ns = 5400,
		.rate_min = 6000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 0,
		.n_transfers = 2,
	},
	{
		.name = "picsl10",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 4000000,
		.min_acq_ns = 20000,
		.rate_min = 20000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 2,
		.n_transfers = 3,
	},
	{
		.name = "picsl12",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 4000000,
		.min_acq_ns = 20000,
		.rate_min = 20000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 0,
		.n_transfers = 3,
	},
	{
		.name = "ads1220",
		.ai_subdev_flags = SDF_READABLE | SDF_DIFF | SDF_GROUND | SDF_CMD_READ | SDF_COMMON | SDF_LSAMPL,
		.max_speed_hz = 500000,
		.min_acq_ns = 20000,
		.rate_min = 200000,
		.spi_mode = 1,
		.spi_bpw = 8,
		.n_chan_bits = 24,
		.n_chan = 5,
		.n_transfers = 3,
	},
	{
		.name = "ads8330",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 16000000,
		.min_acq_ns = 3200,
		.rate_min = 3000,
		.spi_mode = 1,
		.spi_bpw = 8,
		.n_chan_bits = 16,
		.n_chan = 2,
		.n_transfers = 2,
	},
	{
		.name = "special",
		.ai_subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ | SDF_COMMON,
		.max_speed_hz = 64000000,
		.min_acq_ns = 30000,
		.rate_min = 30000,
		.spi_mode = 3,
		.spi_bpw = 8,
		.n_chan_bits = 12,
		.n_chan = 2,
		.n_transfers = SPECIAL_LEN,
	},
};

struct daqgert_board {
	const char *name;
	int32_t board_type;
	int32_t n_aichan;
	uint8_t n_aichan_bits;
	int32_t n_aochan;
	uint8_t n_aochan_bits;
	uint32_t ai_ns_min_calc;
	uint32_t ao_ns_min_calc;
	uint32_t ai_cs;
	uint32_t ao_cs;
	int32_t ai_node;
	int32_t ao_node;
};

static const struct daqgert_board daqgert_boards[] = {
	{
		.name = "Gertboard",
		.board_type = 0,
		.n_aichan = 2,
		.n_aichan_bits = 12,
		.n_aochan = 2,
		.n_aochan_bits = 12,
		.ai_ns_min_calc = 35000,
		.ao_ns_min_calc = 20000,
		.ai_cs = 0,
		.ao_cs = 1,
		.ai_node = 3,
		.ao_node = 2,
	},
	{
		.name = "Myboard",
		.board_type = 1,
		.n_aichan = 8,
		.n_aochan = 8,
		.ai_ns_min_calc = 35000,
		.ao_ns_min_calc = 12000,
		.ai_cs = 0,
		.ao_cs = 1,
		.ai_node = 3,
		.ao_node = 2,
	},
};

static const struct comedi_lrange daqgert_ai_range3_300 = {1,
	{
		UNI_RANGE(3.300),
	}};
static const struct comedi_lrange daqgert_ai_range2_048 = {1,
	{
		UNI_RANGE(2.048),
	}};

static const struct comedi_lrange daqgert_ao_range = {2,
	{/* gains 1,2 */
		UNI_RANGE(2.048),
		UNI_RANGE(4.096)
	}};

static const struct comedi_lrange range_ads1220_ai = {
	3,
	{ /* gains 1,2,4 */
		BIP_RANGE(2.048),
		BIP_RANGE(1.024),
		BIP_RANGE(0.512)
	}
};

/* 
 * SPI attached devices used by Comedi for I/O 
 */
struct spi_param_type {
	uint32_t range : 2;
	uint32_t bits : 2;
	uint32_t link : 1;
	uint32_t pic18 : 2;
	uint32_t chan : 4;
	struct spi_device *spi;
	int32_t device_type;
	int32_t device_detect;
	const struct daqgert_device *device_spi;
	int32_t hunk_size; /* the number of needed values returned as data */
	struct timer_list my_timer;
	struct task_struct *daqgert_task;
};

/* 
 * Comedi SPI device I/O buffer control structure 
 */
struct comedi_spigert {
	uint8_t *tx_buff;
	uint8_t *rx_buff;
	struct spi_transfer t[HUNK_LEN], one_t;
	uint32_t delay_usecs;
	uint32_t delay_usecs_freerun;
	uint32_t mix_delay_usecs;
	uint32_t delay_usecs_calc;
	uint32_t mix_delay_usecs_calc;
	struct list_head device_entry;
	struct spi_param_type slave;
	ktime_t kmin;
	uint32_t delay_nsecs;
};

/* 
 * RPi board control state variables 
 */
struct daqgert_private {
	uint32_t checkmark;
	uint32_t RPisys_rev;
	uint32_t __iomem *timer_1mhz;
	int32_t *pinToGpio;
	int32_t *physToGpio;
	int32_t board_rev;
	int32_t num_subdev;
	unsigned long state_bits;
	uint32_t ao_timer;
	uint32_t ai_count, ao_count, ao_counter, hunk_count;
	uint32_t ai_conv_delay_usecs, ai_conv_delay_10nsecs, ai_cmd_delay_usecs;
	int32_t ai_chan, ao_chan, ai_range, ao_range;
	struct mutex drvdata_lock, cmd_lock;
	uint32_t val;
	uint32_t ai_hunk;
	uint32_t run;
	uint32_t use_hunking : 1;
	uint32_t ai_mix : 1;
	uint32_t ai_neverending : 1;
	uint32_t ao_neverending : 1;
	uint32_t timer : 1;
	uint32_t ai_cmd_canceled : 1;
	uint32_t ao_cmd_canceled : 1;
	uint32_t timing_lockout : 1;
	int32_t mix_chan;
	uint32_t ai_scans; /*  length of scanlist */
	int32_t ai_scans_left; /*  number left to finish */
	uint32_t ao_scans; /*  length of scanlist */
	struct spi_param_type *ai_spi;
	struct spi_param_type *ao_spi;
	void (*pinMode) (struct comedi_device *dev, int32_t pin, int32_t mode);
	void (*digitalWrite) (struct comedi_device *dev, int32_t pin,
		int32_t value);
	void(*setPadDrive) (struct comedi_device *dev, int32_t group,
		int32_t value);
	int32_t(*digitalRead) (struct comedi_device *dev, int32_t pin);
	int32_t ai_node;
	int32_t ao_node;
	uint32_t cpu_nodes;
	bool smp;
	struct comedi_8254 pacer;
};
static int32_t daqgert_spi_setup(struct spi_param_type *);
static int32_t daqgert_spi_probe(struct comedi_device *,
	struct spi_param_type *,
	struct spi_param_type *);
static void daqgert_ai_clear_eoc(struct comedi_device *);
static int32_t daqgert_ai_cancel(struct comedi_device *,
	struct comedi_subdevice *);
static int32_t daqgert_ao_cancel(struct comedi_device *,
	struct comedi_subdevice *);
static void daqgert_handle_ai_eoc(struct comedi_device *,
	struct comedi_subdevice *);
static void daqgert_handle_ao_eoc(struct comedi_device *,
	struct comedi_subdevice *);
static void my_timer_ai_callback(unsigned long);
static void daqgert_ai_set_chan_range(struct comedi_device *,
	uint32_t, char);
static int32_t daqgert_ai_get_sample(struct comedi_device *,
	struct comedi_subdevice *);
static void daqgert_ao_put_sample(struct comedi_device *,
	struct comedi_subdevice *,
	uint32_t);
static void daqgert_handle_ai_hunk(struct comedi_device *,
	struct comedi_subdevice *);

/* 
 * pin exclude list 
 */
static int32_t wpi_pin_safe(struct comedi_device *dev,
	int32_t pin)
{
	struct daqgert_private *devpriv = dev->private;
	uint32_t pin_bit = (0x01 << pin), ret = true;

	if (!gpiosafe)
		return ret;
	if (wiringpi) {
		if (pin_bit & PIN_SAFE_MASK_WPI)
			ret = false;
	} else {
		if (devpriv->board_rev == 1) {
			if (pin_bit & PIN_SAFE_MASK_GPIO1)
				ret = false;
		} else {
			if (pin_bit & PIN_SAFE_MASK_GPIO2)
				ret = false;
		}
	}
	return ret;
}

/* 
 * Wiring PI routines modified for sparce and Comedi 
 */
/*
 Doing it the Arduino way with lookup tables...
      Yes, it's probably more inefficient than all the bit-twiddling, but it
      does tend to make it all a bit clearer. At least to me!

 pinToGpio:
      Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
      Cope for 2 different board revisions here
 */

static int32_t pinToGpioR1 [64] = {
	/* From the Original Wiki - GPIO 0 through 7:   wpi  0 -  7 */
	17, 18, 21, 22, 23, 24, 25, 4,
	0, 1, /* I2C  - SDA1, SCL1   wpi  8 -  9 */
	8, 7, /* SPI  - CE1, CE0     wpi 10 - 11 */
	10, 9, 11, /* SPI  - MOSI, MISO, SCLK    wpi 12 - 14 */
	14, 15, /* UART - Tx, Rx    wpi 15 - 16 */
	/* Padding: */
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

/* 
 * Revision 2: 
 */

static int32_t pinToGpioR2 [64] = {
	/* From the Original Wiki - GPIO 0 through 7:   wpi  0 -  7 */
	17, 18, 27, 22, 23, 24, 25, 4,
	2, 3, /* I2C  - SDA0, SCL0     wpi  8 -  9 */
	8, 7, /* SPI  - CE1, CE0       wpi 10 - 11 */
	10, 9, 11, /* SPI  - MOSI, MISO, SCLK  wpi 12 - 14 */
	14, 15, /* UART - Tx, Rx   wpi 15 - 16 */
	28, 29, 30, 31, /* Rev 2: New GPIOs 8 though 11  wpi 17 - 20 */
	5, 6, 13, 19, 26, /* B+   wpi 21, 22, 23, 24, 25 */
	12, 16, 20, 21, /* B+     wpi 26, 27, 28, 29 */
	0, 1, /* B+               wpi 30, 31 */

	/* Padding: */

	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

/*	physToGpio:
	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
	Cope for 2 different board revisions here.
	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56
 */

static int32_t physToGpioR1 [64] = {
	-1, /* 0 */
	-1, -1, /* 1, 2 */
	0, -1,
	1, -1,
	4, 14,
	-1, 15,
	17, 18,
	21, -1,
	22, 23,
	-1, 24,
	10, -1,
	9, 25,
	11, 8,
	-1, 7, /* 25, 26 */

	-1, -1, -1, -1, -1, /* ... 31 */
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

static int32_t physToGpioR2 [64] = {
	-1, /* 0 */
	-1, -1, /* 1, 2 */
	2, -1,
	3, -1,
	4, 14,
	-1, 15,
	17, 18,
	27, -1,
	22, 23,
	-1, 24,
	10, -1,
	9, 25,
	11, 8,
	-1, 7, /* 25, 26 */

	/* B+ */

	0, 1,
	5, -1,
	6, 12,
	13, -1,
	19, 16,
	26, 20,
	-1, 21,

	/* the P5 connector on the Rev 2 boards: */

	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	28, 29,
	30, 31,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
};

/*
	gpioToGPFSEL:
	Map a BCM_GPIO pin to it's Function Selection
	control port. (GPFSEL 0-5)
	Groups of 10 - 3 bits per Function - 30 bits per port
 */

static uint8_t gpioToGPFSEL [] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
};

/*
	gpioToShift
	Define the shift up for the 3 bits per pin in each GPFSEL port
 */

static uint8_t gpioToShift [] = {
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
	0, 3, 6, 9, 12, 15, 18, 21, 24, 27,
};

/*
	gpioToGPSET:
	(Word) offset to the GPIO Set registers for each GPIO pin
 */

static uint8_t gpioToGPSET [] = {
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
};

/*
	gpioToGPCLR:
	(Word) offset to the GPIO Clear registers for each GPIO pin
 */

static uint8_t gpioToGPCLR [] = {
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
};

/*
	gpioToGPLEV:
	(Word) offset to the GPIO Input level registers for each GPIO pin
 */

static uint8_t gpioToGPLEV [] = {
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
};

/*
	gpioToPUDCLK
	(Word) offset to the Pull Up Down Clock register
 */

static uint8_t gpioToPUDCLK [] = {
	38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38,
	38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38,
	39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
	39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39,
};

/*
 * pullUpDownCtrl:
 *      Control the internal pull-up/down resistors on a GPIO pin
 *      The Arduino only has pull-ups and these are enabled by writing 1
 *      to a port when in input mode - this paradigm doesn't quite apply
 *      here though.
 ****************************************************************************
 */

static void pullUpDnControl(struct comedi_device *dev, int32_t pin, int32_t pud)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t *pinToGpio = devpriv->pinToGpio;

	pin = pinToGpio [pin];
	iowrite32(pud & 3, (__iomem uint32_t*) dev->mmio + GPPUD);
	udelay(5);
	iowrite32(1 << (pin & 31), (__iomem uint32_t*) dev->mmio
		+ gpioToPUDCLK [pin]);
	udelay(5);

	iowrite32(0, (__iomem uint32_t*) dev->mmio + GPPUD);
	udelay(5);
	iowrite32(0, (__iomem uint32_t*) dev->mmio + gpioToPUDCLK [pin]);
	udelay(5);
}

/*
 * pinMode:
 *      Sets the mode of a pin to be input, output
 ************************************************************
 */

static void pinModeGpio(struct comedi_device *dev,
	int32_t pin,
	int32_t mode)
{
	int32_t fSel, shift;

	pin &= 63;
	fSel = gpioToGPFSEL [pin];
	shift = gpioToShift [pin];

	if (mode == INPUT) /* Sets bits to zero = input */
		iowrite32(ioread32((__iomem uint32_t*) dev->mmio + fSel)
		& ~(7 << shift),
		(__iomem uint32_t*) dev->mmio + fSel);
	else
		if (mode == OUTPUT)
		iowrite32((ioread32((__iomem uint32_t*) dev->mmio + fSel)
		& ~(7 << shift)) | (1 << shift),
		(__iomem uint32_t*) dev->mmio + fSel);
}

static void pinModeWPi(struct comedi_device *dev,
	int32_t pin,
	int32_t mode)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t *pinToGpio = devpriv->pinToGpio;

	pinModeGpio(dev, pinToGpio [pin & 63], mode);
}

/*
 * digitalWrite:
 *      Set an output bit
 *****************************************************************
 */

static void digitalWriteWPi(struct comedi_device *dev,
	int32_t pin,
	int32_t value)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t *pinToGpio = devpriv->pinToGpio;

	pin = pinToGpio [pin & 63];
	if (value == LOW)
		iowrite32(1 << (pin & 31), (__iomem uint32_t*) dev->mmio
		+ gpioToGPCLR [pin]);
	else
		iowrite32(1 << (pin & 31), (__iomem uint32_t*) dev->mmio
		+ gpioToGPSET [pin]);
}

static void digitalWriteGpio(struct comedi_device *dev,
	int32_t pin,
	int32_t value)
{

	pin &= 63;
	if (value == LOW)
		iowrite32(1 << (pin & 31), (__iomem uint32_t*) dev->mmio
		+ gpioToGPCLR [pin]);
	else
		iowrite32(1 << (pin & 31), (__iomem uint32_t*) dev->mmio
		+ gpioToGPSET [pin]);
}

/*
 * digitalRead:
 *      Read the value of a given Pin, returning HIGH or LOW
 *******************************************************************
 */

static int32_t digitalReadWPi(struct comedi_device *dev,
	int32_t pin)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t *pinToGpio = devpriv->pinToGpio;

	pin = pinToGpio [pin & 63];
	if ((ioread32((__iomem uint32_t*) dev->mmio + gpioToGPLEV [pin])
		& (1 << (pin & 31))) != 0)
		return HIGH;
	else
		return LOW;
}

static int32_t digitalReadGpio(struct comedi_device *dev,
	int32_t pin)
{

	pin &= 63;
	if ((ioread32((__iomem uint32_t*) dev->mmio + gpioToGPLEV [pin])
		& (1 << (pin & 31))) != 0)
		return HIGH;
	else
		return LOW;
}

/*
 * piBoardRev:
 *	Return a number representing the hardware revision of the board.
 *	Revision is currently 1,2 or 3. -1 is returned on error.
SRRR MMMM PPPP TTTT TTTT VVVV

S scheme (0=old, 1=new)
R RAM (0=256, 1=512, 2=1024)
M manufacturer (0='SONY',1='EGOMAN',2='EMBEST',3='UNKNOWN',4='EMBEST')
P processor (0=2835, 1=2836)
T type (0='A', 1='B', 2='A+', 3='B+', 4='Pi 2 B', 5='Alpha', 6='Compute Module')
V revision (0-15)

1010 0010 0001 0000 0100 0001
SRRR MMMM PPPP TTTT TTTT VVVV

S=1 new scheme
R=2 1024 MB
M=2 EMBEST
P=1 2836
T=4 Pi 2 B
V=1 1
 *********************************************************************
 */

static int32_t piBoardRev(struct comedi_device *dev)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t r = -1, nscheme = 0;
	static int32_t boardRev = -1;

	if (boardRev != -1) /* skip if already detected */
		return boardRev;

	if (devpriv->RPisys_rev & 0x80000000)
		dev_info(dev->class_dev, "over-volt bit set\n");
	if (devpriv->RPisys_rev & 0x800000) {
		nscheme = 1;
		r = devpriv->RPisys_rev & 0xf;
		dev_info(dev->class_dev, "RPi new scheme rev %x, "
			"serial %08x%08x, new rev %x\n",
			devpriv->RPisys_rev, system_serial_high,
			system_serial_low, r);
	} else {
		r = devpriv->RPisys_rev & 0xff;
		dev_info(dev->class_dev, "RPi old scheme rev %x, "
			"serial %08x%08x\n",
			devpriv->RPisys_rev, system_serial_high,
			system_serial_low);
	}

	if (nscheme) {
		if (PERI_BASE == 0x20000000)
			r = -1;
		switch (r) {
		case 1:
			boardRev = 3;
			break;
		case 2:
			boardRev = 3;
			break;
		default:
			boardRev = -1;
		}
	} else {
		if (PERI_BASE == 0x3F000000)
			r = 1;
		switch (r) {
		case 2:
		case 3:
			boardRev = 1;
			break;
		case 0:
		case 1:
			boardRev = -1;
			break;
		default:
			boardRev = 2;
		}
	}

	dev_info(dev->class_dev, "driver gpio board rev %i\n",
		boardRev);
	dio_conf = boardRev; /* set module param */

	return boardRev;
}

/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initializes the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 ************************************************************************
 */

static int32_t wiringPiSetup(struct comedi_device *dev)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t boardRev;

	devpriv->pinMode = pinModeWPi;
	devpriv->digitalWrite = digitalWriteWPi;
	devpriv->digitalRead = digitalReadWPi;

	if ((boardRev = piBoardRev(dev)) < 0)
		return -1;

	/* set the comedi private data */
	if (boardRev == 1) {
		devpriv->pinToGpio = pinToGpioR1;
		devpriv->physToGpio = physToGpioR1;
	} else {
		devpriv->pinToGpio = pinToGpioR2;
		devpriv->physToGpio = physToGpioR2;
	}
	return 0;
}

/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initializes the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *************************************************************************
 */

static int32_t wiringPiSetupGpio(struct comedi_device *dev)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t x;

	if ((x = wiringPiSetup(dev)) < 0)
		return x;

	devpriv->pinMode = pinModeGpio;
	devpriv->digitalWrite = digitalWriteGpio;
	devpriv->digitalRead = digitalReadGpio;

	return 0;
}

static void ADS1220WriteRegister(int32_t StartAddress, int32_t NumRegs, uint32_t * pData, struct comedi_subdevice *s)
{
	int32_t i;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	struct spi_message m;

	pdata->tx_buff[0] = ADS1220_CMD_WREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1)&0x03));

	for (i = 0; i < NumRegs; i++) {
		pdata->tx_buff[i + 1] = *pData++;
	}

	pdata->one_t.len = NumRegs + 2;
	pdata->one_t.cs_change = false;
	pdata->one_t.delay_usecs = 0;
	spi_message_init_with_transfers(&m, &pdata->one_t, 1);
	spi_bus_lock(pdata->slave.spi->master);
	spi_sync_locked(pdata->slave.spi, &m);
	spi_bus_unlock(pdata->slave.spi->master);

	return;
}

/* 
 * chip byte offsets for arrays for spi device transfers 
 */
static int32_t daqgert_device_offset(int32_t device_type)
{
	int32_t len;

	switch (device_type) {
	case ads8330:
	case mcp3002:
		len = 2;
		break;
	case mcp3202:
		len = 3;
		break;
	case special:
		len = SPECIAL_LEN;
		break;
	default:
		len = 3;
	}
	return len;
}

/* 
 * A client must be connected with a valid comedi cmd 
 * and *data a pointer to that comedi structure
 * for this not to segfault 
 */
static DECLARE_WAIT_QUEUE_HEAD(daqgert_ai_thread_wq);

static int32_t daqgert_ai_thread_function(void *data)
{
	struct comedi_device *dev = (void*) data;
	struct comedi_subdevice *s = dev->read_subdev;
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;

	if (!dev)
		return -EFAULT;
	dev_info(dev->class_dev, "ai device thread start\n");

	while (!kthread_should_stop()) {
		while (unlikely(!devpriv->run)) {
			if (devpriv->timer)
				schedule();
			else
				wait_event_interruptible(daqgert_ai_thread_wq, test_bit(AI_CMD_RUNNING, &devpriv->state_bits));

			if (kthread_should_stop())
				return 0;
		}
		if (likely(test_bit(AI_CMD_RUNNING, &devpriv->state_bits))) {
			if (likely(devpriv->ai_hunk)) {
				daqgert_handle_ai_hunk(dev, s);
				devpriv->hunk_count++;
				hunk_count = devpriv->hunk_count;
			} else {
				daqgert_handle_ai_eoc(dev, s);
				devpriv->ai_count++;
				pdata->kmin = ktime_set(0, pdata->delay_nsecs);
				__set_current_state(TASK_UNINTERRUPTIBLE);
				schedule_hrtimeout_range(&pdata->kmin, 0,
					HRTIMER_MODE_REL_PINNED);
			}
		} else {
			clear_bit(SPI_AI_RUN, &devpriv->state_bits);
			smp_mb__after_atomic();
			wait_event_interruptible(daqgert_ai_thread_wq, test_bit(AI_CMD_RUNNING, &devpriv->state_bits));
			smp_mb__before_atomic();
			set_bit(SPI_AI_RUN, &devpriv->state_bits);
			smp_mb__after_atomic();
		}
	}

	return 0;
}

/*
 * AO async thread
 */
static DECLARE_WAIT_QUEUE_HEAD(daqgert_ao_thread_wq);

static int32_t daqgert_ao_thread_function(void *data)
{
	struct comedi_device *dev = (void*) data;
	struct comedi_subdevice *s = &dev->subdevices[2];
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;

	if (!dev)
		return -EFAULT;
	dev_info(dev->class_dev, "ao device thread start\n");

	while (!kthread_should_stop()) {
		if (likely(test_bit(AO_CMD_RUNNING, &devpriv->state_bits))) {
			daqgert_handle_ao_eoc(dev, s);
			pdata->kmin = ktime_set(0, pdata->delay_nsecs);
			__set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_hrtimeout_range(&pdata->kmin, 0,
				HRTIMER_MODE_REL_PINNED);
		} else {
			clear_bit(SPI_AO_RUN, &devpriv->state_bits);
			smp_mb__after_atomic();
			wait_event_interruptible(daqgert_ao_thread_wq, test_bit(AO_CMD_RUNNING, &devpriv->state_bits));
			smp_mb__before_atomic();
			set_bit(SPI_AO_RUN, &devpriv->state_bits);
			smp_mb__after_atomic();
		}
	}

	return 0;
}

/*
 * AI async thread timer
 * 
 */
static void daqgert_ai_start_pacer(struct comedi_device *dev,
	bool load_timers)
{
	struct daqgert_private *devpriv = dev->private;

	if (load_timers)
		/* setup timer interval to 100 msecs */
		mod_timer(&devpriv->ai_spi->my_timer, jiffies
		+ msecs_to_jiffies(100));
}

static void daqgert_ai_set_chan_range_ads1220(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint32_t chanspec)
{
	struct daqgert_private *devpriv = dev->private;
	uint32_t range = CR_RANGE(chanspec);
	uint32_t chan = CR_CHAN(chanspec);
	uint32_t cMux;

	/*
	 * convert chanspec to input MUX switches/gains if needed
	 * we could just feed the raw bits to the Mux if needed
	 */

	if ((devpriv->ai_chan != chan) || (devpriv->ai_range != range)) {
		switch (chan) {
		case 0:
			cMux = ADS1220_MUX_0_1;
			break;
		case 1:
			cMux = ADS1220_MUX_2_3;
			break;
		case 2:
			cMux = ADS1220_MUX_2_G;
			break;
		case 3:
			cMux = ADS1220_MUX_3_G;
			break;
		case 4:
			cMux = ADS1220_MUX_DIV2;
			break;
		default:
			cMux = ADS1220_MUX_0_1;
		}
		cMux |= ((range & 0x03) << 1); /* setup the gain bits for range with NO pga */
		cMux |= ads1220_r0_for_mux_gain;
		ADS1220WriteRegister(ADS1220_0_REGISTER, 0x01, &cMux, s);
	}
}

static void daqgert_ai_set_chan_range_ads8330(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint32_t chanspec)
{
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	struct spi_message m;
	uint32_t chan = CR_CHAN(chanspec);
	uint32_t cMux;

	/*
	 * convert chanspec to input MUX switches if needed
	 * we could just feed the raw bits to the Mux if needed
	 */

	if (devpriv->ai_chan != chan) {
		switch (chan) {
		case 1:
			cMux = ADS8330_CMR_CH0 >> 8;
			break;
		default:
			cMux = ADS8330_CMR_CH0 >> 8;
		}

		pdata->tx_buff[4] = cMux;
		pdata->tx_buff[5] = 0;
		pdata->tx_buff[0] = (ADS8330_CMR_CONF_MANU) >> 8;
		pdata->tx_buff[1] = ADS8330_CFR_CONF_MANU;
		pdata->tx_buff[2] = (ADS8330_CMR_CONF_AUTO) >> 8;
		pdata->tx_buff[3] = ADS8330_CFR_CONF_AUTO;
		/* one transfer */
		pdata->t[0].cs_change = false;
		pdata->t[0].len = 2;
		pdata->t[0].tx_buf = &pdata->tx_buff[0];
		pdata->t[0].rx_buf = &pdata->rx_buff[0];
		pdata->t[0].delay_usecs = 0;
		pdata->t[0].cs_change_usecs = CS_CHANGE_DELAY_USECS;
		pdata->t[1].cs_change = false;
		pdata->t[1].len = 2;
		pdata->t[1].tx_buf = &pdata->tx_buff[2];
		pdata->t[1].rx_buf = &pdata->rx_buff[2];
		pdata->t[1].delay_usecs = 0;
		pdata->t[1].cs_change_usecs = CS_CHANGE_DELAY_USECS;
		pdata->t[2].cs_change = false;
		pdata->t[2].len = 2;
		pdata->t[2].tx_buf = &pdata->tx_buff[4];
		pdata->t[2].rx_buf = &pdata->rx_buff[4];
		pdata->t[2].delay_usecs = 0;
		pdata->t[2].cs_change_usecs = CS_CHANGE_DELAY_USECS;
		spi_message_init_with_transfers(&m, &pdata->t[0], 2);
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
	}
}

/*
 * ADC SPI channel and voltage gains
 */
static void daqgert_ai_set_chan_range(struct comedi_device *dev,
	uint32_t chanspec,
	char wait)
{
	struct daqgert_private *devpriv = dev->private;

	if (devpriv->ai_spi->device_type == ads1220)
		daqgert_ai_set_chan_range_ads1220(dev, &dev->subdevices[1], chanspec);
	if (devpriv->ai_spi->device_type == ads8330)
		daqgert_ai_set_chan_range_ads8330(dev, &dev->subdevices[1], chanspec);

	devpriv->ai_chan = CR_CHAN(chanspec);
	devpriv->ai_range = CR_RANGE(chanspec);

	if (wait)
		udelay(1);
}

/*
 * DAC SPI channel and voltage gains
 */
static void daqgert_ao_set_chan_range(struct comedi_device *dev,
	uint32_t chanspec,
	char wait)
{
	struct daqgert_private *devpriv = dev->private;
	devpriv->ao_chan = CR_CHAN(chanspec);
	devpriv->ao_range = CR_RANGE(chanspec);

	if (wait)
		udelay(1);
}

/*
 * transfers one 32 bit value to the DAC device
 */
static void daqgert_ao_put_sample(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint32_t val)
{
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	uint32_t chan, range;

	mutex_lock(&devpriv->drvdata_lock);
	chan = devpriv->ao_chan;
	range = devpriv->ao_range;
	pdata->tx_buff[1] = val & 0xff;
	pdata->tx_buff[0] = (0x10 | ((chan & 0x01) << 7) | ((~range & 0x01) << 5) | ((val >> 8)& 0x0f));
	spi_write(spi, pdata->tx_buff, 2);
	s->readback[chan] = val;
	devpriv->ao_count++;
	mutex_unlock(&devpriv->drvdata_lock);
	smp_mb__after_atomic();
}

/*
 * returns one 32 bit value from the ADC device
 */
static int32_t daqgert_ai_get_sample(struct comedi_device *dev,
	struct comedi_subdevice *s)
{
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	struct spi_message m;
	uint32_t chan, sync, i;
	int32_t val = 0;

	mutex_lock(&devpriv->drvdata_lock);
	chan = devpriv->ai_chan;

	/* Make SPI messages for the type of ADC are we talking to */
	switch (devpriv->ai_spi->device_type) {
	case ads1220:
		/* read the ads1220 3 byte data result */
		pdata->one_t.len = 4;
		pdata->one_t.cs_change = false;
		pdata->one_t.delay_usecs = 0;
		pdata->tx_buff[0] = ADS1220_CMD_RDATA;
		pdata->tx_buff[1] = 0;
		pdata->tx_buff[2] = 0;
		pdata->tx_buff[3] = 0;
		spi_message_init_with_transfers(&m,
			&pdata->one_t, 1);
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
		val = pdata->rx_buff[1];
		val = (val << 8) | pdata->rx_buff[2];
		val = (val << 8) | pdata->rx_buff[3];

		/* mangle the data as necessary */
		/* Bipolar Offset Binary */
		val &= 0x0ffffff;
		val ^= 0x0800000;

		sync = ADS1220_CMD_SYNC;
		spi_write(spi, &sync, 1);
		devpriv->ai_count++;
		break;
	case ads8330:
		if (likely(devpriv->ai_hunk)) {
			spi_message_init_with_transfers(&m,
				&pdata->t[0], hunk_len);
		} else {
			/* read the ads8330 2 byte data result and tag if needed */
			pdata->tx_buff[0] = ADS8330_CMR_RDATA >> 8;
			pdata->tx_buff[1] = 0;
			pdata->tx_buff[2] = 0;
			pdata->tx_buff[3] = 0;
			pdata->t[0].len = 2;
			pdata->t[0].cs_change = false;
			pdata->t[0].delay_usecs = 0;
			pdata->t[0].cs_change_usecs = CS_CHANGE_DELAY_USECS;
			pdata->t[0].tx_buf = &pdata->tx_buff[0];
			pdata->t[0].rx_buf = &pdata->rx_buff[0];
			spi_message_init_with_transfers(&m,
				&pdata->t[0], 1);
		}
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
		if (likely(devpriv->ai_hunk)) {
			/* data will be sent to comedi buffers later */
			val = 0;
		} else {
			val = pdata->rx_buff[1];
			val += (pdata->rx_buff[0] << 8);
			devpriv->ai_count++;
		}
		break;
	case picsl10:
	case picsl12:
		pdata->tx_buff[0] = CMD_ADC_GO + chan;
		pdata->tx_buff[1] = CMD_ADC_DATA;
		pdata->tx_buff[2] = CMD_ZERO;
		/* use three spi transfers for the message */
		pdata->t[0].cs_change = false;
		pdata->t[0].len = 1;
		pdata->t[0].tx_buf = &pdata->tx_buff[0];
		pdata->t[0].rx_buf = &pdata->rx_buff[0];
		pdata->t[0].delay_usecs = devpriv->ai_conv_delay_usecs;
		pdata->t[1].cs_change = false;
		pdata->t[1].len = 1;
		pdata->t[1].tx_buf = &pdata->tx_buff[1];
		pdata->t[1].rx_buf = &pdata->rx_buff[1];
		pdata->t[1].delay_usecs = devpriv->ai_cmd_delay_usecs;
		pdata->t[2].cs_change = false;
		pdata->t[2].len = 1;
		pdata->t[2].tx_buf = &pdata->tx_buff[2];
		pdata->t[2].rx_buf = &pdata->rx_buff[2];
		pdata->t[2].delay_usecs = devpriv->ai_cmd_delay_usecs;
		spi_message_init_with_transfers(&m, &pdata->t[0], 3);
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
		val = pdata->rx_buff[1];
		val += (pdata->rx_buff[2] << 8);
		devpriv->ai_count++;
		break;
	case mcp3002:
	case mcp3202:
		if (likely(devpriv->ai_hunk)) {
			spi_message_init_with_transfers(&m,
				&pdata->t[0], hunk_len);
		} else {
			pdata->one_t.len = daqgert_device_offset(devpriv->ai_spi->device_type);
			if (devpriv->ai_spi->device_type == mcp3002)
				pdata->tx_buff[0] = 0xd0 | ((chan & 0x01) << 5);
			if (devpriv->ai_spi->device_type == mcp3202) {
				pdata->tx_buff[0] = 0x01;
				pdata->tx_buff[1] = 0b10100000 | ((chan & 0x01) << 6);
			}
			spi_message_init_with_transfers(&m,
				&pdata->one_t, 1);
		}
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
		/* ADC type code result munging */
		if (likely(devpriv->ai_hunk)) {
			/* data will be sent to comedi buffers later */
			val = 0;
		} else {
			if (devpriv->ai_spi->device_type == mcp3002) {
				val = (pdata->rx_buff[1] | (pdata->rx_buff[0] << 8)) & 0x3FF;
			} else {
				val = pdata->rx_buff[2];
				val += (pdata->rx_buff[1]&0x1f) << 8;
			}
			devpriv->ai_count++;
		}
		break;
	case special: // dummy device transfer speed testing
		pdata->one_t.len = devpriv->ai_spi->device_spi->n_transfers;
		for (i = 0; i < 32; i++)
			pdata->tx_buff[i] = 0xff;

		spi_message_init_with_transfers(&m,
			&pdata->one_t, 1);
		spi_bus_lock(spi->master);
		spi_sync_locked(spi, &m);
		spi_bus_unlock(spi->master);
		val = 99;
		devpriv->ai_count++;
		break;
	default:
		devpriv->ai_count++;
		dev_info(dev->class_dev, "unknown ai device\n");
	}

	mutex_unlock(&devpriv->drvdata_lock);
	clear_bit(SPI_AI_RUN, &devpriv->state_bits);
	smp_mb__after_atomic();

	return val & s->maxdata;
}

/* 
 * start chan set in ai_cmd 
 */
static void daqgert_handle_ai_eoc(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	uint32_t next_chan;
	uint32_t val;
	uint32_t chan = s->async->cur_chan;

	val = daqgert_ai_get_sample(dev, s);
	comedi_buf_write_samples(s, &val, 1);

	next_chan = s->async->cur_chan;
	if (cmd->chanlist[chan] != cmd->chanlist[next_chan])
		daqgert_ai_set_chan_range(dev, cmd->chanlist[next_chan], false);

	if (cmd->stop_src == TRIG_COUNT &&
		s->async->scans_done >= cmd->stop_arg) {
		if (!devpriv->ai_neverending) {
			daqgert_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
		}
	}
}

static void daqgert_ao_next_chan(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;

	if (cmd->stop_src == TRIG_COUNT &&
		s->async->scans_done >= cmd->stop_arg) {
		if (!devpriv->ao_neverending) {
			/* all data sampled */
			daqgert_ao_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
		}
	}
}

/* 
 * start chan set in ao_cmd 
 */
static void daqgert_handle_ao_eoc(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	uint32_t next_chan, sampl_val;
	uint32_t chan = s->async->cur_chan;

	/* zero bytes from the write data buffer read */
	if (!comedi_buf_read_samples(s, &sampl_val, 1)) {
		s->async->events |= COMEDI_CB_OVERFLOW;
		comedi_handle_events(dev, s);
		return;
	}

	/* possible munge of data */
	daqgert_ao_put_sample(dev, s, sampl_val);
	next_chan = s->async->cur_chan;

	if (cmd->chanlist[chan] != cmd->chanlist[next_chan])
		daqgert_ao_set_chan_range(dev, cmd->chanlist[next_chan], false);
	daqgert_ao_next_chan(dev, s);
}

/*
 * moves the data from the SPI buffers into the Comedi buffer 16bit
 */
static void transfer_from_hunk_buf_8330(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint8_t *bufptr,
	uint32_t bufpos,
	uint32_t len)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	uint32_t i, val;

	s->async->cur_chan = 0; /* reset the hunk start chan */
	for (i = 0; i < len; i++) {
		//		val = (*bufptr++ << 8);
		//		val += *bufptr++;

		val = bufptr[1 + bufpos];
		val += (bufptr[0 + bufpos] << 8);

		comedi_buf_write_samples(s, &val, 1);
		bufpos += 2;

		if (unlikely(cmd->stop_src == TRIG_COUNT &&
			s->async->scans_done >= cmd->stop_arg)) {
			daqgert_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
			comedi_handle_events(dev, s);
			break;
		}
	}
}

/*
 * moves the data from the SPI buffers into the Comedi buffer 10bit
 */
static void transfer_from_hunk_buf_3002(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint8_t *bufptr,
	uint32_t bufpos,
	uint32_t len)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	uint32_t i, val;

	s->async->cur_chan = 0; /* reset the hunk start chan */
	for (i = 0; i < len; i++) {
		val = (bufptr[1 + bufpos] | (bufptr[0 + bufpos] << 8)) & 0x3FF;
		comedi_buf_write_samples(s, &val, 1);
		bufpos += 2;

		if (unlikely(cmd->stop_src == TRIG_COUNT &&
			s->async->scans_done >= cmd->stop_arg)) {
			daqgert_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
			comedi_handle_events(dev, s);
			break;
		}
	}
}

/*
 * moves the data from the SPI buffers into the Comedi buffer 12bit
 */
static void transfer_from_hunk_buf_3202(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint8_t *bufptr,
	uint32_t bufpos,
	uint32_t len)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	uint32_t i, val;

	s->async->cur_chan = 0; /* reset the hunk start chan */
	for (i = 0; i < len; i++) {
		val = bufptr[2 + bufpos] | ((bufptr[1 + bufpos] &0x1f) << 8);
		//		val += (bufptr[1 + bufpos] &0x1f) << 8;

		comedi_buf_write_samples(s, &val, 1);
		bufpos += 3;

		if (unlikely(cmd->stop_src == TRIG_COUNT &&
			s->async->scans_done >= cmd->stop_arg)) {
			daqgert_ai_cancel(dev, s);
			s->async->events |= COMEDI_CB_EOA;
			comedi_handle_events(dev, s);
			break;
		}
	}
}

/*
 * uses the Comedi cmd info to construct a transfers buffer to 
 * improve sample timing
 */
static int32_t transfer_to_hunk_buf(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint8_t *bufptr,
	uint32_t bufpos,
	uint32_t hunk_len,
	uint32_t offset,
	bool mix_mode)
{
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	uint32_t i, len;
	uint32_t chan;
	uint8_t *tx_buff, *rx_buff;
	int32_t ret = 0;

	chan = devpriv->ai_chan;
	memset(&pdata->t, 0, sizeof(pdata->t));
	len = daqgert_device_offset(spi_data->device_type);

	tx_buff = pdata->tx_buff;
	rx_buff = pdata->rx_buff;

	if (unlikely(hunk_len > hunk_len)) {
		ret = -E2BIG;
		hunk_len = hunk_len;
	}

	for (i = 0; i < hunk_len; i++) {
		/* format the tx_buffer */
		if (mix_mode) {
			if (i % 2) { /* use an even/odd mix of adc devices */
				chan = devpriv->mix_chan;
			} else {
				chan = devpriv->ai_chan;
			}
		} else {

			chan = devpriv->mix_chan; /* for single channel hunks */
		}

		if (devpriv->ai_spi->device_type == mcp3002)
			bufptr[bufpos] = 0xd0 | ((chan & 0x01) << 5);
		if (devpriv->ai_spi->device_type == mcp3202) {
			bufptr[bufpos] = 0x01;
			bufptr[bufpos + 1] = 0b10100000 | ((chan & 0x01) << 6);
		}
		if (devpriv->ai_spi->device_type == ads8330) {
			bufptr[bufpos] = (ADS8330_CMR_RDATA) >> 8;
			bufptr[bufpos + 1] = 0;
		}

		bufpos += offset;
		/*
		 *  format the transfer array 
		 *  use cs_change to start the ADC on every transfer
		 *  if it's not automatic
		 *  the spec says a brief toggle but maybe it's too
		 *  long at the default of 10us
		 */
		pdata->t[i].cs_change = true;
		pdata->t[i].len = len; // set to len, a fixed number is for testing
		pdata->t[i].tx_buf = tx_buff;
		pdata->t[i].rx_buf = rx_buff;
		pdata->t[i].delay_usecs = 0;
		/*
		 * cs_change_usecs is a optional patch to spi.h and spi.c
		 */
#ifdef CS_CHANGE_USECS
		pdata->t[i].cs_change_usecs = CS_CHANGE_DELAY_USECS;
#endif
		tx_buff += len; /* move to the next data set */
		rx_buff += len;
	}
	/* 
	 * the spi-bcm2835 driver needs this, it switches cs to false after
	 * every transfer in a msg but the last one
	 * turn off cs on last transfer
	 */
	pdata->t[i - 1].cs_change = false;
	return ret;
}

static void daqgert_handle_ai_hunk(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	int32_t len, bufpos;
	uint8_t *bufptr;

	daqgert_ai_get_sample(dev, s); /* get the data from the ADC via SPI */
	bufptr = (uint8_t *) pdata->rx_buff;
	bufpos = 0;

	len = devpriv->ai_scans;
	if (cmd->stop_src == TRIG_COUNT) {
		if (devpriv->ai_scans_left > hunk_len) {
			devpriv->ai_scans_left -= hunk_len;
			len = hunk_len;
		} else {
			len = devpriv->ai_scans_left;
			devpriv->ai_scans_left = 0;
		}
	}

	devpriv->ai_count += len;
	/*
	 * routines to optimize speed in each device transfer
	 */
	switch (spi_data->device_type) {
	case mcp3202:
		transfer_from_hunk_buf_3202(dev, s, bufptr, bufpos, len);
		break;
	case mcp3002:
		transfer_from_hunk_buf_3002(dev, s, bufptr, bufpos, len);
		break;
	case ads8330:
		transfer_from_hunk_buf_8330(dev, s, bufptr, bufpos, len);
		break;
	default:
		dev_info(dev->class_dev, "unknown ai hunk device\n");
	}

	/* 
	 * debug comment
	if (cmd->stop_src == TRIG_COUNT)
	dev_info(dev->class_dev, "From hunk %i %i\n",
	s->async->scans_done, cmd->stop_arg);
	 */
}

/*
 * test for conditions that allow for the hunk_len transfer buffer
 */
static int32_t daqgert_ai_setup_hunk(struct comedi_device *dev,
	struct comedi_subdevice *s,
	bool mix_mode)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	uint32_t len, offset, bufpos;
	uint8_t *bufptr;

	len = devpriv->ai_scans;
	if (cmd->stop_src == TRIG_COUNT) { /* optimize small samples */
		if (devpriv->ai_scans > hunk_len)
			len = hunk_len;

		else
			len = devpriv->ai_scans;
	}

	bufptr = (uint8_t *) pdata->tx_buff;
	bufpos = 0;
	offset = daqgert_device_offset(spi_data->device_type);

	/* load the message for the ADC conversions in to the tx buffer */
	return transfer_to_hunk_buf(dev, s, bufptr, bufpos, len, offset,
		mix_mode);
}

/*
 * setup a single AI transfer
 */
static void daqgert_ai_setup_eoc(struct comedi_device *dev,
	struct comedi_subdevice * s)
{

	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	uint32_t len;

	memset(&pdata->t, 0, sizeof(pdata->t));
	len = daqgert_device_offset(spi_data->device_type);

	/* format the tx_buffer */
	pdata->tx_buff[2] = 0;
	pdata->tx_buff[1] = 0;

	/* format the transfer array */
	pdata->t[0].cs_change = false;
	pdata->t[0].len = len;
	pdata->t[0].tx_buf = pdata->tx_buff;
	pdata->t[0].rx_buf = pdata->rx_buff;
}

static int32_t daqgert_ai_inttrig(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint32_t trig_num)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	int32_t ret = 0;

	if (trig_num != cmd->start_arg)
		return -EINVAL;

	mutex_lock(&devpriv->cmd_lock);
	dev_info(dev->class_dev, "ai inttrig\n");

	if (!test_bit(AI_CMD_RUNNING, &devpriv->state_bits)) {
		devpriv->run = false;
		devpriv->timer = true;
		daqgert_ai_start_pacer(dev, true);
		smp_mb__before_atomic();
		set_bit(AI_CMD_RUNNING, &devpriv->state_bits);
		wake_up_interruptible(&daqgert_ai_thread_wq);
		smp_mb__after_atomic();
		devpriv->ai_cmd_canceled = false;
		s->async->inttrig = NULL;
	} else {

		ret = -EBUSY;
	}

	mutex_unlock(&devpriv->cmd_lock);
	return ret;
}

static int32_t daqgert_ao_inttrig(struct comedi_device *dev,
	struct comedi_subdevice *s,
	uint32_t trig_num)
{
	struct daqgert_private *devpriv = dev->private;
	struct comedi_cmd *cmd = &s->async->cmd;
	int32_t ret = 0;

	if (unlikely(!devpriv))
		return -EFAULT;

	if (trig_num != cmd->start_arg)
		return -EINVAL;

	mutex_lock(&devpriv->cmd_lock);
	dev_info(dev->class_dev, "ao inttrig\n");

	if (!test_bit(AO_CMD_RUNNING, &devpriv->state_bits)) {
		smp_mb__before_atomic();
		set_bit(AO_CMD_RUNNING, &devpriv->state_bits);
		wake_up_interruptible(&daqgert_ao_thread_wq);
		smp_mb__after_atomic();
		devpriv->ao_cmd_canceled = false;
		s->async->inttrig = NULL;
	} else {
		ret = -EBUSY;
	}

	mutex_unlock(&devpriv->cmd_lock);
	return ret;
}

static int32_t daqgert_ao_cmd(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	int ret = 0;

	if (unlikely(!devpriv))
		return -EFAULT;

	mutex_lock(&devpriv->cmd_lock);
	dev_info(dev->class_dev, "ao_cmd\n");
	if (test_bit(AO_CMD_RUNNING, &devpriv->state_bits)) {
		dev_info(dev->class_dev, "ao_cmd busy\n");
		ret = -EBUSY;
		goto ao_cmd_exit;
	}

	/* 
	 * inter-spacing speed adjustments update from cmd_test
	 * 
	 * delay between any single conversion 
	 */
	pdata->delay_usecs = pdata->delay_usecs_calc;
	pdata->delay_nsecs = pdata->delay_usecs * NSEC_PER_USEC;
	/* delay for alt mix command conversions */
	pdata->mix_delay_usecs = pdata->mix_delay_usecs_calc;

	/* for possible hunking of AO */
	if (cmd->stop_src == TRIG_COUNT) {
		devpriv->ao_scans = cmd->chanlist_len * cmd->stop_arg;
		devpriv->ao_neverending = false;
	} else {
		devpriv->ao_scans = hunk_len;
		devpriv->ao_neverending = true;
	}

	/* 1ms */
	/* timing of the scan: we set all channels at once */
	devpriv->ao_timer = cmd->scan_begin_arg / 1000;
	if (devpriv->ao_timer < 1) {
		ret = -EINVAL;
		goto ao_cmd_exit;
	}
	devpriv->ao_counter = devpriv->ao_timer;
	s->async->cur_chan = 0;
	daqgert_ao_set_chan_range(dev, cmd->chanlist[s->async->cur_chan], false);

	if (cmd->start_src == TRIG_NOW) {
		s->async->inttrig = NULL;
		/* enable this acquisition operation */
		smp_mb__before_atomic();
		set_bit(AI_CMD_RUNNING, &devpriv->state_bits);
		wake_up_interruptible(&daqgert_ai_thread_wq);
		smp_mb__after_atomic();
		devpriv->ao_cmd_canceled = false;
	} else {
		/* TRIG_INT */

		/* wait for an internal signal */
		s->async->inttrig = daqgert_ao_inttrig;
	}

ao_cmd_exit:
	mutex_unlock(&devpriv->cmd_lock);
	return ret;
}

static int32_t daqgert_ai_cmd(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct comedi_cmd *cmd = &s->async->cmd;
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	int32_t ret = 0, i;

	if (unlikely(!devpriv))
		return -EFAULT;

	mutex_lock(&devpriv->cmd_lock);
	dev_info(dev->class_dev, "ai_cmd\n");
	if (test_bit(AI_CMD_RUNNING, &devpriv->state_bits)) {
		dev_info(dev->class_dev, "ai_cmd busy\n");
		ret = -EBUSY;
		mutex_unlock(&devpriv->cmd_lock);
		return ret;
	}

	/* 
	 * inter-spacing speed adjustments from cmd_test
	 */
	pdata->delay_usecs = pdata->delay_usecs_calc;
	pdata->delay_nsecs = pdata->delay_usecs * NSEC_PER_USEC;
	pdata->mix_delay_usecs = pdata->mix_delay_usecs_calc;

	if (cmd->stop_src == TRIG_COUNT) {
		devpriv->ai_scans = cmd->chanlist_len * cmd->stop_arg;
		devpriv->ai_neverending = false;
	} else {
		devpriv->ai_scans = hunk_len;
		devpriv->ai_neverending = true;
	}
	devpriv->ai_scans_left = devpriv->ai_scans; /* a count down */

	/* 
	 * check if we can use HUNK transfer 
	 */
	if (devpriv->use_hunking && !spi_data->pic18) {
		devpriv->ai_hunk = true;
		devpriv->ai_mix = false;
		/* set single channel hunk chan */
		devpriv->mix_chan = CR_CHAN(cmd->chanlist[0]);
		for (i = 1; i < cmd->chanlist_len; i++) {
			if (cmd->chanlist[0] != cmd->chanlist[i]) {
				/* we might not be able to use HUNK :-( */
				devpriv->ai_hunk = false;
				break;
			}
		}
		/* check for the special mix_mode case */
		if (cmd->chanlist_len == 2 && (cmd->chanlist[0]
			!= cmd->chanlist[1])) {
			devpriv->ai_hunk = true;
			devpriv->ai_mix = true;
			devpriv->mix_chan = CR_CHAN(cmd->chanlist[1]);
			dev_info(dev->class_dev,
				"hunk mix_mode ai transfers enabled\n");
		}
	} else {
		devpriv->ai_hunk = false;
	}

	if (!devpriv->ai_hunk)
		dev_info(dev->class_dev,
		"hunk ai mode transfers disabled\n");

	s->async->cur_chan = 0;
	daqgert_ai_set_chan_range(dev, cmd->chanlist[s->async->cur_chan], false);

	/* want wake up every scan? */
	if (cmd->flags & CMDF_WAKE_EOS) {
		/* HUNK is useless for this situation */
		devpriv->ai_hunk = false;
		dev_info(dev->class_dev,
			"hunk transfers disabled from CMDF_WAKE_EOS\n");
	}
	if (devpriv->timing_lockout) {
		devpriv->ai_hunk = false;
		dev_info(dev->class_dev,
			"hunk transfers disabled from timing lockout\n");
	}

	if (devpriv->ai_hunk) /* run batch conversions in background */
		ret = daqgert_ai_setup_hunk(dev, s, true);
	else
		daqgert_ai_setup_eoc(dev, s);

	if (cmd->start_src == TRIG_NOW) {
		s->async->inttrig = NULL;
		/* enable this acquisition operation */
		devpriv->run = false;
		devpriv->timer = true;
		daqgert_ai_start_pacer(dev, true);
		smp_mb__before_atomic();
		set_bit(AI_CMD_RUNNING, &devpriv->state_bits);
		wake_up_interruptible(&daqgert_ai_thread_wq);
		smp_mb__after_atomic();
		devpriv->ai_cmd_canceled = false;
	} else {
		/* TRIG_INT */
		/* don't enable the acquisition operation */

		/* wait for an internal signal */
		s->async->inttrig = daqgert_ai_inttrig;
	}

	devpriv->timing_lockout = true;
	mutex_unlock(&devpriv->cmd_lock);
	dev_info(dev->class_dev, "ai_cmd return\n");
	return 0;
}

/* 
 * get close to a good sample spacing for one second, 
 * test_mode is to see what the max sample rate is 
 */
static int32_t daqgert_ao_delay_rate(struct comedi_device *dev,
	int32_t rate,
	int32_t device_type,
	bool test_mode)
{
	//	const struct daqgert_board *board = dev->board_ptr;
	struct daqgert_private *devpriv = dev->private;
	int32_t spacing_usecs = 0, sample_freq, total_sample_time, delay_time;
	if (test_mode) {
		dev_info(dev->class_dev,
			"ao speed testing: rate %i, spacing usecs %i\n",
			rate, spacing_usecs);
		return spacing_usecs;
	}

	if (rate <= 0)
		rate = devpriv->ao_spi->device_spi->rate_min;
	if (rate > MAX_BOARD_RATE)
		rate = MAX_BOARD_RATE;
	sample_freq = MAX_BOARD_RATE / rate;
	/* ns time needed for all samples in one second */
	total_sample_time = devpriv->ao_spi->device_spi->rate_min * sample_freq;
	delay_time = MAX_BOARD_RATE - total_sample_time; /* what's left */
	if (delay_time >= sample_freq) { /* something */
		spacing_usecs = (delay_time / sample_freq) / NSEC_PER_USEC;
		if (spacing_usecs < 0)
			spacing_usecs = 0;
	} else { /* or nothing */

		spacing_usecs = 0;
	}
	dev_info(dev->class_dev, "ao rate %i, spacing usecs %i\n", rate, spacing_usecs);
	return spacing_usecs;
}

static int32_t daqgert_ao_cmdtest(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_cmd * cmd)
{
	const struct daqgert_board *board = dev->board_ptr;
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	int32_t i, err = 0;
	uint32_t tmp_timer;

	if (unlikely(!devpriv))
		return -EFAULT;

	/* Step 1 : check if triggers are trivially valid */

	err |= comedi_check_trigger_src(&cmd->start_src, TRIG_NOW | TRIG_INT);
	err |= comedi_check_trigger_src(&cmd->convert_src, TRIG_NOW);
	err |= comedi_check_trigger_src(&cmd->scan_begin_src, TRIG_TIMER);
	err |= comedi_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
	err |= comedi_check_trigger_src(&cmd->stop_src, TRIG_COUNT | TRIG_NONE);

	if (err)
		return 1;

	/* Step 2a : make sure trigger sources are unique */

	err |= comedi_check_trigger_is_unique(cmd->start_src);
	err |= comedi_check_trigger_is_unique(cmd->stop_src);

	/* Step 2b : and mutually compatible */

	if (err)
		return 2;

	/* Step 3: check if arguments are trivially valid */

	err |= comedi_check_trigger_arg_is(&cmd->start_arg, 0);

	if (cmd->scan_begin_src == TRIG_FOLLOW) /* internal trigger */
		err |= comedi_check_trigger_arg_is(&cmd->scan_begin_arg, 0);

	if (cmd->scan_begin_src == TRIG_TIMER) {
		i = 1;
		/* find a power of 2 for the number of channels */
		while (i < (cmd->chanlist_len))
			i = i * 2;
		tmp_timer = (board->ao_ns_min_calc / 2) * i;
		err |= comedi_check_trigger_arg_min(&cmd->scan_begin_arg,
			tmp_timer); /* fastest */
		/* now calc the real sampling rate with all the
		 * rounding errors */
		tmp_timer = ((uint32_t) (cmd->scan_begin_arg
			/ devpriv->ao_spi->device_spi->min_acq_ns)) * devpriv->ao_spi->device_spi->min_acq_ns / 10;
		pdata->delay_usecs_calc = daqgert_ao_delay_rate(dev,
			tmp_timer,
			spi_data->device_type,
			speed_test);
		err |= comedi_check_trigger_arg_max(&cmd->scan_begin_arg,
			MAX_BOARD_RATE);
	} else {
		pdata->delay_usecs_calc = 0;
	}
	pdata->delay_nsecs = pdata->delay_usecs_calc * NSEC_PER_USEC;

	err |= comedi_check_trigger_arg_is(&cmd->scan_end_arg,
		cmd->chanlist_len);

	if (cmd->stop_src == TRIG_COUNT)
		err |= comedi_check_trigger_arg_min(&cmd->stop_arg, 1);
	else /* TRIG_NONE */
		err |= comedi_check_trigger_arg_is(&cmd->stop_arg, 0);

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (err)
		return 4;

	return 0;
}

static int32_t daqgert_ai_poll(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t num_bytes;

	if (unlikely(!devpriv))
		return -EFAULT;

	if (!devpriv->ai_hunk)
		return 0; /* poll is valid only for HUNK transfer */

	dev_info(dev->class_dev, "ai_poll\n");
	num_bytes = comedi_buf_n_bytes_ready(s);
	return num_bytes;
}

/* 
 * get close to a good sample spacing for one second, 
 * test_mode is to see what the max sample rate is 
 */
static int32_t daqgert_ai_delay_rate(struct comedi_device *dev,
	int32_t rate,
	int32_t device_type,
	bool test_mode)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t spacing_usecs = 0, sample_freq, total_sample_time, delay_time;

	if (test_mode) {
		dev_info(dev->class_dev,
			"ai speed testing: rate %i, spacing usecs %i\n",
			rate, spacing_usecs);
		return spacing_usecs;
	}

	if (rate <= 0)
		rate = devpriv->ai_spi->device_spi->rate_min;
	if (rate > MAX_BOARD_RATE)
		rate = MAX_BOARD_RATE;
	sample_freq = MAX_BOARD_RATE / rate;
	total_sample_time = devpriv->ai_spi->device_spi->rate_min * sample_freq;
	delay_time = MAX_BOARD_RATE - total_sample_time;
	if (delay_time >= sample_freq) {
		spacing_usecs = (delay_time / sample_freq) / NSEC_PER_USEC;
		if (spacing_usecs < 0)
			spacing_usecs = 0;
	} else {
		spacing_usecs = 0;
	}
	if (devpriv->use_hunking)
		spacing_usecs += CONV_SPEED_FIX;
	else
		spacing_usecs += CONV_SPEED_FIX_FREERUN;

	if (device_type == mcp3002)
		spacing_usecs += CONV_SPEED_FIX_FAST;
	if (device_type == ads8330)
		spacing_usecs += CONV_ADS8330;
	dev_info(dev->class_dev, "ai rate %i, spacing usecs %i\n", rate, spacing_usecs);
	return spacing_usecs;
}

/*
 * comedi_8254_cascade_ns_to_timer - calculate the cascaded divisor values
 * @i8254:	comedi_8254 struct for the timer
 * @nanosec:	the desired ns time
 * @flags:	comedi_cmd flags
 */
void comedi_8254_cascade_ns_to_timer(struct comedi_8254 *i8254,
	unsigned int *nanosec,
	unsigned int flags)
{
	unsigned int d1 = i8254->next_div1 ? i8254->next_div1 : I8254_MAX_COUNT;
	unsigned int d2 = i8254->next_div2 ? i8254->next_div2 : I8254_MAX_COUNT;
	unsigned int div = d1 * d2;
	unsigned int ns_lub = 0xffffffff;
	unsigned int ns_glb = 0;
	unsigned int d1_lub = 0;
	unsigned int d1_glb = 0;
	unsigned int d2_lub = 0;
	unsigned int d2_glb = 0;
	unsigned int start;
	unsigned int ns;
	unsigned int ns_low;
	unsigned int ns_high;

	/* exit early if everything is already correct */
	if (div * i8254->osc_base == *nanosec &&
		d1 > 1 && d1 <= I8254_MAX_COUNT &&
		d2 > 1 && d2 <= I8254_MAX_COUNT &&
		/* check for overflow */
		div > d1 && div > d2 &&
		div * i8254->osc_base > div &&
		div * i8254->osc_base > i8254->osc_base)
		return;

	div = *nanosec / i8254->osc_base;
	d2 = I8254_MAX_COUNT;
	start = div / d2;
	if (start < 2)
		start = 2;
	for (d1 = start; d1 <= div / d1 + 1 && d1 <= I8254_MAX_COUNT; d1++) {
		for (d2 = div / d1;
			d1 * d2 <= div + d1 + 1 && d2 <= I8254_MAX_COUNT; d2++) {
			ns = i8254->osc_base * d1 * d2;
			if (ns <= *nanosec && ns > ns_glb) {
				ns_glb = ns;
				d1_glb = d1;
				d2_glb = d2;
			}
			if (ns >= *nanosec && ns < ns_lub) {
				ns_lub = ns;
				d1_lub = d1;
				d2_lub = d2;
			}
		}
	}

	switch (flags & CMDF_ROUND_MASK) {
	case CMDF_ROUND_NEAREST:
	default:
		ns_high = d1_lub * d2_lub * i8254->osc_base;
		ns_low = d1_glb * d2_glb * i8254->osc_base;
		if (ns_high - *nanosec < *nanosec - ns_low) {
			d1 = d1_lub;
			d2 = d2_lub;
		} else {

			d1 = d1_glb;
			d2 = d2_glb;
		}
		break;
	case CMDF_ROUND_UP:
		d1 = d1_lub;
		d2 = d2_lub;
		break;
	case CMDF_ROUND_DOWN:
		d1 = d1_glb;
		d2 = d2_glb;
		break;
	}

	*nanosec = d1 * d2 * i8254->osc_base;
	i8254->next_div1 = d1;
	i8254->next_div2 = d2;
}

/* 
 * For some scans we can do a quasi-DMA-like transfer that's much faster and
 * has better long term timing
 */
static int32_t daqgert_ai_cmdtest(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_cmd * cmd)
{
	struct daqgert_private *devpriv = dev->private;
	struct spi_param_type *spi_data = s->private;
	struct spi_device *spi = spi_data->spi;
	struct comedi_spigert *pdata = spi->dev.platform_data;
	int32_t i, err = 0;
	uint32_t arg;
	uint32_t tmp_timer;

	if (unlikely(!devpriv))
		return -EFAULT;

	/* Step 1 : check if triggers are trivially valid */

	err |= comedi_check_trigger_src(&cmd->start_src, TRIG_NOW | TRIG_INT);
	err |= comedi_check_trigger_src(&cmd->scan_begin_src, TRIG_FOLLOW | TRIG_TIMER);
	err |= comedi_check_trigger_src(&cmd->convert_src, TRIG_TIMER);
	err |= comedi_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
	err |= comedi_check_trigger_src(&cmd->stop_src, TRIG_NONE | TRIG_COUNT);

	if (err)
		return 1;

	/* Step 2a : make sure trigger sources are unique */

	err |= comedi_check_trigger_is_unique(cmd->start_src);
	err |= comedi_check_trigger_is_unique(cmd->stop_src);

	/* Step 2b : and mutually compatible */

	if (err)
		return 2;

	/* Step 3: check if arguments are trivially valid */

	err |= comedi_check_trigger_arg_is(&cmd->start_arg, 0);

	if (cmd->scan_begin_src == TRIG_FOLLOW) /* internal trigger */
		err |= comedi_check_trigger_arg_is(&cmd->scan_begin_arg, 0);

	if (cmd->scan_begin_src == TRIG_TIMER) {
		i = 1;
		/* find a power of 2 for the number of channels */
		while (i < (cmd->chanlist_len))
			i = i * 2;
		err |= comedi_check_trigger_arg_min(&cmd->scan_begin_arg,
			devpriv->ai_spi->device_spi->rate_min / 2 * i);
	}

	if (cmd->convert_src == TRIG_TIMER)
		err |= comedi_check_trigger_arg_min(&cmd->convert_arg,
		devpriv->ai_spi->device_spi->rate_min);

	err |= comedi_check_trigger_arg_min(&cmd->chanlist_len, 1);
	err |= comedi_check_trigger_arg_is(&cmd->scan_end_arg,
		cmd->chanlist_len);


	if (cmd->stop_src == TRIG_COUNT)
		err |= comedi_check_trigger_arg_min(&cmd->stop_arg, 1);
	else /* TRIG_NONE */
		err |= comedi_check_trigger_arg_is(&cmd->stop_arg, 0);

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	/*
	 * Not currently used 
	 */
	if (cmd->convert_src == TRIG_NOW) {
		pdata->delay_usecs_calc = 0;
		/* double delay with zero for the first scan chan */
		pdata->mix_delay_usecs_calc = CONV_SPEED_FIX * 2;
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		/* now calc the real sampling rate with all the
		 * rounding errors */
		tmp_timer = ((uint32_t) (cmd->scan_begin_arg
			/ devpriv->ai_spi->device_spi->min_acq_ns)) * devpriv->ai_spi->device_spi->min_acq_ns;
		pdata->delay_usecs_calc = daqgert_ai_delay_rate(dev, tmp_timer,
			spi_data->device_type,
			speed_test);
		/* double delay with zero for the first scan chan */
		pdata->mix_delay_usecs_calc = pdata->delay_usecs_calc * 2;

		/*
		 * dev_info(dev->class_dev, "ai cmd spacing usecs %i, mix %i\n", pdata->delay_usecs, pdata->mix_delay_usecs);
		 */
		arg = cmd->convert_arg * cmd->scan_end_arg;
		err |= comedi_check_trigger_arg_is(&cmd->scan_begin_arg, arg);
	}

	if (cmd->convert_src == TRIG_TIMER) {
		arg = cmd->convert_arg;
		devpriv->pacer.osc_base = devpriv->ai_conv_delay_10nsecs;
		comedi_8254_cascade_ns_to_timer(&devpriv->pacer, &arg,
			cmd->flags);
		pdata->delay_usecs_calc = daqgert_ai_delay_rate(dev, arg,
			spi_data->device_type,
			speed_test);
		/* double delay with zero for the first scan chan */
		pdata->mix_delay_usecs_calc = pdata->delay_usecs_calc * 2;

		/*
		 * dev_info(dev->class_dev, "ai cmd spacing usecs %i, mix %i\n", pdata->delay_usecs, pdata->mix_delay_usecs);
		 */
		err |= comedi_check_trigger_arg_is(&cmd->convert_arg, arg);
	}
	pdata->delay_nsecs = pdata->delay_usecs_calc * NSEC_PER_USEC;
	pdata->mix_delay_usecs = pdata->mix_delay_usecs_calc;

	if (err)
		return 4;

	return 0;
}

/*
 * Possible DMA timer that's not currently useful except for speed benchmarks
 */
static void my_timer_ai_callback(unsigned long data)
{
	struct comedi_device *dev = (void*) data;
	struct daqgert_private *devpriv = dev->private;
	static uint32_t time_marks = 0;

	if (!dev)
		return;

	if (!devpriv->run) {
		devpriv->run = true;
		devpriv->timer = true;
	}
	daqgert_ai_start_pacer(dev, true);
	if (speed_test) {
		if (!(time_marks++ % 100))
			dev_info(dev->class_dev,
			"speed testing %i: ao count %i, ai count %i, hunk %i, "
			"length %i 1Mhz timer value 0x%x:0x%x\n",
			time_marks, devpriv->ao_count, devpriv->ai_count, devpriv->hunk_count, hunk_len,
			(uint32_t) ioread32(devpriv->timer_1mhz + 2),
			(uint32_t) ioread32(devpriv->timer_1mhz + 1));
	}
}

static void daqgert_ai_clear_eoc(struct comedi_device * dev)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t count = 500;

	del_timer_sync(&devpriv->ai_spi->my_timer);
	setup_timer(&devpriv->ai_spi->my_timer, my_timer_ai_callback,
		(unsigned long) dev);
	devpriv->run = false;
	devpriv->timer = false;
	do { /* wait if needed to SPI to clear or timeout */
		schedule(); /* force a context switch */
		usleep_range(750, 1000);
		if (special_test) usleep_range(13000, 15000);
	} while (test_bit(SPI_AI_RUN, &devpriv->state_bits) && (count--));

	devpriv->run = false;
	devpriv->timer = false;
}

static int32_t daqgert_ai_cancel(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;

	if (unlikely(!devpriv))
		return -EFAULT;

	if (unlikely(!test_bit(AI_CMD_RUNNING, &devpriv->state_bits)))
		return 0;

	daqgert_ai_clear_eoc(dev);
	dev_info(dev->class_dev, "ai cancel\n");
	ai_count = devpriv->ai_count;
	hunk_count = devpriv->hunk_count;
	devpriv->ai_hunk = false;
	s->async->cur_chan = 0;
	s->async->inttrig = NULL;
	devpriv->ai_cmd_canceled = true;
	clear_bit(AI_CMD_RUNNING, &devpriv->state_bits);
	smp_mb__after_atomic();
	devpriv->timing_lockout = false;

	return 0;
}

static int32_t daqgert_ao_cancel(struct comedi_device *dev,
	struct comedi_subdevice * s)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t count = 500;

	if (unlikely(!devpriv))
		return -EFAULT;

	if (!test_bit(AO_CMD_RUNNING, &devpriv->state_bits))
		return 0;

	dev_info(dev->class_dev, "ao cancel start\n");
	ao_count = devpriv->ao_count;
	s->async->cur_chan = 0;
	clear_bit(AO_CMD_RUNNING, &devpriv->state_bits);
	smp_mb__after_atomic();
	do { /* wait if needed to SPI to clear or timeout */
		schedule(); /* force a context switch to stop the AO thread */
		usleep_range(750, 1000);
	} while (test_bit(SPI_AO_RUN, &devpriv->state_bits) && (count--));

	usleep_range(750, 1000);
	devpriv->ao_cmd_canceled = true;
	s->async->inttrig = NULL;
	dev_info(dev->class_dev, "ao cancel end\n");
	return 0;
}

/* 
 * 
 * FIXME Slow brute forced IO bits, 5us reads from userland
 * 
 * need to use (fix) state to optimize changes 
 */
static int32_t daqgert_dio_insn_bits(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_insn *insn,
	uint32_t * data)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t pinWPi;
	uint32_t val = 0, mask = 0;

	if (unlikely(!devpriv))
		return -EFAULT;

	/* s->state contains the GPIO bits */
	/* s->io_bits contains the GPIO direction */

	/* i/o testing with gpio pins  */
	/* We need to shift a single bit from state to set or clear the GPIO */
	for (pinWPi = 0; pinWPi < s->n_chan; pinWPi++) {
		mask = comedi_dio_update_state(s, data);
		if (wpi_pin_safe(dev, pinWPi)) {
			/* Do nothing on marked pins */
			if (mask) {
				if (mask & 0xffffffff)
					devpriv->digitalWrite(dev, pinWPi,
					(s->state
					& (0x01 << pinWPi))
					>> pinWPi);
			}
			val = s->state & 0xffffffff;
			val |= (devpriv->digitalRead(dev, pinWPi) << pinWPi);
		}
		data[1] = val;
	}
	return insn->n;
}

/* 
 * query or change DIO config 
 */
static int32_t daqgert_dio_insn_config(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_insn *insn,
	uint32_t * data)
{
	struct daqgert_private *devpriv = dev->private;
	uint32_t wpi_pin = CR_CHAN(insn->chanspec), chan = 1 << wpi_pin;

	if (unlikely(!devpriv))
		return -EFAULT;

	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
		if (wpi_pin_safe(dev, wpi_pin)) {
			s->io_bits |= chan;
			devpriv->pinMode(dev, wpi_pin, OUTPUT);
		}
		break;
	case INSN_CONFIG_DIO_INPUT:
		if (wpi_pin_safe(dev, wpi_pin)) {

			s->io_bits &= (~chan);
			devpriv->pinMode(dev, wpi_pin, INPUT);
			pullUpDnControl(dev, wpi_pin, pullups);
		}
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (s->io_bits & chan) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}
	dev_dbg(dev->class_dev, "%s: gpio pins setting 0x%x\n",
		dev->board_name,
		(uint32_t) s->io_bits);
	return insn->n;
}

/* 
 * Talk to the ADC via the SPI 
 */
static int32_t daqgert_ai_rinsn(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_insn *insn,
	uint32_t * data)
{
	struct daqgert_private *devpriv = dev->private;
	int32_t ret = -EBUSY;
	int32_t n;

	if (unlikely(!devpriv))
		return -EFAULT;

	mutex_lock(&devpriv->cmd_lock);
	if (unlikely(test_bit(AI_CMD_RUNNING, &devpriv->state_bits)))
		goto ai_read_exit;

	devpriv->ai_hunk = false;

	if (devpriv->ai_spi->device_type == ads1220)
		daqgert_ai_set_chan_range_ads1220(dev, s, insn->chanspec);
	if (devpriv->ai_spi->device_type == ads8330)
		daqgert_ai_set_chan_range_ads8330(dev, s, insn->chanspec);

	devpriv->ai_chan = CR_CHAN(insn->chanspec);
	devpriv->ai_range = CR_RANGE(insn->chanspec);

	/* convert n samples */
	for (n = 0; n < insn->n; n++) {
		data[n] = daqgert_ai_get_sample(dev, s);
	}
	ai_count = devpriv->ai_count;
	ret = 0;
ai_read_exit:
	mutex_unlock(&devpriv->cmd_lock);
	return ret ? ret : insn->n;
}

/*
 * does nothing yet
 */
static int32_t daqgert_ai_insn_config(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	struct daqgert_private *devpriv = dev->private;
	int result = -EINVAL;

	if (insn->n < 1)
		return result;

	result = insn->n;

	if (data[1] > 0)
		devpriv->ai_count = devpriv->ai_count;

	return result;
}

/* 
 * write to the DAC via SPI and read the last value back DON't LOCK 
 */
static int32_t daqgert_ao_winsn(struct comedi_device *dev,
	struct comedi_subdevice *s,
	struct comedi_insn *insn,
	uint32_t * data)
{
	struct daqgert_private *devpriv = dev->private;
	uint32_t chan = CR_CHAN(insn->chanspec);
	uint32_t n, val = s->readback[chan];

	daqgert_ao_set_chan_range(dev, insn->chanspec, false);
	for (n = 0; n < insn->n; n++) {

		val = data[n];
		daqgert_ao_put_sample(dev, s, val);
	}
	ao_count = devpriv->ao_count;
	return insn->n;
}

static int32_t daqgert_ai_config(struct comedi_device *dev,
	struct comedi_subdevice * s)
{

	struct spi_param_type *spi_data = s->private;

	/* Stuff here? */
	return spi_data->chan;
}

static int32_t daqgert_ao_config(struct comedi_device *dev,
	struct comedi_subdevice * s)
{

	struct spi_param_type *spi_data = s->private;

	/* Stuff here? */
	return spi_data->chan;
}

/*
 * make two threads for the spi i/o streams
 */
static int32_t daqgert_create_thread(struct comedi_device *dev,
	struct daqgert_private * devpriv)
{
	const char hunk_thread_name[] = "daqgerth", thread_name[] = "daqgert";
	const char *name_ptr;

	if (devpriv->use_hunking)
		name_ptr = hunk_thread_name;
	else
		name_ptr = thread_name;

	devpriv->ai_spi->daqgert_task =
		kthread_create_on_node(&daqgert_ai_thread_function,
		(void *) dev,
		cpu_to_node(devpriv->ai_node),
		"%s_a/%d", name_ptr,
		devpriv->ai_node);
	if (!IS_ERR(devpriv->ai_spi->daqgert_task)) {
		kthread_bind(devpriv->ai_spi->daqgert_task, devpriv->ai_node);
		wake_up_process(devpriv->ai_spi->daqgert_task);
	} else {
		return PTR_ERR(devpriv->ai_spi->daqgert_task);
	}

	devpriv->ao_spi->daqgert_task =
		kthread_create_on_node(&daqgert_ao_thread_function,
		(void *) dev,
		cpu_to_node(devpriv->ao_node),
		"%s_d/%d", name_ptr,
		devpriv->ao_node);
	if (!IS_ERR(devpriv->ao_spi->daqgert_task)) {
		kthread_bind(devpriv->ao_spi->daqgert_task, devpriv->ao_node);
		wake_up_process(devpriv->ao_spi->daqgert_task);
	} else {

		return PTR_ERR(devpriv->ao_spi->daqgert_task);
	}

	return 0;
}

/*
 * when the module is loaded handle the details
 */
static int32_t daqgert_auto_attach(struct comedi_device *dev,
	unsigned long unused_context)
{
	const struct daqgert_board *thisboard = &daqgert_boards[gert_type & 0x01];
	struct comedi_subdevice *s;
	int32_t ret, i;
	unsigned long spi_device_missing = 0;
	int32_t num_ai_chan, num_ao_chan, num_dio_chan = NUM_DIO_CHAN;
	struct daqgert_private *devpriv;
	struct comedi_spigert *pdata;
	struct spi_message m;

	/* 
	 * auto free on exit of comedi module
	 */
	devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
	if (!devpriv)
		return -ENOMEM;

	devpriv->checkmark = CHECKMARK;

	/* set hardware defaults table */
	dev->board_ptr = thisboard;

	devpriv->cpu_nodes = num_online_cpus();
	if (devpriv->cpu_nodes >= 4) {
		dev_info(dev->class_dev, "%d cpu(s) online for threads\n",
			devpriv->cpu_nodes);
		devpriv->ai_node = thisboard->ai_node;
		devpriv->ao_node = thisboard->ao_node;
		devpriv->smp = true;
	} else {
		use_hunking = false;
	}
	if (daqgert_conf == 4 || daqgert_conf == 14 || daqgert_conf == 99) /* single transfers, ADC is in continuous conversion mode */
		use_hunking = false;
	devpriv->use_hunking = use_hunking; /* defaults to true */

	if (speed_test)
		dev_info(dev->class_dev, "samples per second speed test mode\n");

	/*
	 * loop the spi device queue for needed devices
	 */
	if (list_empty(&device_list))
		return -ENODEV;

	/*
	 * set the wanted device bits
	 */
	set_bit(CSnA, &spi_device_missing);
	set_bit(CSnB, &spi_device_missing);

	list_for_each_entry(pdata, &device_list, device_entry)
	{
		/*
		 * use smaller SPI data buffers if we can't hunk
		 */
		if (!devpriv->use_hunking) {
			if (pdata->rx_buff)
				kfree(pdata->rx_buff);
			if (pdata->tx_buff)
				kfree(pdata->tx_buff);
			pdata->tx_buff = kzalloc(SPI_BUFF_SIZE_NOHUNK,
				GFP_DMA);
			if (!pdata->tx_buff) {
				ret = -ENOMEM;
				goto daqgert_kfree_exit;
			}
			pdata->rx_buff = kzalloc(SPI_BUFF_SIZE_NOHUNK,
				GFP_DMA);
			if (!pdata->rx_buff) {
				ret = -ENOMEM;
				goto daqgert_kfree_tx_exit;
			}
		}
		/*
		 * we have a valid device pointer, see which one and 
		 * probe/init hardware for special cases that may need 
		 * many SPI transfers
		 */
		if (pdata->slave.spi->chip_select == thisboard->ai_cs) {
			devpriv->ai_spi = &pdata->slave;
			pdata->one_t.tx_buf = pdata->tx_buff;
			pdata->one_t.rx_buf = pdata->rx_buff;
			if (daqgert_conf == 4 || daqgert_conf == 14) { /* ads1220 mode */
				/* 
				 * setup ads1220 registers
				 */
				pdata->one_t.len = 5;
				pdata->tx_buff[0] = ADS1220_CMD_WREG + 3;
				pdata->tx_buff[1] = ads1220_r0;
				pdata->tx_buff[2] = ads1220_r1;
				pdata->tx_buff[3] = ads1220_r2;
				pdata->tx_buff[4] = ads1220_r3;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				pdata->slave.spi->max_speed_hz = daqgert_devices[ads1220].max_speed_hz;
				pdata->slave.spi->mode = daqgert_devices[ads1220].spi_mode;
				spi_setup(pdata->slave.spi);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
				usleep_range(40, 50);
				pdata->one_t.len = 5;
				pdata->tx_buff[0] = ADS1220_CMD_RREG + 3;
				pdata->tx_buff[1] = 0;
				pdata->tx_buff[2] = 0;
				pdata->tx_buff[3] = 0;
				pdata->tx_buff[4] = 0;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
				usleep_range(40, 50);
				/*
				 * Check to be sure we have a device
				 */
				pdata->slave.device_detect = pdata->rx_buff[2];
				if ((pdata->rx_buff[1] != ads1220_r0) ||
					(pdata->rx_buff[2] != ads1220_r1)) {
					dev_err(dev->class_dev,
						"ADS1220 configuration error: %x %x %x %x\n",
						pdata->rx_buff[1], pdata->rx_buff[2],
						pdata->rx_buff[3], pdata->rx_buff[4]);
					return -ENODEV;
				}
				pdata->one_t.len = 1;
				pdata->tx_buff[0] = ADS1220_CMD_SYNC;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
			}
			if (daqgert_conf == 16 || daqgert_conf == 17) { /* ads8330 mode */
				/* 
				 * setup ads8330 register
				 */
				pdata->one_t.len = 2;
				pdata->one_t.delay_usecs = 0;
				pdata->tx_buff[0] = (ADS8330_CMR_DEFAULT) >> 8; /* software reset */
				pdata->tx_buff[1] = 0;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				pdata->slave.spi->max_speed_hz = daqgert_devices[ads8330].max_speed_hz;
				pdata->slave.spi->mode = daqgert_devices[ads8330].spi_mode;
				spi_setup(pdata->slave.spi);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
				usleep_range(400, 500);
				pdata->one_t.len = 2;
				pdata->one_t.delay_usecs = 0;
				pdata->tx_buff[0] = (ADS8330_CMR_CONF_AUTO) >> 8;
				pdata->tx_buff[1] = ADS8330_CFR_CONF_AUTO;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
				usleep_range(40, 50);
				pdata->one_t.len = 2;
				pdata->one_t.delay_usecs = 0;
				pdata->tx_buff[0] = ADS8330_CMR_RCFR >> 8;
				pdata->tx_buff[1] = 0;
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
				/*
				 * Check to be sure we have a device
				 */
				pdata->slave.device_detect = pdata->rx_buff[1];
				if (pdata->rx_buff[1] != (ADS8330_CFR_CONF_AUTO)) {
					dev_err(dev->class_dev,
						"ADS8330 configuration error: %x %x\n",
						pdata->rx_buff[0], pdata->rx_buff[1]);
					return -ENODEV;
				}
				usleep_range(40, 50);
				pdata->one_t.len = 1;
				pdata->one_t.delay_usecs = 0;
				pdata->tx_buff[0] = ADS8330_CMR_CH0; /* set to channel 0 */
				spi_message_init_with_transfers(&m,
					&pdata->one_t, 1);
				spi_bus_lock(pdata->slave.spi->master);
				spi_sync_locked(pdata->slave.spi, &m);
				spi_bus_unlock(pdata->slave.spi->master);
			}
			clear_bit(CSnA, &spi_device_missing);
		} else {
			devpriv->ao_spi = &pdata->slave;
			pdata->one_t.tx_buf = pdata->tx_buff;
			pdata->one_t.rx_buf = pdata->rx_buff;
			clear_bit(CSnB, &spi_device_missing);
		}
	}

	/*
	 * check for possible bad spigert table entry (dupe)
	 * or missing special case device
	 */
	if (spi_device_missing)
		return -ENODEV;

	/* multi user locking */
	mutex_init(&devpriv->cmd_lock);
	mutex_init(&devpriv->drvdata_lock);

	/* Board  operation data */
	dev->board_name = thisboard->name;
	devpriv->ai_cmd_delay_usecs = PIC18_CMDD_25K22;
	devpriv->ai_conv_delay_usecs = PIC18_CONVD_25K22;
	devpriv->ai_neverending = true;
	devpriv->ai_mix = false;
	devpriv->ai_conv_delay_10nsecs = CONV_SPEED;
	devpriv->timing_lockout = false;

	/* Use the kernel system_rev EXPORT_SYMBOL */
	devpriv->RPisys_rev = system_rev; /* what board are we running on? */
	if (devpriv->RPisys_rev < 2) {
		dev_err(dev->class_dev, "invalid RPi board revision! %u\n",
			devpriv->RPisys_rev);
		return -EINVAL;
	}

	/* 
	 * setup the pins in a static matter for now
	 * PIN mode for all 
	 */
	if (wiringpi) {
		dev_info(dev->class_dev,
			"%s WiringPi pins setup\n", thisboard->name);
		if (wiringPiSetup(dev) < 0) {
			dev_err(dev->class_dev,
				"board gpio detection failed!\n");
			return -EINVAL;
		}
	} else {
		dev_info(dev->class_dev,
			"%s GpioPi pins setup\n", thisboard->name);
		if (wiringPiSetupGpio(dev) < 0) {
			dev_err(dev->class_dev,
				"board gpio detection failed!\n");
			return -EINVAL;
		}
	}

	dev->iobase = GPIO_BASE; /* bcm iobase */
	/* 
	 * dev->mmio is a void pointer with 8bit pointer indexing, 
	 * we need 32bit indexing so mmio is casted to a (__iomem uint32_t*) 
	 * pointer for GPIO R/W operations 
	 */
	dev->mmio = ioremap(dev->iobase, SZ_16K);
	if (!dev->mmio) {
		dev_err(dev->class_dev, "invalid gpio io base address!\n");
		return -EINVAL;
	}

	devpriv->timer_1mhz = ioremap(ST_BASE, 8);
	if (!devpriv->timer_1mhz) {
		dev_err(dev->class_dev, "invalid 1mhz timer base address!\n");
		return -EINVAL;
	}


	devpriv->board_rev = piBoardRev(dev);
	switch (devpriv->board_rev) {
	case 2:
		num_dio_chan = NUM_DIO_CHAN_REV2; /* This a Rev 2 board */
		break;
	case 3:
		num_dio_chan = NUM_DIO_CHAN_REV3; /* This a Rev 3 board */
		break;
	default:
		num_dio_chan = NUM_DIO_CHAN; /* Rev 1 board setup */
	}

	if (wiringpi) {
		for (i = 0; i < NUM_DIO_OUTPUTS; i++) { /* [0..7] OUTPUTS */
			devpriv->pinMode(dev, i, OUTPUT);
		}
		dev_info(dev->class_dev,
			"%s WPi pins set [0..7] to outputs\n",
			thisboard->name);
	}

	dev_info(dev->class_dev,
		"%s device detection started, daqgert_conf option value %i\n",
		thisboard->name, daqgert_conf);
	devpriv->num_subdev = 1;
	if (daqgert_spi_probe(dev, devpriv->ai_spi, devpriv->ao_spi)) {
		devpriv->num_subdev += 2;
	} else {
		dev_err(dev->class_dev, "board device detection failed!\n");
		return -EINVAL;
	}

	/* 
	 * all Comedi buffers default to 32 bits
	 * add AI and AO channels 
	 */
	ret = comedi_alloc_subdevices(dev, devpriv->num_subdev);
	if (ret) {
		dev_err(dev->class_dev, "alloc subdevice(s) failed!\n");
		return ret;
	}

	/* daq_gert dio */
	s = &dev->subdevices[0];
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = num_dio_chan;
	s->len_chanlist = num_dio_chan;
	s->range_table = &range_digital;
	s->maxdata = 1;
	s->insn_bits = daqgert_dio_insn_bits;
	s->insn_config = daqgert_dio_insn_config;
	s->state = 0;

	if (devpriv->num_subdev > 1) { /* setup comedi for on-board devices */
		/* daq_gert ai */
		if (devpriv->use_hunking)
			dev_info(dev->class_dev,
			"hunk ai transfers enabled, length: %i\n",
			hunk_len);
		s = &dev->subdevices[1];
		s->private = devpriv->ai_spi;
		num_ai_chan = daqgert_ai_config(dev, s);
		s->type = COMEDI_SUBD_AI;
		/* default setups, we support single-ended (ground)  */
		s->n_chan = num_ai_chan;
		s->len_chanlist = num_ai_chan;
		s->maxdata = (1 << (thisboard->n_aichan_bits - devpriv->ai_spi->device_spi->n_chan_bits)) - 1;
		if (devpriv->ai_spi->range)
			s->range_table = &daqgert_ai_range2_048;
		else
			s->range_table = &daqgert_ai_range3_300;
		s->insn_read = daqgert_ai_rinsn;
		if (devpriv->smp) {
			s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags;
			s->do_cmdtest = daqgert_ai_cmdtest;
			s->do_cmd = daqgert_ai_cmd;
			s->poll = daqgert_ai_poll;
			s->cancel = daqgert_ai_cancel;
		} else {
			s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags - SDF_CMD_READ;
		}
		if (devpriv->ai_spi->device_type == ads1220) {
			/* we support single-ended (ground) & diff bipolar  24-bit samples */
			/* 32 bit buffers */
			lsamp_size = SDF_LSAMPL;
			s->maxdata = (1 << devpriv->ai_spi->device_spi->n_chan_bits) - 1;
			s->range_table = &range_ads1220_ai;
			s->n_chan = devpriv->ai_spi->device_spi->n_chan;
			s->len_chanlist = devpriv->ai_spi->device_spi->n_chan;
			s->insn_config = daqgert_ai_insn_config;
			if (devpriv->smp) {
				s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags;
				s->do_cmdtest = daqgert_ai_cmdtest;
				s->do_cmd = daqgert_ai_cmd;
				s->poll = daqgert_ai_poll;
				s->cancel = daqgert_ai_cancel;
			} else {
				s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags - SDF_CMD_READ;
			}
		}
		if (devpriv->ai_spi->device_type == ads8330 || devpriv->ai_spi->device_type == special) {
			/* we support single-ended (ground) & diff  16-bit samples */
			s->maxdata = (1 << devpriv->ai_spi->device_spi->n_chan_bits) - 1;
			s->range_table = &daqgert_ai_range3_300;
			s->n_chan = devpriv->ai_spi->device_spi->n_chan;
			s->len_chanlist = devpriv->ai_spi->device_spi->n_chan;
			s->insn_config = daqgert_ai_insn_config;
			if (devpriv->smp) {
				s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags;
				s->do_cmdtest = daqgert_ai_cmdtest;
				s->do_cmd = daqgert_ai_cmd;
				s->poll = daqgert_ai_poll;
				s->cancel = daqgert_ai_cancel;
			} else {
				s->subdev_flags = devpriv->ai_spi->device_spi->ai_subdev_flags - SDF_CMD_READ;
			}
		}

		if (lsamp_size != SDF_LSAMPL) {
			lsamp_size = 0;
			dev_info(dev->class_dev, "16 bit and less device buffers set to 16 bits\n");
		}
		dev->read_subdev = s;

		/* daq-gert ao */
		s = &dev->subdevices[2];
		s->private = devpriv->ao_spi;
		num_ao_chan = daqgert_ao_config(dev, s);
		s->type = COMEDI_SUBD_AO;
		/* we support single-ended (ground)  */
		s->n_chan = num_ao_chan;
		s->len_chanlist = num_ao_chan;
		/* analog resolution depends on the DAC chip 8,10,12 bits */
		s->maxdata = (1 << thisboard->n_aochan_bits) - 1;
		s->range_table = &daqgert_ao_range;
		s->insn_write = daqgert_ao_winsn;
		s->insn_read = comedi_readback_insn_read;
		if (devpriv->smp) {
			s->subdev_flags = devpriv->ao_spi->device_spi->ao_subdev_flags;
			s->do_cmdtest = daqgert_ao_cmdtest;
			s->do_cmd = daqgert_ao_cmd;
			s->cancel = daqgert_ao_cancel;
		} else {
			s->subdev_flags = devpriv->ao_spi->device_spi->ao_subdev_flags - SDF_CMD_WRITE;
		}
		ret = comedi_alloc_subdev_readback(s);
		if (ret) {
			dev_err(dev->class_dev,
				"alloc subdevice readback failed!\n");
			return ret;
		}
	}

	/* 
	 * setup the timer to call my_timer_ai_callback 
	 */
	setup_timer(&devpriv->ai_spi->my_timer, my_timer_ai_callback,
		(unsigned long) dev);
	/* 
	 * setup kthreads on other cores if possible
	 */
	if (devpriv->smp) {
		ret = daqgert_create_thread(dev, devpriv);
		if (ret) {

			dev_err(dev->class_dev, "cpu thread creation failed\n");
			return ret;
		}
	}

	dev_info(dev->class_dev,
		"%s attached: gpio iobase 0x%lx, ioremaps 0x%lx  "
		"0x%lx, io pins 0x%x, 1Mhz timer value 0x%x:0x%x\n",
		dev->driver->driver_name,
		dev->iobase,
		(long unsigned int) dev->mmio,
		(long unsigned int) devpriv->timer_1mhz,
		(uint32_t) s->io_bits,
		(uint32_t) ioread32(devpriv->timer_1mhz + 2),
		(uint32_t) ioread32(devpriv->timer_1mhz + 1));

	return 0;

daqgert_kfree_tx_exit:
	kfree(pdata->tx_buff);
daqgert_kfree_exit:
	return ret;
}

static void daqgert_detach(struct comedi_device * dev)
{
	struct daqgert_private *devpriv = dev->private;

	/* wakeup and kill the threads */
	if (devpriv->smp) {
		if (devpriv->ao_spi->daqgert_task) {
			set_bit(AO_CMD_RUNNING, &devpriv->state_bits);
			wake_up_interruptible(&daqgert_ao_thread_wq);
			kthread_stop(devpriv->ao_spi->daqgert_task);
		}
		devpriv->ao_spi->daqgert_task = NULL;

		if (devpriv->ai_spi->daqgert_task) {

			set_bit(AI_CMD_RUNNING, &devpriv->state_bits);
			wake_up_interruptible(&daqgert_ai_thread_wq);
			kthread_stop(devpriv->ai_spi->daqgert_task);
		}
		devpriv->ai_spi->daqgert_task = NULL;
	}

	del_timer_sync(&devpriv->ai_spi->my_timer);

	iounmap(devpriv->timer_1mhz);
	iounmap(dev->mmio);
	dev_info(dev->class_dev,
		"data i/o counts:  adc %u: dac %u\n",
		devpriv->ai_count, devpriv->ao_count);
	dev_info(dev->class_dev, "daq_gert detached\n");
}

static struct comedi_driver daqgert_driver = {
	.driver_name = "daq_gert",
	.module = THIS_MODULE,
	.auto_attach = daqgert_auto_attach,
	.detach = daqgert_detach,
	.board_name = &daqgert_boards[0].name,
	.num_names = ARRAY_SIZE(daqgert_boards),
	.offset = sizeof(struct daqgert_board),
};

/* 
 * called for each listed spigert device 
 * SO THIS RUNS FIRST, setup basic spi comm parameters here
 * so it defaults to slow speed
 */
static int32_t spigert_spi_probe(struct spi_device * spi)
{
	struct comedi_spigert *pdata;
	int32_t ret;

	pdata = kzalloc(sizeof(struct comedi_spigert), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	spi->dev.platform_data = pdata;
	pdata->tx_buff = kzalloc(SPI_BUFF_SIZE, GFP_DMA);
	if (!pdata->tx_buff) {
		ret = -ENOMEM;
		goto kfree_exit;
	}
	pdata->rx_buff = kzalloc(SPI_BUFF_SIZE, GFP_DMA);
	if (!pdata->rx_buff) {
		ret = -ENOMEM;
		goto kfree_tx_exit;
	}

	/*
	 * Do only two chip selects for the Gertboard 
	 */

	dev_info(&spi->dev,
		"default setup: cd %d: %d Hz: bpw %u, mode 0x%x\n",
		spi->chip_select, spi->max_speed_hz, spi->bits_per_word,
		spi->mode);

	if (spi->chip_select == CSnA) {
		/* 
		 * get a copy of the slave device 0 to share with Comedi 
		 * we need a device to talk to the ADC 
		 * 
		 * create entry into the Comedi device list 
		 */
		INIT_LIST_HEAD(&pdata->device_entry);
		pdata->slave.spi = spi;
		/* 
		 * put entry into the Comedi device list 
		 */
		list_add_tail(&pdata->device_entry, &device_list);
		spi->mode = daqgert_devices[defdev0].spi_mode;
		spi->max_speed_hz = daqgert_devices[defdev0].max_speed_hz;
	}
	if (spi->chip_select == CSnB) {
		/* 
		 * we need a device to talk to the DAC 
		 */
		INIT_LIST_HEAD(&pdata->device_entry);
		pdata->slave.spi = spi;
		list_add_tail(&pdata->device_entry, &device_list);
		spi->mode = daqgert_devices[defdev0].spi_mode;
		spi->max_speed_hz = daqgert_devices[defdev0].max_speed_hz;
	}
	spi->bits_per_word = daqgert_devices[defdev0].spi_bpw;
	spi_setup(spi);

	/* 
	 * Check for basic errors 
	 */
	ret = spi_w8r8(spi, 0); /* check for spi comm error */
	if (ret < 0) {
		dev_err(&spi->dev, "spi comm error\n");
		ret = -EIO;
		goto kfree_rx_exit;
	}

	/* setup comedi part of driver */
	if (spi->chip_select == CSnA) {
		ret = comedi_driver_register(&daqgert_driver);
		if (ret < 0)
			goto kfree_rx_exit;

		if (gert_autoload)
			ret = comedi_auto_config(&spi->master->dev,
			&daqgert_driver, 0);

		if (ret < 0)
			goto kfree_rx_exit;
	}
	return 0;

kfree_rx_exit:
	kfree(pdata->rx_buff);
kfree_tx_exit:
	kfree(pdata->tx_buff);
kfree_exit:
	kfree(pdata);
	return ret;
}

static int32_t spigert_spi_remove(struct spi_device * spi)
{
	struct comedi_spigert *pdata = spi->dev.platform_data;

	if (!list_empty(&device_list)) list_del(&pdata->device_entry);

	if (pdata->rx_buff)
		kfree(pdata->rx_buff);
	if (pdata->tx_buff)
		kfree(pdata->tx_buff);

	if (pdata)
		kfree(pdata);
	dev_info(&spi->dev, "released\n");
	return 0;
}

static struct spi_driver spigert_spi_driver = {
	.driver =
	{
		.name = "spigert",
		.owner = THIS_MODULE,
	},
	.probe = spigert_spi_probe,
	.remove = spigert_spi_remove,
};

/*
 * use table data to setup the SPI hardware
 */
static int32_t daqgert_spi_setup(struct spi_param_type * spi)
{

	spi->spi->max_speed_hz = spi->device_spi->max_speed_hz;
	spi->spi->mode = spi->device_spi->spi_mode;
	return spi_setup(spi->spi);
}

/*
 * setup and probe the spi bus for board devices for channels
 * save the data to the global spi variables
 * return number of channels found
 */
static int32_t daqgert_spi_probe(struct comedi_device * dev,
	struct spi_param_type * spi_adc,
	struct spi_param_type * spi_dac)
{
	int32_t ret = 0, reset;
	const struct daqgert_board *thisboard = dev->board_ptr;

	if (!spi_adc->spi) {
		dev_err(dev->class_dev, "no spi channel detected\n");
		spi_adc->chan = 0;
		spi_dac->chan = 0;
		return 0;
	}

	spi_dac->chan = thisboard->n_aochan;

	switch (daqgert_conf) {
	case 1:
		spi_adc->device_type = mcp3202;
		spi_dac->device_type = mcp4822;
		break;
	case 2:
		spi_adc->device_type = mcp3002;
		spi_dac->device_type = mcp4822;
		break;
	case 3:
		spi_adc->device_type = mcp3202;
		spi_dac->device_type = mcp4802;
		break;
	case 4:
		spi_adc->device_type = ads1220;
		spi_dac->device_type = mcp4822;
		break;
	case 14:
		spi_adc->device_type = ads1220;
		spi_dac->device_type = mcp4802;
		break;
	case 16:
		spi_adc->device_type = ads8330;
		spi_dac->device_type = mcp4822;
		break;
	case 17:
		spi_adc->device_type = ads8330;
		spi_dac->device_type = mcp4802;
		break;
	case 99:
		spi_adc->device_type = special;
		spi_dac->device_type = mcp4822;
		special_test = true;
		break;
	default:
		spi_adc->device_type = mcp3002;
		spi_dac->device_type = mcp4802;
	}
	spi_adc->device_spi = &daqgert_devices[spi_adc->device_type];
	spi_dac->device_spi = &daqgert_devices[spi_dac->device_type];

	/* set AO spi */
	daqgert_spi_setup(spi_dac);

	/* default setup */
	spi_adc->pic18 = 0;
	spi_adc->chan = 2;

	if ((spi_adc->device_type != ads1220) && (spi_adc->device_type != ads8330)) {
		/* 
		 * SPI data transfers, send a few dummies for config info 
		 * probes
		 */
		daqgert_spi_setup(spi_adc);
		spi_w8r8(spi_adc->spi, CMD_DUMMY_CFG);
		spi_w8r8(spi_adc->spi, CMD_DUMMY_CFG);

		switch (daqgert_conf) {
		case 5:
			ret = 76; /* P8722 slave mode */
			dev_info(dev->class_dev,
				"force p8722 slave mode\n");
			break;
		case 6:
			ret = 110; /* P25k22 slave mode */
			dev_info(dev->class_dev,
				"force p25k22 slave mode\n");
			break;
		default:
			ret = spi_w8r8(spi_adc->spi, CMD_DUMMY_CFG);
		}

		if ((ret != 76) && (ret != 110)) { /* PIC slave adc codes */
			spi_adc->pic18 = 0; /* MCP3X02 mode */
			spi_adc->chan = thisboard->n_aichan;
			spi_adc->range = 0; /* range 2.048 */
			dev_info(dev->class_dev,
				"onboard %s detected with %s, %i channels, "
				"range code %i, device code %i, "
				"detect code %i\n",
				spi_adc->device_spi->name,
				spi_dac->device_spi->name,
				spi_adc->chan,
				spi_adc->range, spi_adc->device_type,
				ret);
		}

		if (ret == 76 || ret == 110) {
			daqgert_spi_setup(spi_adc);
			spi_adc->pic18 = 1; /* PIC18 single-end mode 10 bits */
			spi_adc->device_type = picsl10;
			spi_adc->chan = ret & 0x0f;
			spi_adc->range = (ret & 0x20) >> 5;
			spi_adc->bits = (ret & 0x10) >> 4;
			if (spi_adc->bits) {
				spi_adc->pic18 = 2; /* PIC24 mode 12 bits */
				spi_adc->device_type = picsl12;
			}
			dev_info(dev->class_dev,
				"PIC %s slave adc detected with %s, "
				"%i channels, range code %i, device code %i, "
				"bits code %i, PIC code %i, detect Code %i\n",
				spi_adc->device_spi->name,
				spi_dac->device_spi->name,
				spi_adc->chan, spi_adc->range, spi_adc->device_type,
				spi_adc->bits, spi_adc->pic18, ret);
		}
	} else {
		if (spi_adc->device_type == ads1220) {
			daqgert_spi_setup(spi_adc);
			reset = ADS1220_CMD_RESET;
			spi_write(spi_adc->spi, &reset, 1);
			usleep_range(300, 350);
			spi_adc->pic18 = 1; /* ACP1220 mode */
			spi_adc->chan = spi_adc->device_spi->n_chan;
			spi_adc->range = 0; /* N/A range 2.048 default */
			spi_adc->bits = spi_adc->device_spi->n_chan_bits;
		}
		if (spi_adc->device_type == ads8330) {
			daqgert_spi_setup(spi_adc);
			reset = ADS8330_CMR_DEFAULT;
			spi_write(spi_adc->spi, &reset, 1);
			usleep_range(300, 350);
			spi_adc->pic18 = 0; /* ACP8330 mode */
			spi_adc->chan = spi_adc->device_spi->n_chan;
			spi_adc->range = 0; /* N/A range Vdd default */
			spi_adc->bits = spi_adc->device_spi->n_chan_bits;
		}
		if (spi_adc->device_type == special) {
			spi_adc->pic18 = 0; /* special mode */
			spi_adc->chan = spi_adc->device_spi->n_chan;
			spi_adc->range = 0; /* N/A range Vdd default */
			spi_adc->bits = spi_adc->device_spi->n_chan_bits;
		}
		dev_info(dev->class_dev,
			"%s adc detected with %s, "
			"%i channels, range code %i, device code %i, "
			"bits code %i, PIC code %i, detect Code 0x%x\n",
			spi_adc->device_spi->name,
			spi_dac->device_spi->name,
			spi_adc->chan, spi_adc->range, spi_adc->device_type,
			spi_adc->bits, spi_adc->pic18, spi_adc->device_detect);
	}

	dev_info(dev->class_dev,
		"board setup: spi cd %d: %d Hz: mode 0x%x: "
		"assigned to adc device %s\n",
		spi_adc->spi->chip_select,
		spi_adc->spi->max_speed_hz,
		spi_adc->spi->mode,
		spi_adc->device_spi->name);
	dev_info(dev->class_dev,
		"board setup: spi cd %d: %d Hz: mode 0x%x: "
		"assigned to dac device %s\n",
		spi_dac->spi->chip_select,
		spi_dac->spi->max_speed_hz,
		spi_dac->spi->mode,
		spi_dac->device_spi->name);

	return spi_adc->chan;
}

static int32_t __init daqgert_init(void)
{
	return spi_register_driver(&spigert_spi_driver);
}
module_init(daqgert_init);

static void __exit daqgert_exit(void)
{

	struct comedi_spigert *pdata;
	static struct spi_param_type *slave_spi;

	/* 
	 * find the needed spi device for module shutdown 
	 */
	list_for_each_entry(pdata, &device_list, device_entry)
	{
		slave_spi = &pdata->slave;
	}

	comedi_auto_unconfig(&slave_spi->spi->master->dev);
	comedi_driver_unregister(&daqgert_driver);
	spi_unregister_driver(&spigert_spi_driver);
}
module_exit(daqgert_exit);

MODULE_AUTHOR("Fred Brooks <spam@sma2.rain.com>");
MODULE_DESCRIPTION("RPi DIO/AI/AO Driver");
MODULE_VERSION("4.12.0");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spigert");

