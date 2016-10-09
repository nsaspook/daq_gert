daq_gert
========
Comedi driver for RPi ai, ao gpio for the  gertboard daq_gert.c
Driver: "experimental" protocol driver daq_gert in progress ... for 4.4+ kernels with DT
on the RPi2/3 with the bcm2835 SPI master with async Comedi commands and triggers
The single processor version for the RPi has only the analog sample capability with no commands
**********
Device-tree operation:  Use overlay rpi-spigert-overlay.dtb
to load the device module in /boot/config.txt: 
dtparam=spi=on
dtoverlay=rpi-spigert-overlay.dtb
dtoverlay=spi0-hw-cs
**********
spi0-hw-cs is needed to assign the chip select HW pins

The DAQ-GERT appears in Comedi as a  digital I/O subdevice (0) with
17 or 21 or 30 channels, 
a analog input subdevice (1) with a Mux for bipolar input voltages
2 differential-ended channels: (0) inputs 0-1, (1) inputs 2-3
2 single-ended channels: (2) (3) for inputs 2 or 3 
a internal short of inputs for a offset reading: (4) with the ADC1220 adc, OR
a analog input subdevice (1) with 2 single-ended channels with onboard adc, OR
a analog input subdevice (1) with single-ended channels set by the SPI slave device
and a analog output subdevice(2) with 2 channels with onboard dac


Current Status:
Mainly works


Notes:
This driver currently requires a small kernel patch to gain direct SPI access at the kernel level
with an optional cs_change_usecs patch to spi.c and spi.h 
needed to reduce delays in cs_change=true transfers
comment out #define CS_CHANGE_USECS in the daq_gert.c source if you don't use this part of the patch

 * git clone https://github.com/raspberrypi/linux.git in /usr/src for the latest
 * linux kernel source tree
 * 
 * cd to the linux kernel source directory: /usr/src/linux etc...
 * copy the daq_gert.diff patch file from the daq_gert directory to the source
 * directory 
 * copy RPix.config_4.x.y from the daq_gert directory to .config in the Linux source directory
 * or
 * copy RPi.config_4.x.y from the daq_gert directory to .config in the Linux source directory
 * 
 * patch the kernel source with the daq_gert.diff patch file
 * patch -p1 <daq_gert.diff
 * copy the daq_gert.c source file to drivers/staging/comedi/drivers
 * edit the /boot/config.txt file to add:
 * dtoverlay=rpi-spigert-overlay.dtb
 * dtoverlay=spi0-hw-cs
 * so on boot the system will disable the spi_dev protocol interface and use the spigert protocol instead
 * 
 *  make menuconfig or xconfig
 *  select SPI_COMEDI=m and SPI_DEV =m in SPI MASTERS to enable the SPI side of the driver 
 *  select DAQ_GERT=m to select the Comedi protocol part of the driver in the 
 *  staging daq comedi misc drivers section.
 *  make -j4 for a RPi 2/3 to compile a new kernel and driver in much less time
 *  use the instructions here to build the new kernel, modules, device-trees and overlays.
 *  https://www.raspberrypi.org/documentation/linux/kernel/building.md
 *
 *  after the reboot: daq_gert should auto-load to device /dev/comedi0*
 *  if the legacy option is set in /etc/modprobe.d/comedi.conf the new device will be created after those.
 *  dmesg should see the kernel module messages from daq_gert
 *  run the test program: bmc_test_program to see if it's working
 * 
 * 
The input  range is 0 to 1023/4095 for 0.0 to 3.3(Vdd) onboard devices, 2.048 volts/Vdd for PIC slaves 
or +-0.512, +-1.024, +-2.048 for the ADS1220 device with usable range in double %2.6fV format for output
The output range is 0 to 4095 for 0.0 to 2.048 onboard devices (output resolution depends on the device)
 * In the async command mode transfers can be handled in HUNK mode by creating a SPI message
 * of many conversion sequences into one message, this allows for close to native driver wire-speed 

Analog: The type and resolution of the onboard ADC/DAC chips are set
by the module option variable daqgert_conf in the /etc/modprobe.d directory

 * options daq_gert daqgert_conf=1 gert_autoload=1
 * 
daqgert_conf options:
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

gert_autolocal options:
0 = don't autoload (mainly for testing)
1 = load and configure daq_gert on boot (default)

All modules parameters can read in  /sys/module/daq_gert/parameters


Comments:
Things are looking pretty good for the MCP3X02 ADC driver section of the code. 
I can get a 30usec per sample avg over a 1 second burst period using xoscope 
with 26usecs for wire-speed and ~5 for CPU overhead without DMA for 12bit reads. 
Without DMA the output data stream is not totally continuous at full speed as I 
have to stop the SPI transfer to process and copy data (about 500us to process 1000 samples).
With a driver request for less than full speed sample rates the software inserts 
calculated delays between samples (or groups of samples during a two channel scan) 
to adjust the sample to the correct time-splice.

*  PIC Slave Info:
Currently on runs in a slow polled mode with the PIC18 but the plan is to offload 
commands to it with a PIC24 version with 12bit samples at much higher speeds 
and auto sequencing.

* ADS1220 mode is for slow ~20 Sps of low level analog data

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

