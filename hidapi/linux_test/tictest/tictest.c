/* ****************************************************

 Windows HID simplification

 Alan Ott
 Signal 11 Software

 8/22/2009

 Copyright 2009

 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.

 *******************************************************

  MCP2210 HID programming example using C with the HIDAPI
  library.

 * talks to the MCP3204 and MCP23S08

  Command response data from the MCP2210 is sent to
  the terminal.

  The commands that this program uses configure the MCP2210's
  VM ram only. The NVM is left unchanged.

 *   -------------- connections to the MCP2210 for ADM00420 -----------------

	 buf[4] bit 0 = CS 0    EEPROM CS
	 buf[4] bit 1 = CS 1    MCP3204 CS pin
	 buf[4] bit 2 = GPIO 2
	 buf[4] bit 3 =	FUNC2	SPI ACTIVE LED
	 buf[4] bit 4 = CS 4    MCP23S08
	 buf[4] bit 5 = CS 5	TIC12400
	 buf[4] bit 6 = FUNC2	Ext Interrupt counter
	 buf[4] bit 7 = CS 7    temp chip CS
	 buf[5] bit 0 = GPIO 8

 *******************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <hidapi/hidapi.h>
#include "tictest.h"
#include "tic12400.h"

uint8_t buf[COMMAND_BUFFER_LENGTH]; // command buffer written to MCP2210
uint8_t rbuf[RESPONSE_BUFFER_LENGTH]; // response buffer
int32_t res = 0; // # of bytes sent from hid_read(), hid_write() functions

hid_device *handle; // handle of device returned by hid_open()

#define MAX_STR 255
wchar_t wstr[MAX_STR]; // buffer for id settings strings from MPC2210
struct hid_device_info *devs, *cur_dev;

void cbufs(void)
{
	memset(buf, 0, sizeof(buf)); // initialize bufs to zeros
	memset(rbuf, 0, sizeof(rbuf));
}

int32_t SendUSBCmd(hid_device *handle, uint8_t *cmdBuf, uint8_t *responseBuf)
{
	int32_t r;

	r = hid_write(handle, cmdBuf, COMMAND_BUFFER_LENGTH);
	if (r < 0) {
		return ERROR_UNABLE_TO_WRITE_TO_DEVICE;
	}

	//when the hid device is configured as synchronous, the first 
	//hid_read returns the desired results. and the while() loop
	//is skipped.
	//
	//when the hid device is configured as asynchronous, the first
	//hid_read may or may not succeed, depending on the latency
	//of the attached device. When no data is returned, r = 0 and
	//the while loop keeps polling the returned data until it is 
	//received.
	r = hid_read(handle, responseBuf, RESPONSE_BUFFER_LENGTH);
	if (r < 0) {
		return ERROR_UNABLE_TO_READ_FROM_DEVICE;
	}

	while (r == 0) {
		r = hid_read(handle, responseBuf, RESPONSE_BUFFER_LENGTH);
		if (r < 0) {
			return ERROR_UNABLE_TO_READ_FROM_DEVICE;
		}
		sleep_us(10);
	}

	return responseBuf[1];
}

int32_t nanosleep(const struct timespec *, struct timespec *);

void sleep_us(uint32_t microseconds)
{
	struct timespec ts;
	ts.tv_sec = microseconds / 1000000; // whole seconds
	ts.tv_nsec = (microseconds % 1000000) * 1000; // remainder, in nanoseconds
	nanosleep(&ts, NULL);
}
/*
 * when connected to the TIC12400 interrupt pin it shows a switch has changed state
 */
bool get_MCP2210_ext_interrupt(void)
{
	cbufs();
	buf[0] = 0x12; // Get (VM) the Current Number of Events From the Interrupt Pin, GPIO 6 FUNC2
	buf[1] = 0x00; // reads, then resets the event counter
	res = SendUSBCmd(handle, buf, rbuf);
	if (rbuf[4] || rbuf[5]) {
		return true;
	}
	return false;
}

int32_t cancel_spi_transfer(void)
{
	cbufs();
	buf[0] = 0x11; // 0x11 cancel SPI transfer
	res = SendUSBCmd(handle, buf, rbuf);
	return res;
}

bool SPI_WriteRead(hid_device *handle, uint8_t *buf, uint8_t *rbuf)
{
	res = SendUSBCmd(handle, buf, rbuf);
	while (rbuf[3] == SPI_STATUS_STARTED_NO_DATA_TO_RECEIVE || rbuf[3] == SPI_STATUS_SUCCESSFUL) {
		res = SendUSBCmd(handle, buf, rbuf);
	}
	return true;
}

bool SPI_MCP2210_WriteRead(uint8_t* pTransmitData, size_t txSize, uint8_t* pReceiveData, size_t rxSize)
{
	static uint32_t tx_count = 0;
	uint32_t rcount = 0;

	cbufs();
	buf[0] = 0x42; // transfer SPI data command
	buf[1] = rxSize; // no. of SPI bytes to transfer
	buf[4] = pTransmitData[3];
	buf[5] = pTransmitData[2];
	buf[6] = pTransmitData[1];
	buf[7] = pTransmitData[0];
	res = SendUSBCmd(handle, buf, rbuf);
#ifdef DPRINT
	printf("TX SPI res %x - tx count %i\n", res, ++tx_count);
#endif

	rcount = 0;
	while (rbuf[3] == SPI_STATUS_STARTED_NO_DATA_TO_RECEIVE || rbuf[3] == SPI_STATUS_SUCCESSFUL) {
#ifdef DPRINT
		printf("SPI RX wait %i: code %x\n", ++rcount, rbuf[3]);
#endif
		res = SendUSBCmd(handle, buf, rbuf);
	}
	pReceiveData[3] = rbuf[4];
	pReceiveData[2] = rbuf[5];
	pReceiveData[1] = rbuf[6];
	pReceiveData[0] = rbuf[7];
	return true;
}

bool hidrawapi_mcp2210_init(void)
{
	if (hid_init()) {
		printf("hidapi init error\n");
		return false;
	}

	cbufs(); // clear command and response buffers

	//------------------ Open MCP2210 and display info ---------------

	printf("Open Device: 04d8:00de\n");
	// Open the device using the VID(vendor ID, PID(product ID),
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0xde, NULL); // open the MCP2210 device
	if (!handle) {
		printf("unable to open the MCP2210\n");
		return 1;
	}
	// Read the Manufacturer String
	wstr[0] = 0x0000;
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	if (res < 0) {
		printf("Unable to read manufacturer string\n");
	}
	printf("Manufacturer String: %ls\n", wstr);

	// Read the Product String
	wstr[0] = 0x0000;
	res = hid_get_product_string(handle, wstr, MAX_STR);
	if (res < 0) {
		printf("Unable to read product string\n");
	}
	printf("Product String: %ls\n", wstr);

	// Read the Serial Number String
	wstr[0] = 0x0000;
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	if (res < 0) {
		printf("Unable to read serial number string\n");
	}
	printf("Serial Number String: (%d) %ls", wstr[0], wstr);
	printf("\n");

	// Read Indexed String 1
	wstr[0] = 0x0000;
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	if (res < 0) {
		printf("Unable to read indexed string 1\n");
	}
	printf("Indexed String 1: %ls\n\n", wstr);
	hid_set_nonblocking(handle, 1); // async operation, don't block

	//-------------- Set GPIO pin function (0x21) -------------
	cbufs();
	buf[0] = 0x21; // command 21 - set GPIO pin's functions
	buf[7] = 0x02; // act led
	res = SendUSBCmd(handle, buf, rbuf);

	// ------------ Set GPIO pin direction (0x32)--------------
	cbufs();
	buf[0] = 0x32; // command 32 - set GPIO pin's directions
	// function:  0 = output, 1 = input
	buf[4] = 0x00; // set GPIO 0-7 to outputs
	buf[5] = 0x00; // set GPIO 8 to output
	res = SendUSBCmd(handle, buf, rbuf);

	// ------------ Set GPIO pin level (0x30)--------------
	cbufs();
	buf[0] = 0x30; // command 30 - set GPIO pin's level to all high
	buf[4] = 0xff;
	buf[5] = 0xff;
	res = SendUSBCmd(handle, buf, rbuf);

	printf("Ctrl c to exit\n"); // ctrl c to exit blink loop and exit program
	return true;
}

void mcp23s08_init(void)
{
	cbufs();
	// MCP23S08 config
	buf[0] = 0x42; // transfer SPI data command
	buf[1] = 3; // no. of SPI bytes to transfer
	buf[4] = 0x40; //device address is 01000A1A0, write
	buf[5] = 0x00; //write to IODIR register,
	buf[6] = 0x00; //set all outputs to low
	res = SendUSBCmd(handle, buf, rbuf);
}

void setup_mcp23s08_transfer(void)
{
	//-------------- Set GPIO pin function (0x21) -------------
	cbufs();
	buf[0] = 0x21; // command 21 - set GPIO pin's functions
	buf[7] = 0x02; // SPI act led
	buf[4] = 0x01; // GPIO 0 set to 0x01 - SPI CS
	buf[5] = 0x01; // GPIO 1 set to 0x01 - SPI CS
	buf[8] = 0x01; // GPIO 4 set to 0x01 - SPI CS, mcp23s08
	buf[9] = 0x01; // GPIO 5 set to 0x01 - SPI CS
	buf[10] = 0x02; // GPIO 6 external interrupt input
	buf[11] = 0x01; // GPIO 7 set to 0x01 - SPI CS
	res = SendUSBCmd(handle, buf, rbuf);

	cbufs();
	buf[0] = 0x40; // SPI transfer settings command
	buf[4] = 0x40; // set SPI transfer bit rate;
	buf[5] = 0x4b; // 32 bits, lsb = buf[4], msb buf[7]
	buf[6] = 0x4c; // 5MHz
	buf[7] = 0x00;
	buf[8] = 0xff; // set CS idle values to 1
	buf[9] = 0x01;
	buf[10] = 0b11101111; // set CS active values to 0, set the rest to 1
	buf[11] = 0b00000001;
	buf[18] = 0x03; // set no of bytes to transfer = 3
	buf[20] = 0x00; // spi mode 0
	res = SendUSBCmd(handle, buf, rbuf);
}

void setup_tic12400_transfer(void)
{
	cbufs();
	buf[0] = 0x21; // command 21 - set GPIO pin's functions
	buf[7] = 0x02; // SPI act led
	buf[4] = 0x01; // GPIO 0 set to 0x01 - SPI CS
	buf[5] = 0x01; // GPIO 1 set to 0x01 - SPI CS
	buf[8] = 0x01; // GPIO 4 set to 0x01 - SPI CS
	buf[9] = 0x01; // GPIO 5 set to 0x01 - SPI CS, tic12400
	buf[10] = 0x02; // GPIO 6 external interrupt input
	buf[11] = 0x01; // GPIO 7 set to 0x01 - SPI CS
	res = SendUSBCmd(handle, buf, rbuf);

	cbufs();
	buf[0] = 0x40; // SPI transfer settings command
	buf[4] = 0x00; // set SPI transfer bit rate;
	buf[5] = 0x09; // 32 bits, lsb = buf[4], msb buf[7]
	buf[6] = 0x3d; // 4MHz
	buf[7] = 0x00;
	buf[8] = 0xff; // set CS idle values to 1
	buf[9] = 0x01;
	buf[10] = 0b11011111; // set CS active values to 0, set the rest to 1
	buf[11] = 0b00000001;
	buf[18] = 0x4; // set no of bytes to transfer = 4 // 32-bit transfers
	buf[20] = 0x01; // spi mode 1
	res = SendUSBCmd(handle, buf, rbuf);
}

void get_mcp23s08_transfer(void)
{
	// ---------- Get SPI transfer settings (0x41)-------------
	cbufs();
	buf[0] = 0x41; // 0x41 Get SPI transfer settings
	res = SendUSBCmd(handle, buf, rbuf);
	printf("SPI MCP23S08 transfer settings\n   "); // Print out the 0x41 returned buffer.
	for (int i = 0; i < rbuf[2]; i++) {
		printf("%02hhx ", rbuf[i]);
	}
	printf("\n");
}

void get_tic12400_transfer(void)
{
	// ---------- Get SPI transfer settings (0x41)-------------
	cbufs();
	buf[0] = 0x41; // 0x41 Get SPI transfer settings
	res = SendUSBCmd(handle, buf, rbuf);
	printf("SPI TIC12400 transfer settings\n   "); // Print out the 0x41 returned buffer.
	for (int i = 0; i < rbuf[2]; i++) {
		printf("%02hhx ", rbuf[i]);
	}
	printf("\n");
}

int main(int argc, char* argv[])
{
	uint32_t fspeed = 20000; // led movement speed

	/*
	 * setup the hidraw* device to communicate with the MCP2210
	 */
	if (!hidrawapi_mcp2210_init()) {
		return -1;
	}

	cancel_spi_transfer(); // cleanup
	/*
	 * handle the TIC12400 chip MCP2210 SPI setting
	 */
	setup_tic12400_transfer();
	get_tic12400_transfer();

	/*
	 * init and program 24 switch inputs from the TIC12400 chip
	 */
	tic12400_reset();
	if (!tic12400_init()) {
		printf("tic12400_init failed\n");
	}

	/*
	 * we need to change SPI speed, mode, transfer size and cs as we switch to different devices.
	 */
	while (true) { // blink LED loop
		/*
		 * handle the MCP23S08 chip MCP2210 SPI setting
		 */
		setup_mcp23s08_transfer(); // CS 4 and mode 0
		/*
		 * handle the MCP23S08 chip setting
		 */
		mcp23s08_init();
		/*
		 * SPI data to update the MCP23S08 outputs
		 */
		buf[4] = 0x40;
		buf[5] = 0x0a;

		/*
		 * light show sequence
		 */
		for (int k = 0; k < 10; k++) {
			//lights up LED0 through LED7 one by one
			for (int i = 0; i < 8; i++) {
				buf[6] = 1 << i;
				SPI_WriteRead(handle, buf, rbuf);
				sleep_us(fspeed);
			}
			//lights up LED7 through LED0 one by one
			for (int i = 0; i < 8; i++) {
				buf[6] = 0x80 >> i;
				SPI_WriteRead(handle, buf, rbuf);
				sleep_us(fspeed);
			}
		}
		/*
		 * handle the TIC12400 chip MCP2210 SPI setting
		 */
		setup_tic12400_transfer(); // CS 5 and mode 1
		/*
		 * read 24 switch inputs after light show sequence
		 */
		tic12400_read_sw(0, 0);
		/*
		 * look for switch 0 changes
		 */
		if (tic12400_get_sw() & raw_mask_0) {
			fspeed = 20000;
		} else {
			fspeed = 2000;
		}
	}
	hid_close(handle);
	hid_exit(); /* Free static HIDAPI objects. */
	return 0;
}
