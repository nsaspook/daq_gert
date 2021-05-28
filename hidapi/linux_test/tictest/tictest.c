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
	 buf[4] bit 3 = GPIO 3
	 buf[4] bit 4 = CS 4    MCP23S08
	 buf[4] bit 5 = GPIO 5
	 buf[4] bit 6 = GPIO 6
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

unsigned char buf[64]; // command buffer writen to MCP2210
unsigned char rbuf[64]; // response buffer read from MCP2210
char snd = 0; // data to send to LCD display
int res = 0; // # of bytes sent from hid_read(), hid_write() functions

hid_device *handle; // handle of device returned by hid_open()

#define MAX_STR 255
wchar_t wstr[MAX_STR]; // buffer for id settings strings from MPC2210
struct hid_device_info *devs, *cur_dev;

void cbufs(void)
{
	memset(buf, 0, sizeof(buf)); // initialize bufs to zeros
	memset(rbuf, 0, sizeof(rbuf));
}

int SendUSBCmd(hid_device *handle, uint8_t *cmdBuf, uint8_t *responseBuf)
{
	int r = 0;
	r = hid_write(handle, cmdBuf, 64);
	if (r < 0) return ERROR_UNABLE_TO_WRITE_TO_DEVICE;

	//when the hid device is configured as synchronous, the first 
	//hid_read returns the desired results. and the while() loop
	//is skipped.
	//
	//when the hid device is configured as asynchronous, the first
	//hid_read may or may not succeed, depending on the latency
	//of the attached device. When no data is returned, r = 0 and
	//the while loop keeps polling the returned data until it is 
	//received.
	r = 0;
	r = hid_read(handle, responseBuf, 64);
	if (r < 0) return ERROR_UNABLE_TO_READ_FROM_DEVICE;

	while (r == 0) {
		r = hid_read(handle, responseBuf, 64);
		if (r < 0) return ERROR_UNABLE_TO_READ_FROM_DEVICE;
		sleep_us(1000);
	}

	return responseBuf[1];
}

int nanosleep(const struct timespec *, struct timespec *);

void sleep_us(unsigned long microseconds)
{
	struct timespec ts;
	ts.tv_sec = microseconds / 1000000; // whole seconds
	ts.tv_nsec = (microseconds % 1000000) * 1000; // remainder, in nanoseconds
	nanosleep(&ts, NULL);
}

bool SPI5_WriteRead(unsigned char* pTransmitData, size_t txSize, unsigned char* pReceiveData, size_t rxSize)
{
	static uint32_t tx_count = 0;

	cbufs();
	buf[0] = 0x40; // SPI transfer settings command
	buf[4] = 0xc0; // set SPI transfer bit rate;
	buf[5] = 0xc6; // 32 bits, lsb = buf[4], msb buf[7]
	buf[6] = 0x2d; // 2d
	buf[7] = 0x00;
	buf[8] = 0xff; // set CS idle values to 1
	buf[9] = 0x01;
	buf[10] = 0x00; // set CS active values to 0
	buf[11] = 0x00;
	buf[18] = txSize; // set no of bytes to transfer = 4 // 32-bit transfers
	buf[20] = 0x01; // spi mode 1

	res = SendUSBCmd(handle, buf, rbuf);
	printf("TX set trans %x\n", res);
	if (res == 0xf8) {
		sleep_us(5000);
		res = SendUSBCmd(handle, buf, rbuf);
		printf("Again TX set trans %x\n", res);
	}

	cbufs();
	buf[0] = 0x42; // transfer SPI data command
	buf[1] = rxSize; // no. of SPI bytes to transfer
	buf[4] = pTransmitData[3];
	buf[5] = pTransmitData[2];
	buf[6] = pTransmitData[1];
	buf[7] = pTransmitData[0];
	//	    buf[4] = 0x0f;
	//	    buf[5] = 0xf0;
	//	    buf[6] = 0x0f;
	//	    buf[7] = 0xf0;

	res = SendUSBCmd(handle, buf, rbuf);
	printf("TX SPI res %x - tx count %i\n", res, ++tx_count);

	pReceiveData[3] = rbuf[4];
	pReceiveData[2] = rbuf[5];
	pReceiveData[1] = rbuf[6];
	pReceiveData[0] = rbuf[7];
	return true;
}

bool hidrawapi_mcp2210_init(void)
{
	if (hid_init()) {
		return false;
	}

	//	devs = hid_enumerate(0x0, 0x0);
	//	cur_dev = devs;
	//	while (cur_dev) {
	//		printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
	//		printf("\n\n");
	//		printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
	//		printf("  Product:      %ls\n", cur_dev->product_string);
	//		printf("  Release:      %hx\n", cur_dev->release_number);
	//		printf("  Interface:    %d\n", cur_dev->interface_number);
	//		printf("\n");
	//		cur_dev = cur_dev->next;
	//	}
	//	hid_free_enumeration(devs);

	// Set up the command buffer -- memset() writes 0x00's to buf

	memset(buf, 0x00, sizeof(buf)); // buf initialized to all zeros

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
	hid_set_nonblocking(handle, 1); // async

	//	return true;

	//-------------- Set GPIO pin function (0x21) -------------
	cbufs();
	buf[0] = 0x21; // command 21 - set GPIO pin's functions

	// function: 0x00 = gpio, 0x01 = CS, 0x02 = dedicated function
	// with buf all zeros, all 9 GPIO pins are set to GPIO's
	//	buf[13] = 0xff;
	//	buf[14] = 0xff;
	//	buf[11] = 0x00; // release SPI bus
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
	memset(buf, 0, 32); // buf initialized to zeros
	memset(rbuf, 0, sizeof(rbuf)); // rbuf initialized to all zeros

	// MCP23S08 config
	buf[0] = 0x42; // transfer SPI data command
	buf[1] = 3; // no. of SPI bytes to transfer
	buf[4] = 0x40; //device address is 01000A1A0, write
	buf[5] = 0x00; //write to IODIR register,
	buf[6] = 0x00; //set all outputs to low

	res = hid_write(handle, buf, 7);
	res = hid_read(handle, rbuf, 7);
}

void setup_mcp23s08_transfer(void)
{
	//-------------- Set GPIO pin function (0x21) -------------
	memset(buf, 0, 32); // buf initialized to zeros
	buf[0] = 0x21; // command 21 - set GPIO pin's functions
	buf[8] = 0x01; // GPIO 4 set to 0x01 - SPI CS

	res = hid_write(handle, buf, 18); // write pin function settings into MCP2210
	if (res < 0) {
		printf("Unable to write()\n");
		printf("Error: error setting pin function %ls\n", hid_error(handle));
	}
	res = hid_read(handle, rbuf, 2); // read the 0x21 response

	memset(buf, 0, sizeof(buf)); // initialize buf to zeros
	buf[0] = 0x40; // SPI transfer settings command

	buf[4] = 0x40; // set SPI transfer bit rate;
	buf[5] = 0x4b; // 32 bits, lsb = buf[4], msb buf[7]
	buf[6] = 0x4c; // 5Mhz
	buf[7] = 0x00;
	buf[8] = 0xff; // set CS idle values to 1
	buf[9] = 0x01;
	buf[10] = 0x00; // set CS active values to 0
	buf[11] = 0x00;
	buf[18] = 0x03; // set no of bytes to transfer = 3
	buf[20] = 0x00; // spi mode 0

	res = hid_write(handle, buf, 21); // write setting into MCP2210
	if (res < 0) {
		printf("Unable to write()\n");
		printf("Error: setting SPI transfer settings %ls\n", hid_error(handle));
	}
	res = hid_read(handle, rbuf, 2); // read the 0x40 response from MCP2210
}

void setup_tic12400_transfer(void)
{
	cbufs();
	buf[0] = 0x21; // command 21 - set GPIO pin's functions
	buf[8] = 0x01; // GPIO 4 set to 0x01 - SPI CS
	res = SendUSBCmd(handle, buf, rbuf);

	cbufs();
	buf[0] = 0x40; // SPI transfer settings command
	buf[4] = 0xc0; // set SPI transfer bit rate;
	buf[5] = 0xc6; // 32 bits, lsb = buf[4], msb buf[7]
	buf[6] = 0x2d; // 2d
	buf[7] = 0x00;
	buf[8] = 0xff; // set CS idle values to 1
	buf[9] = 0x01;
	buf[10] = 0x00; // set CS active values to 0
	buf[11] = 0x00;
	buf[18] = 0x4; // set no of bytes to transfer = 4 // 32-bit transfers
	buf[20] = 0x01; // spi mode 1
	res = SendUSBCmd(handle, buf, rbuf);
}

void get_mcp23s08_transfer(void)
{
	// ---------- Get SPI transfer settings (0x41)-------------

	buf[0] = 0x41; // 0x41 Get SPI transfer settings
	res = hid_write(handle, buf, 2); // write setting into MCP2210
	if (res < 0) {
		printf("Unable to write()\n");
		printf("Error: getting SPI transfer settings %ls\n", hid_error(handle));
	}

	res = hid_read(handle, rbuf, 21); // read the 0x41 response from MCP2210
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
	/*
	 * setup the hidraw* device to communicate with the MCP2210
	 */
	if (!hidrawapi_mcp2210_init()) {
		return -1;
	}

	/*
	 * handle the TIC12400 chip MCP2210 SPI setting
	 */
	setup_tic12400_transfer();
	get_tic12400_transfer();

	/*
	 * init and read 24 switch inputs from the TIC12400 chip
	 */
	memset(buf, 0, sizeof(buf)); // initialize buf to zeros
	buf[0] = 0x11; // 0x11 cancel SPI transfer
	hid_write(handle, buf, 64);

	tic12400_reset();

	if (!tic12400_init()) {
		printf("tic12400_init failed\n");
	}

	while (1) {
	//	tic12400_interrupt(0, 0);
		sleep_us(200000);
	}

	/*
	 * handle the MCP23S08 chip MCP2210 SPI setting
	 */
	setup_mcp23s08_transfer();

	/*
	 * handle the MCP23S08 chip setting
	 */
	mcp23s08_init();

	/*
	 * update the MCP23S08 outputs
	 */
	buf[4] = 0x40;
	buf[5] = 0x0a;

	while (1) { // blink LED loop
		for (int k = 0; k < 10; k++) {
			//lights up LED0 through LED7 one by one
			for (int i = 0; i < 8; i++) {
				buf[6] = 1 << i;
				res = hid_write(handle, buf, 7);
				res = hid_read(handle, rbuf, 7);
				sleep_us(20000ul);
			}
			//lights up LED7 through LED0 one by one
			for (int i = 0; i < 8; i++) {
				buf[6] = 0x80 >> i;
				res = hid_write(handle, buf, 7);
				res = hid_read(handle, rbuf, 7);
				sleep_us(20000ul);
			}
		}
	}
	hid_close(handle);
	hid_exit(); /* Free static HIDAPI objects. */
	return 0;
}
