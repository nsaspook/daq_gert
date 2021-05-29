/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   tictest.h
 * Author: root
 *
 * Created on May 23, 2021, 7:23 PM
 */

#ifndef TICTEST_H
#define TICTEST_H

#ifdef __cplusplus
extern "C" {
#endif
#include <time.h>
#include <hidapi/hidapi.h>

	/*
	 * debugging print enable
	 */
	//#define DPRINT

#define OPERATION_SUCCESSFUL 0
#define ERROR_UNABLE_TO_OPEN_DEVICE -1
#define ERROR_UNABLE_TO_WRITE_TO_DEVICE -2
#define ERROR_UNABLE_TO_READ_FROM_DEVICE -3
#define ERROR_INVALID_DEVICE_HANDLE -99

#define COMMAND_BUFFER_LENGTH 64
#define RESPONSE_BUFFER_LENGTH 64

#define SPI_STATUS_FINISHED_NO_DATA_TO_SEND 0x10
#define SPI_STATUS_STARTED_NO_DATA_TO_RECEIVE 0x20
#define SPI_STATUS_SUCCESSFUL 0x30 

	void cbufs();
	int32_t SendUSBCmd(hid_device *, uint8_t *, uint8_t *);
	void sleep_us(uint32_t);
	bool get_MCP2210_ext_interrupt(void);
	int32_t cancel_spi_transfer(void);
	bool SPI_WriteRead(hid_device *, uint8_t *, uint8_t *);
	bool SPI_MCP2210_WriteRead(uint8_t* pTransmitData, size_t txSize, uint8_t* pReceiveData, size_t rxSize);
	void setup_tic12400_transfer(void);
	void get_tic12400_transfer(void);
	void setup_mcp23s08_transfer(void);
	void get_mcp23s08_transfer(void);
	void mcp23s08_init(void);
	bool hidrawapi_mcp2210_init(void);

#ifdef __cplusplus
}
#endif

#endif /* TICTEST_H */

