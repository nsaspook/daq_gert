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

	void sleep_us(unsigned long);
	bool SPI5_WriteRead(unsigned char* pTransmitData, size_t txSize, unsigned char* pReceiveData, size_t rxSize);
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
