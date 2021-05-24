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
bool SPI5_WriteRead (unsigned char* pTransmitData, size_t txSize, unsigned char* pReceiveData, size_t rxSize);

#ifdef __cplusplus
}
#endif

#endif /* TICTEST_H */

