/*
 * Copyright (C) 2014 Microchip Technology Inc. and its subsidiaries.  You may use this software and any derivatives
 * exclusively with Microchip products.
 *
 * MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any derivatives created by any person or
 * entity by or on your behalf, exclusively with Microchip?s products.  Microchip and its licensors retain all ownership
 * and intellectual property rights in the accompanying software and in all derivatives hereto.
 *
 * This software and any accompanying information is for suggestion only.  It does not modify Microchip?s standard
 * warranty for its products.  You agree that you are solely responsible for testing the software and determining its
 * suitability.  Microchip has no obligation to modify, test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,
 * BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
 * APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN
 * ANY APPLICATION.
 *
 * IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
 * STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
 * TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
 * SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
 *
 *
 * File:        uart.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * Uart functions
 * 
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uart.h"
#include "config.h"

//UART receive buffer type
typedef struct {
    volatile uint8_t buffer[SIZE_RxBuffer];
    volatile uint8_t volatile *head;
    volatile uint8_t *tail;
    volatile uint16_t byteCount;
} UART_RX_BUFFER_T;

//UART transmit buffer type
typedef struct {
    volatile uint8_t buffer[SIZE_TxBuffer];
    volatile uint8_t *head;
    volatile uint8_t volatile *tail;
    volatile uint16_t byteCount;
} UART_TX_BUFFER_T;

//Buffer instances
static UART_RX_BUFFER_T rxBuf;
static UART_TX_BUFFER_T txBuf;

//**********************************************************************************************************************
// Initialize the UART to communicate with the Bluetooth module
void UART_Init(void)
{
    rxBuf.tail = &rxBuf.buffer[0];     //Initialize the pointers
    rxBuf.head = &rxBuf.buffer[0];
    txBuf.tail = &txBuf.buffer[0];
    txBuf.head = &txBuf.buffer[0];
    txBuf.byteCount = rxBuf.byteCount = 0;
    
    U1BRG = 34;                             //Baud rate 115,200 - actually 115,384 baud 0.16% error
    U1MODE = 0x8008;                        //Enable UART, no flow control, BRGH = 1 for high speed baud mode
    U1STA = 0x0400;                         //Enable transmit
    UART_RX_IF = 0;                         //Clear UART interrupt flags
    UART_ER_IF = 0;
    UART_RX_IE = 1;                         //Enable UART Receive and Error interrupt
    UART_ER_IE = 1;
}

//**********************************************************************************************************************
// Start transmission by enabling the UART transmit interrupt
inline void UART_TxStart(void)
{
    UART_TX_IE = 1;                    //Enable transmit interrupts
}

//**********************************************************************************************************************
// See if there are one or more bytes in the receive buffer
bool UART_IsNewRxData(void)
{
    __builtin_disi(0x3FFF);             //disable interrupts
    if(rxBuf.byteCount == 0) {          //Check if data in buffer
        __builtin_disi(0);              //enable interrupts
        return(false);                  //No bytes in the buffer so return false
    }
    __builtin_disi(0);          //enable interrupts
    return(true);               //There are bytes in the buffer
}

//**********************************************************************************************************************
// Read a byte from the receive buffer
uint8_t UART_ReadRxBuffer(void)
{
    uint8_t Temp;

    __builtin_disi(0x3FFF);     //disable interrupts
    if(rxBuf.byteCount == 0) {              //For safety, do not allow read of empty buffer
        __builtin_disi(0);                  //enable interrupts
        return(0);                          //Return zero if there is nothing in the buffer
    }

    rxBuf.byteCount--;          //Decrement byte count
    __builtin_disi(0);          //enable interrupts
    Temp = *rxBuf.tail++;                   //Get the byte and increment the pointer
    if (rxBuf.tail > &rxBuf.buffer[SIZE_RxBuffer - 1]) { //Check if at end of buffer
        rxBuf.tail = &rxBuf.buffer[0];          //then wrap the pointer to beginning
    }
    return(Temp);
}

//**********************************************************************************************************************
// Write a byte to the transmit buffer
void UART_WriteTxBuffer(const uint8_t TxByte)
{
    *txBuf.head++ = TxByte;                 //Put the byte in the transmit buffer and increment the pointer
    if (txBuf.head > &txBuf.buffer[SIZE_TxBuffer - 1]) { //Check if at end of buffer
        txBuf.head = &txBuf.buffer[0];          //Wrap pointer to beginning
    }
    __builtin_disi(0x3FFF);     //disable interrupts
    if(txBuf.byteCount < SIZE_TxBuffer) {     //Increment byte count
        txBuf.byteCount++;
    }
    __builtin_disi(0);    //enable interrupts
}

//**********************************************************************************************************************
// Return the number of bytes free in the TX buffer
uint16_t UART_GetTXBufferFreeSpace(void) {
    uint16_t space;

    __builtin_disi(0x3FFF);     //disable interrupts            
    space = SIZE_TxBuffer - txBuf.byteCount;
    __builtin_disi(0);          //enable interrupts
    return space;
}

//Peek at buffer tail
uint8_t UART_PeekRxBuffer(void) {
    __builtin_disi(0x3FFF);     //disable interrupts
    if(rxBuf.byteCount == 0) {              //Check if pointers are the same
        __builtin_disi(0);                  //enable interrupts
        return(NULL);                      //No bytes in the buffer so return NULL
    }
    else {
        __builtin_disi(0);                  //enable interrupts
        return *rxBuf.tail;
    }
}

//**********************************************************************************************************************
// Interrupt routine for UART receive interrupts
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    UART_RX_IF = 0;                    //Clear UART Receive interrupt flag
    *rxBuf.head++ = UART_RX_BUF;       //Put received byte in the buffer
    if (rxBuf.head > &rxBuf.buffer[SIZE_RxBuffer - 1]) {  //Check if end of buffer
        rxBuf.head = &rxBuf.buffer[0];     //Wrap pointer to beginning
    }
    if(rxBuf.byteCount < SIZE_RxBuffer) {     //Increment byte count
        rxBuf.byteCount++;
    }
}

//**********************************************************************************************************************
// Interrupt routine for UART transmit interrupts
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    if(txBuf.byteCount > 0)           //Check if more data is in the buffer
    {
        //Only do anything if hardware buffer has space
        if(!UART_FULL) {
            UART_TX_IF = 0;                         //Clear UART 1 TX interrupt flag
            UART_TX_BUF = *txBuf.tail++;            //Load next byte into the TX buffer
            if (txBuf.tail > &txBuf.buffer[SIZE_TxBuffer - 1]) {  //Check if end of buffer
                txBuf.tail = &txBuf.buffer[0];      //Wrap pointer to beginning
            }
            txBuf.byteCount--;     //Decrement byte count
        }
    }
    else {
        UART_TX_IE = 0;                //No more data to transmit, so stop interrupts
    }
}

//**********************************************************************************************************************
// Interrupt routine for UART error interrupts
void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void) {
    IFS4bits.U1ERIF = 0;        //Clear interrupt flag
    
    //Handle an overflow error by reading next byte and clearing flag
    if(U1STAbits.OERR == 1) {
        *rxBuf.head++ = UART_RX_BUF;       //Put received byte in the buffer
        if (rxBuf.head > &rxBuf.buffer[SIZE_RxBuffer - 1]) {  //Check if end of buffer
            rxBuf.head = &rxBuf.buffer[0];     //Wrap pointer to beginning
        }
        if(rxBuf.byteCount < SIZE_RxBuffer) {     //Increment byte count
            rxBuf.byteCount++;
        }
        U1STAbits.OERR = 0;
    }

    //Clear any other error bits
    U1STAbits.FERR = 0;
    U1STAbits.PERR = 0;
}
