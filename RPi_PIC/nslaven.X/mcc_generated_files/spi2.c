/**
  SPI2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    spi2.c

  @Summary
    This is the generated driver implementation file for the SPI2 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides implementations for driver APIs for SPI2.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F47Q84
        Driver Version    :  3.0.0
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above or later
        MPLAB             :  MPLAB X 6.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "spi2.h"
#include <xc.h>

void SPI2_DefaultHandler(void);
void SPI2_DefaultRxHandler(void);
void SPI2_DefaultTxHandler(void);

typedef struct { 
    uint8_t con0; 
    uint8_t con1; 
    uint8_t con2; 
    uint8_t baud; 
    uint8_t operation;
} spi2_configuration_t;

//con0 == SPIxCON0, con1 == SPIxCON1, con2 == SPIxCON2, baud == SPIxBAUD, operation == Master/Slave
static const spi2_configuration_t spi2_configuration[] = {   
    { 0x0, 0x20, 0x4, 0x0, 1 }
};

void SPI2_Initialize(void)
{
    //EN disabled; LSBF MSb first; MST bus slave; BMODE last byte; 
    SPI2CON0 = 0x00;
    //SMP Middle; CKE Idle to active; CKP Idle:High, Active:Low; FST disabled; SSP active high; SDIP active high; SDOP active high; 
    SPI2CON1 = 0x20;
    //SSET enabled; TXR not required for a transfer; RXR data is not stored in the FIFO; 
    SPI2CON2 = 0x04;
    //CLKSEL FOSC; 
    SPI2CLK = 0x00;
    //BAUD 0; 
    SPI2BAUD = 0x00;
    TRISBbits.TRISB3 = 1;
    PIE5bits.SPI2IE = 1;
    PIE5bits.SPI2RXIE = 1;
    PIE5bits.SPI2TXIE = 1;
    SPI2_SetInterruptHandler(SPI2_DefaultHandler);
    SPI2_SetRxInterruptHandler(SPI2_DefaultRxHandler);
    SPI2_SetTxInterruptHandler(SPI2_DefaultTxHandler);
}

bool SPI2_Open(spi2_modes_t spi2UniqueConfiguration)
{
    if(!SPI2CON0bits.EN)
    {
        SPI2CON0 = spi2_configuration[spi2UniqueConfiguration].con0;
        SPI2CON1 = spi2_configuration[spi2UniqueConfiguration].con1;
        SPI2CON2 = spi2_configuration[spi2UniqueConfiguration].con2 | (_SPI2CON2_SPI2RXR_MASK | _SPI2CON2_SPI2TXR_MASK);
        SPI2BAUD = spi2_configuration[spi2UniqueConfiguration].baud;        
        TRISBbits.TRISB3 = spi2_configuration[spi2UniqueConfiguration].operation;
        SPI2CON0bits.EN = 1;
        return true;
    }
    return false;
}

void SPI2_Close(void)
{
    SPI2CON0bits.EN = 0;
}

uint8_t SPI2_ExchangeByte(uint8_t data)
{
    SPI2TCNTL = 1;
    SPI2TXB = data;
    while(!PIR5bits.SPI2RXIF);
    return SPI2RXB;
}

void SPI2_ExchangeBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        SPI2TCNTL = 1;
        SPI2TXB = *data;
        while(!PIR5bits.SPI2RXIF);
        *data++ = SPI2RXB;
    }
}

// Half Duplex SPI Functions
void SPI2_WriteBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        SPI2_ExchangeByte(*data++);
    }
}

void SPI2_ReadBlock(void *block, size_t blockSize)
{
    uint8_t *data = block;
    while(blockSize--)
    {
        *data++ = SPI2_ExchangeByte(0);
    }
}

void SPI2_WriteByte(uint8_t byte)
{
    SPI2TXB = byte;
}

uint8_t SPI2_ReadByte(void)
{
    return SPI2RXB;
}

void SPI2_DefaultHandler(void)
{
    // add your SPI2 interrupt custom code
}

void SPI2_SetInterruptHandler(spi2InterruptHandler_t handler)
{
    SPI2_InterruptHandler = handler;
}

void __interrupt(irq(SPI2),base(8)) SPI2_Isr(void)
{
    if(SPI2_InterruptHandler)
    {
        SPI2_InterruptHandler();
    }
}

void SPI2_DefaultRxHandler(void)
{
    // add your SPI2RX interrupt custom code
}

void SPI2_SetRxInterruptHandler(spi2InterruptHandler_t handler)
{
    SPI2_RxInterruptHandler = handler;
}

void __interrupt(irq(SPI2RX),base(8)) SPI2_RxIsr(void)
{
    if(SPI2_RxInterruptHandler)
    {
        SPI2_RxInterruptHandler();
    }
}

void SPI2_DefaultTxHandler(void)
{
    // add your SPI2TX interrupt custom code
}

void SPI2_SetTxInterruptHandler(spi2InterruptHandler_t handler)
{
    SPI2_TxInterruptHandler = handler;
}

void __interrupt(irq(SPI2TX),base(8)) SPI2_TxIsr(void)
{
    if(SPI2_TxInterruptHandler)
    {
        SPI2_TxInterruptHandler();
    }
}
