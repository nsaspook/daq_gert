/*
 * Copyright (C) 2015 Microchip Technology Inc. and its subsidiaries.  You may use this software and any derivatives
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
 * File:        comparator.c
 * Date:        September 18, 2015
 * Compiler:    XC16 v1.23
 *
 * Comparator functions
 *
 */

#include <xc.h>
#include "app.h"
#include "config.h"
#include "timers.h"

//State machine for comparator
//Returns next timer value
uint16_t CMP_Tasks(void) {
    if(CM2CONbits.CON == 0) {       //Modules are off - turn them on
        CVRCONbits.CVREN = 1;       //Enable CVref
        CM2CONbits.CON = 1;         //Enable CMP1
        return BAT_CHK_WAIT_MS;     //Return module stabilization timer value
    }
    else {      //Modules are on and stable - time to turn off
        LED7 = CMSTATbits.C2OUT;        //Update LED status first
        CM2CONbits.CON = 0;
        CVRCONbits.CVREN = 0;
        return BAT_CHK_DELAY_MS;        //return next reading delay timer value
    }
}

//Initialize CMP2 for low input voltage detection
void CMP_Init(void) {
    //CVref config
    ANCFGbits.VBG2EN = 0;       //Vbg2 disabled

    CVRCONbits.CVREFP = 0;      //CVR is Vref
    CVRCONbits.CVREFM = 0b00;   //Vbg to comparators
    CVRCONbits.CVROE = 0;       //CVref not output to pin
    CVRCONbits.CVRSS = 0;       //CVrsrc is Vdd-Vss
    CVRCONbits.CVR = CVR_BITS;  //CVR<4:0>
    CVRCONbits.CVREN = 1;       //Enable module

    //Comparator 2 config
    CMSTATbits.CMIDL = 0;       //Continue operation in idle mode
    
    CM2CONbits.COE = 0;         //Not output on C2OUT pin
    CM2CONbits.CPOL = 0;        //Non-inverted output
    CM2CONbits.EVPOL = 0b11;    //Trigger on either edge
    CM2CONbits.CREF = 1;        //Non-inverting input is CVref
    CM2CONbits.CCH = V_SENSE_CMP_CHAN;      //Inverting input is input voltage sense
    CM2CONbits.CEVT = 0;        //Clear event bit
    
    WaitMs(200);               //Wait for CVref and input voltage cap to stabilize
    
    CM2CONbits.CON = 1;         //Enable module
    
    IFS1bits.CMIF = 0;          //Clear IF
    IEC1bits.CMIE = 1;          //Enable comparator interrupt
}

//Comparator interrupt
void _ISR_NO_AUTO_PSV _CompInterrupt(void) {
    IFS1bits.CMIF = 0;              //clear flags
    CM2CONbits.CEVT = 0;
    //Create hysteresis by adjusting CVref voltage output
    if(CM2CONbits.CON && CMSTATbits.C2OUT) {
        CVRCONbits.CVR = CVR_BITS + 1;      //detected low voltage - increase CVref
    }
    else if(CM2CONbits.CON) {
        CVRCONbits.CVR = CVR_BITS;          //voltage above threshold - reset CVref
    }
}
