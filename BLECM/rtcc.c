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
 * File:        rtcc.c
 * Date:        September 23, 2015
 * Compiler:    XC16 v1.23
 *
 */

#include <xc.h>
#include "app.h"
#include "config.h"

#ifdef USE_SLEEP            //see config.h, Application settings section
#ifdef SLEEP_MODE_RTCC

extern APP_DATA appData;

void RTCC_Init(void) {
    __builtin_disi(0x3FFF);     //disable interrupts

    // Set the RTCWREN bit
    __builtin_write_RTCWEN();
   
    RCFGCALbits.RTCEN = 0;  //disable RTCC

    // set date Mon Jun 29 17:18:47 MST 2015
    RCFGCALbits.RTCPTR = 3; // start the sequence
    RTCVAL = 0x15; // YEAR
    RTCVAL = 0x629; // MONTH-1/DAY-1
    RTCVAL = 0x117; // WEEKDAY/HOURS
    RTCVAL = 0x1847; // MINUTES/SECONDS

    // set alarm Mon Jan 01 23:59:00 MST 2001
    ALCFGRPTbits.ALRMPTR = 2;       //sart write sequence
    ALRMVAL = 0x101;
    ALRMVAL = 0x123;
    ALRMVAL = 0x5900;
    
    ALCFGRPTbits.ALRMEN = 1;        //enable alarm and chime for every second
    ALCFGRPTbits.ARPT = 0xFF;
    ALCFGRPTbits.CHIME = 1;
    ALCFGRPTbits.AMASK = 0x2;
    
    // PWCPOL disabled; PWCEN disabled; RTCLK LPRC; PWCPRE disabled; RTCOUT alarm pulse; PWSPRE disabled; 
    RTCPWCbits.RTCLK = 0b01;

    //clear RTCWREN
    RCFGCALbits.RTCWREN = 0;

    __builtin_disi(0);    //enable interrupts    
}

void _ISR_NO_AUTO_PSV _RTCCInterrupt(void) {
    IFS3bits.RTCIF = 0;
    appData.RTCCalarm = true;
}


#endif  //SLEEP_MODE_RTCC
#endif  //USE_SLEEP
