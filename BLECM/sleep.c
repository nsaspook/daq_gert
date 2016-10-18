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
 * File:        sleep.c
 * Date:        September 17, 2015
 * Compiler:    XC16 v1.23
 *
 *
 */

#include "config.h"

#ifdef USE_SLEEP                //see config.h, Application setting section

#include <xc.h>
#include <stdint.h>
#include "sleep.h"
#include "app.h"
#include "leds.h"

extern APP_DATA appData;

//Enter and exit sleep mode
void APP_SleepNow(void) {
    uint8_t ADCstate,
            CMPstate,
            CVRstate;
    
    BT_WAKE_SW = 0;                 //put RN module to sleep
    BT_WAKE_HW = 0;

    LED_SET_LightShow(LED_SLEEP);   //update LEDs
    LED_Tasks();

    T5CONbits.TON = 0;              //disable timers
    T2CONbits.TON = 0;

    ADCstate = ADCON1bits.ADON;     //save peripheral states and turn them off
    ADCON1bits.ADON = 0;
    CMPstate = CM2CONbits.CON;
    CM2CONbits.CON = 0;
    CVRstate = CVRCONbits.CVREN;
    CVRCONbits.CVREN = 0;

#ifdef SLEEP_MODE_RTCC     //see config.h, Application settings section
    //Use RTCC for periodic wakeup
    __builtin_disi(0x3FFF);     //disable interrupts
    __builtin_write_RTCWEN();   //set the RTCWREN bit to allow writes
    RCFGCALbits.RTCEN = 1;      //enable RTCC
    RCFGCALbits.RTCWREN = 0;    //clear RTCWREN

    //Switch sys clock to LPRC
    __builtin_write_OSCCONH(0x05);              //Set the NOSC bits to LPRC
    __builtin_write_OSCCONL(OSCCONL | 0x01);    //Set the OSWEN bit

    __builtin_disi(0);          //enable interrupts    
    IEC3bits.RTCIE = 1;
    while(OSCCONbits.OSWEN);    //wait for clock switch to complete
    
    appData.CNint = false;
    //Sleep state loop
    do {
        appData.RTCCalarm = false;      //clear RTCC wakeup flag
        Sleep();                        //put micro to sleep
        if(!appData.RTCCalarm || appData.CNint) {break;} //something other than RTCC woke us
        ALCFGRPTbits.AMASK = 0;         //change alarm time to half second
        LED6 = 1;                       //to keep LED on for half a second
        Sleep();                        //return to sleep
        LED6 = 0;                       //woke by half second alarm - LED off
        ALCFGRPTbits.AMASK = 2;         //reset alarm to 10 second interval
    } while(appData.RTCCalarm);     //go back to sleep if woken up by RTCC

    //Done with sleeping (something other than RTCC woke us up)
    __builtin_disi(0x3FFF);                     //disable interrupts    

    //Return sys clock to FRCPLL
    __builtin_write_OSCCONH(0x01);              //Set the NOSC bits to FRCPLL
    __builtin_write_OSCCONL(OSCCONL | 0x01);    //Set the OSWEN bit

    IEC3bits.RTCIE = 0;           //disable RTCC and interrupt
    RCFGCALbits.RTCEN = 0;
    
#else //not SLEEP_MODE_RTCC
    //Use Timer 1 for periodic wakeup
    __builtin_disi(0x3FFF);     //disable interrupts

    //Switch sys clock to LPRC
    __builtin_write_OSCCONH(0x05);              //Set the NOSC bits to LPRC
    __builtin_write_OSCCONL(OSCCONL | 0x01);    //Set the OSWEN bit

    __builtin_disi(0);          //enable interrupts    

    while(OSCCONbits.OSWEN);    //wait for clock switch to complete
    T1CONbits.TON = 1;          //Start timer 1 for periodic wakeup
    
    appData.CNint = false;
    //Sleep state loop
    do {
        appData.timer1Flag = false;         //clear Timer1 wakeup flag
        Sleep();                        //put micro to sleep
        if(appData.timer1Flag == false || appData.CNint) {    //Something other than Timer1 woke us
            break;
        }
        PR1 = T1_WAKE_PERIOD;          //change timer 1 period to blink time
        LED6 = 1;                       //turn LED on
        Sleep();                        //return to sleep
        LED6 = 0;                       //LED off
        PR1 = T1_SLEEP_PERIOD;          //reset Timer 1 to sleep interval
    } while(appData.timer1Flag);        //go back to sleep if woken up by Timer 1

    //Done with sleeping (something other than Timer 1 woke us up)
    __builtin_disi(0x3FFF);                     //disable interrupts    

    //Return sys clock to FRCPLL
    __builtin_write_OSCCONH(0x01);              //Set the NOSC bits to FRCPLL
    __builtin_write_OSCCONL(OSCCONL | 0x01);    //Set the OSWEN bit

    T1CONbits.TON = 0;           //disable Timer 1
#endif //RTCC else Timer 1

    __builtin_disi(0);              //enable interrupts    

    while(OSCCONbits.OSWEN);        //wait for clock switch to complete
    while(OSCCONbits.LOCK == 0);    //wait for PLL lock
    
    RCONbits.SLEEP = 0;         //clear wake from sleep bit

    ADCON1bits.ADON = ADCstate;     //restore peripheral states
    CVRCONbits.CVREN = CVRstate;
    CM2CONbits.CON = CMPstate;
    
    BT_WAKE_HW = 1;             //wake up RN module
    BT_WAKE_SW = 1;
    while(!BT_WS);              //wait for module to wake

    T5CONbits.TON = 1;          //enable timers
    T2CONbits.TON = 1;
}

#endif //USE_SLEEP
