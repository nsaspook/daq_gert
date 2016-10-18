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
 * File:        timers.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * Timer functions
 *
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "timers.h"
#include "app.h"

extern APP_DATA appData;

static volatile uint16_t tickCount[TMR_COUNT] = {0};

//**********************************************************************************************************************
// Initialize the timers
void Timers_Init(void)
{
    //Timer 5 is used for interrupt based software timers counting 1ms intervals to a resolution of 500us
    T5CON = TIMER_OFF;                      //Timer 5 off
    TMR5 = 0;                               //Clear timer 5
    PR5 = TIMER_500US_PERIOD;               //Set the period value for 500us
    T5CON |= TIMER_ON_PRESCALE1;            //using 1:1 prescaler and turn on timer 5
    IFS1bits.T5IF = 0;                      //Clear the interrupt flag
    IEC1bits.T5IE = 1;                      //Enable the timer 5 interrupt
    
#ifdef USE_SLEEP                //see config.h, Application setting section
    //Timer 2/3 is used in 32-bit mode as inactivity timer to trigger sleep mode
    T2CON = TIMER_OFF;                      //Timer 3 off
    TMR3 = 0;                               //Clear timer 3
    TMR2 = 0;                               //Clear timer 2
    T2CONbits.T32 = 1;                      //Enable 32-bit mode
    PR3 = (uint16_t)(SLEEP_TIME >> 16);               //Set the period value - msw
    PR2 = (uint16_t)(SLEEP_TIME | 0x0000FFFF);        //lsw
    T2CON |= TIMER_ON_PRESCALE256;          //using 1:256 prescaler and turn on timer 3
    IFS0bits.T3IF = 0;                      //Clear the interrupt flag
    IEC0bits.T3IE = 1;                      //Enable the timer 3 interrupt
    
#ifndef SLEEP_MODE_RTCC         //we'll be using Timer 1 for periodic wakeup
    T1CON = 0x0000;                         //Timer 1 off
    T1CONbits.T1ECS = 0b10;                 //Clock source LPRC
    T1CONbits.TCS = 1;
    T1CONbits.TCKPS = 0b11;                 //using 1:256 prescaler
    TMR1 = 0;                               //Clear timer 1
    PR1 = T1_SLEEP_PERIOD;                  //Set the period for sleep
    IFS0bits.T1IF = 0;                      //Clear the interrupt flag
    IEC0bits.T1IE = 1;                      //Enable the timer 1 interrupt
#endif //not SLEEP_MODE_RTCC    
#endif //USE_SLEEP
}

//**********************************************************************************************************************
// Start one of the software timers
inline void StartTimer(uint8_t timer, uint16_t count)
{
    tickCount[timer] = count << 1; //Interrupt is every 500us but StartTimer() takes multiple of 1ms so multiply by 2
}

//**********************************************************************************************************************
// Check if one of the software software timers has timed out
inline bool TimerDone(uint8_t timer)
{
    if(tickCount[timer] == 0) {             //Check if counted down to zero
        return true;                        //then return true
    }
    return false;                           //else return false
}

//**********************************************************************************************************************
// Simple delay for n milliseconds (blocking)
void WaitMs(uint16_t numMilliseconds)
{
    StartTimer(TMR_INTERNAL, numMilliseconds);         //Start software timer and wait for it to count down
    while(!TimerDone(TMR_INTERNAL)) {Idle();}          //Enter idle mode to reduce power while waiting
}                                                      //(timer interrupt will wake part from idle)

#ifdef USE_SLEEP                //see config.h, Application setting section
//Reset the inactivity sleep timer
inline void SleepTimerReset(void) {
    TMR3 = 0;                               //Clear timer 3
    TMR2 = 0;                               //Clear timer 2
}
#endif

//**********************************************************************************************************************
// Timer 5 interrupt routine - software timers
void _ISR_NO_AUTO_PSV _T5Interrupt(void)
{
    uint8_t i;

    IFS1bits.T5IF = 0;                      //Clear the interrupt flag
    //Decrement each software timer
    for(i = 0; i < TMR_COUNT; i++) {
        if(tickCount[i] != 0) {tickCount[i]--;}
    }
}

#ifdef USE_SLEEP                //see config.h, Application setting section
//**********************************************************************************************************************
// Timer 3 interrupt routine - inactivity timer
void _ISR_NO_AUTO_PSV _T3Interrupt(void)
{
    IFS0bits.T3IF = 0;                      //Clear the interrupt flag    
    appData.sleepFlag = true;
}
#ifndef SLEEP_MODE_RTCC
//**********************************************************************************************************************
// Timer 1 interrupt routine - periodic wakeup timer
void _ISR_NO_AUTO_PSV _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;                      //Clear the interrupt flag
    appData.timer1Flag = true;
}
#endif //not SLEEP_MODE_RTCC
#endif //USE_SLEEP
