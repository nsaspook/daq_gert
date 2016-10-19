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
 * File:        switches.c
 * Date:        September 17, 2015
 * Compiler:    XC16 v1.23
 *
 *
 */

#include <xc.h>
#include "app.h"
#include "config.h"
#include "timers.h"

extern APP_DATA appData;

//Switches state machine

void Switch_Tasks(void)
{
	//Check if switches have changed and debounce timers are expired
	if (appData.sw1Changed && TimerDone(TMR_SW1_DEBOUNCE)) {
		appData.sw1 = !SWITCH_S1; //latch new switch position
		appData.sw1Changed = false; //clear individual flag
		appData.sendSwitches = true; //set group flag to request TX
	}
	if (appData.sw2Changed && TimerDone(TMR_SW2_DEBOUNCE)) {
		appData.sw2 = !SWITCH_S2;
		appData.sw2Changed = false;
		appData.sendSwitches = true;
	}
	if (appData.sw3Changed && TimerDone(TMR_SW3_DEBOUNCE)) {
		appData.sw3 = !SWITCH_S3;
		appData.sw3Changed = false;
		appData.sendSwitches = true;
	}
	if (appData.sw4Changed && TimerDone(TMR_SW4_DEBOUNCE)) {
		appData.sw4 = !SWITCH_S4;
		appData.sw4Changed = false;
		appData.sendSwitches = true;
	}
}

//Change notification interrupt
//Process and start debounce timers for switch changes
//The switches are well debounced in hardware
//Adding the software debounce limits unneeded switch update messages
//and groups together multiple switch presses that occur within the debounce period

void _ISR_NO_AUTO_PSV _CNInterrupt(void)
{
	IFS1bits.CNIF = 0; //Clear IF

#ifdef USE_SLEEP                //see config.h, Application setting section
	appData.CNint = true;
	SleepTimerReset(); //Reset the inactivity sleep timer
#endif

	//(Re)Start debounce timers for changed switches (pins are active low; vars are active high)
	if (SWITCH_S1 == appData.sw1) {
		StartTimer(TMR_SW1_DEBOUNCE, DEBOUNCE_MS);
		appData.sw1Changed = true;
	} else {
		appData.sw1Changed = false;
	}

	if (SWITCH_S2 == appData.sw2) {
		StartTimer(TMR_SW2_DEBOUNCE, DEBOUNCE_MS);
		appData.sw2Changed = true;
	} else {
		appData.sw2Changed = false;
	}

	if (SWITCH_S3 == appData.sw3) {
		StartTimer(TMR_SW3_DEBOUNCE, DEBOUNCE_MS);
		appData.sw3Changed = true;
	} else {
		appData.sw3Changed = false;
	}

	if (SWITCH_S4 == appData.sw4) {
		StartTimer(TMR_SW4_DEBOUNCE, DEBOUNCE_MS);
		appData.sw4Changed = true;
	} else {
		appData.sw4Changed = false;
	}
}
