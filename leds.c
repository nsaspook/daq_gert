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
 * File:        leds.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 * add relay outputs
 *
 * LED functions
 *
 */

#include "timers.h"
#include "config.h"
#include "leds.h"
#include "app.h"

extern APP_DATA appData;
static LED_LIGHTSHOW_T lightShow = LED_IDLE;

void LED_Tasks()
{
	switch (lightShow) {
	case LED_IDLE:
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
		LED5 = 1;
		LED6 = 0;
		break;

	case LED_BTLE_ADVERTISING:
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
		LED6 = 0;
		if (TimerDone(TMR_LEDS)) {
			LED5 ^= 1;
			StartTimer(TMR_LEDS, LED_BLINK_MS);
		}
		break;

	case LED_BTLE_PAIRED:
		LED1 = appData.led1; // logic high turns on led
		RELAY1 = !appData.led1; // logic low turns on relay
		LED2 = appData.led2;
		RELAY2 = !appData.led2;
		RELAY2C = appData.led2;
		LED3 = appData.led3;
		RELAY3 = !appData.led3;
		LED4 = appData.led4;
		RELAY4 = !appData.led4;
		LED5 = 1;
		LED6 = 0;
		break;

	case LED_ERROR:
		switch (appData.error_code) {
		case ERROR_INITIALIZATION:
			LED1 = 1;
			LED2 = 0;
			LED3 = 0;
			LED4 = 0;
			LED5 = 1;
			LED6 = 1;
			break;
		case ERROR_RN_FW:
			LED1 = 1;
			LED2 = 1;
			LED3 = 0;
			LED4 = 0;
			LED5 = 1;
			LED6 = 1;
			break;
		default:
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			LED4 = 1;
			LED5 = 1;
			LED6 = 1;
			break;
		}
		break;

	case LED_SLEEP:
		LED1 = 0;
		LED2 = 0;
		LED3 = 0;
		LED4 = 0;
		LED5 = 0;
		LED6 = 1;
		break;

	default:
		break;
	}
}

inline void LED_SET_LightShow(LED_LIGHTSHOW_T setting)
{
	lightShow = setting;
}

//Update LEDs with status from LED update message

void GetNewLEDs(void)
{
	appData.led1 = appData.receive_packet[9] == '1' ? 1 : 0;
	appData.led2 = appData.receive_packet[11] == '1' ? 1 : 0;
	appData.led3 = appData.receive_packet[13] == '1' ? 1 : 0;
	appData.led4 = appData.receive_packet[15] == '1' ? 1 : 0;
}
