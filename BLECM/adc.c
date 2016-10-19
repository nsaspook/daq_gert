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
 * File:        adc.c
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 *
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "adc.h"
#include "app.h"
#include "config.h"
#include "timers.h"

extern APP_DATA appData;

/******************************************************************************
 * Function:        void ADC_Init()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine initializes the ADC 
 *                 
 * Note:
 *
 *****************************************************************************/

void ADC_Init()
{
	ADCON1bits.ADON = 0; // Turn off ADC so we can write registers
	ADCON1bits.ADSIDL = 0; // Halts in CPU Idle
	ADCON1bits.ADSLP = 0; // Ignores triggers and clocks when CPU is in Sleep
	ADCON1bits.FORM = 0b0100; // Integer; 0000 dddd dddd dddd
	ADCON1bits.PUMPEN = 0; //Charge pump disabled
	ADCON1bits.ADCAL = 0; // No operation for internal analog calibration bit
	ADCON1bits.PWRLVL = 0; // Low power mode

	ADCON2bits.PVCFG = 0b00; //AVdd ADREF+ bits
	ADCON2bits.NVCFG0 = 0; // AVss Converter voltage reference configuration
	ADCON2bits.BUFORG = 1; //Result buffer is 32 deep FIFO; stored in sequential order
	ADCON2bits.REFPUMP = 0; //Reference charge pump disabled

	// BUFSTBY Normal; BUFSIDL disabled; BUFEN disabled; BUFOE disabled; BUFREF 1.2 V; BUFSLP disabled; 
	BUFCON1 = 0x0000;

	ADCON3bits.ADRC = 0; // Conversion clock derived from system clock
	ADCON3bits.SLEN3 = 0; // Sampling for this list is disabled
	ADCON3bits.SLEN2 = 0;
	ADCON3bits.SLEN1 = 0;
	ADCON3bits.SLEN0 = 0;
	ADCON3bits.ADCS = 0x04; // Tad = 8*Tsrc (2 MHz w/ 16 MHz clock)

	//Set sample list
	// CTMEN disabled; ASEN enabled; SAMC 0.5*Tad; MULCHEN One at a time; CM Matching is disabled; WM All conversion results saved; SLINT No interrupt; 
	ADL0CONHbits.ASEN = 1; // Auto-Scan
	ADL0CONHbits.SAMC = 0b00000; //250 ns @ 2 MHz
	ADL0CONHbits.SLINT = 0b01;
	// SLEN enabled; THSRC Buffer register; SLSIZE 1; SLTSRC Manual; SLENCLR disabled; SAMP enabled;     
	ADL0CONLbits.SAMP = 1;
	ADL0CONLbits.SLTSRC = 0b00000;
	ADL0CONLbits.SLSIZE = 0; //1 channels in list

	//Set table registers
	// ADCH potentiometer; DIFF disabled; UCTMU disabled; 
	ADTBL0bits.ADCH = POT_AN_CHAN;

	// Set table pointer registers
	ADL0PTR = 0;
	ADL1PTR = 0;
	ADL2PTR = 0;
	ADL3PTR = 0;

	// COUNT 0; TBLSEL ADTBL0; 
	ACCONLbits.COUNT = 0; //256x oversampling
	ACCONLbits.TBLSEL = 0;
	// Enable accumulator and accumulator interrupt 
	ACCONHbits.ACIE = 1;
	ACCONHbits.ACEN = 1;
	ACRESL = 0; // Set initial value for the accumulator.
	ACRESH = 0;


	/* ADC1 - Pipeline A/D Converter 1 */
	__builtin_disi(0x3FFF); //disable interrupts
	//Enable ADC
	ADCON1bits.ADON = 1;

	//Poll the ADREADY bit
	while (!ADSTATHbits.ADREADY);

	// Start calibration   
	ADCON1bits.ADCAL = 1;

	//Poll the ADREADY bit
	while (!ADSTATHbits.ADREADY);
	appData.ADCcalFlag = true;

	//Enable sample list
	ADL0CONLbits.SLEN = 1; // Enable Sample list 0

	// clear the interrupt flags
	IFS0bits.AD1IF = 0;
	ADSTATLbits.ACCIF = 0;
	// enable the interrupt
	IEC0bits.AD1IE = 1;

	__builtin_disi(0x0); //enable interrupts

	//Start initial reading
	ADL0CONLbits.SAMP = 1; //enable sampling for first reading
	ADL0CONLbits.SAMP = 0; //close sample switch
}

//State machine for restarting ADC and taking new readings from pot
//Returns true when module is on, calibrated, and started sampling; false otherwise

bool ADC_Tasks(void)
{
	if (!ADCON1bits.ADON) { //Module is off
		ADCON1bits.ADON = 1; //enable module
		appData.ADCcalFlag = false; //not calibrated yet
		return false;
	}
	if (ADSTATHbits.ADREADY && !appData.ADCcalFlag) { //wait for ready bit
		ADCON1bits.ADCAL = 1; //Start calibration
		appData.ADCcalFlag = true;
		return false;
	}
	if (ADSTATHbits.ADREADY && appData.ADCcalFlag) { //wait for ready bit after calibration
		appData.ADCinUse = true; //set ADC in use app flag
		ADL0CONLbits.SAMP = 1; //enable sampling
		ADL0CONLbits.SAMP = 0; //close sample switch
		IEC0bits.AD1IE = 1; //Enable ADC interrupts
		return true; //return true because we're now sampling
	}
	return false;
}

//Process the accumulator value once it is ready
//And update stored potentiometer values

void ADC_ProcAccum(void)
{
	static uint32_t potSum;
	static uint8_t potCount;
	uint32_t msw;

	msw = ACRESH; //get high word
	msw <<= 16;
	potSum += msw | ACRESL; //store full accumulated value
	potCount++;
	//Average values
	if (potCount == ADC_NUM_AVGS) {
		appData.potValueOld = appData.potValue; //Save previous value
		appData.potValue = (potSum / potCount) >> 10; //Toss lower 10 bits, leaving 10 bits (0 - 1023)
		potSum = 0; //Restart averaging
		potCount = 0;
	}

	ACRESL = 0; // Set initial value for the accumulator
	ACRESH = 0;
	ACCONLbits.COUNT = 0; // Reset counter
	ACCONHbits.ACEN = 1; // Reenable accumulator
}

//ADC ISR

void _ISR_NO_AUTO_PSV _ISR _ADC1Interrupt(void)
{
	IFS0bits.AD1IF = 0;
	//Accumulation complete
	if (ADSTATLbits.ACCIF) {
		ADCON1bits.ADON = 0; //Disable ADC module to lower power consumption
		appData.accumReady = true; //Set accumulator ready app flag
		ADSTATLbits.ACCIF = 0; // Reset accumulator interrupt flag
		ADSTATLbits.SL0IF = 0; // Reset sample list interrupt flag
		IEC0bits.AD1IE = 0; //Disable ADC interrupts
	}		//Conversion complete but accumulation incomplete - trigger another sample for oversampling
	else if (ADSTATLbits.SL0IF) {
		ADSTATLbits.SL0IF = 0;
		ADL0CONLbits.SAMP = 1; //enable sampling
		ADL0CONLbits.SAMP = 0; //close sample switch
	}
}
