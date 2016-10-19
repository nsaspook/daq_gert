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
 * File:        main.c
 * Date:        July 24, 2014
 * Compiler:    XC16 v1.23
 *
 */

#include <xc.h>
#include <stdint.h>
#include "app.h"
#include "config.h"

void initBoard(void);

// Configuration bits
// CONFIG4
#pragma config RTCBAT = ON    // RTC Battery Operation Enable->RTC operation is continued through VBAT
#pragma config IOL1WAY = OFF    // PPS IOLOCK Set Only Once Enable bit->The IOLOCK bit can be set and cleared using the unlock sequence
#pragma config PLLDIV = DIV2    // PLL Input Prescaler Select bits->Oscillator divided by 2 (8 MHz input)
#pragma config DSWDTPS = DSWDTPS1F    // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSSWEN = ON    // DSEN Bit Enable->Deep Sleep is controlled by the register bit DSEN
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = OFF    // Deep Sleep BOR Enable bit->DSBOR Disabled
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer Enable->DSWDT Disabled
#pragma config I2C2SEL = SEC    // Alternate I2C2 Location Select bit->I2C2 is multiplexed to SDA2/RF4 and SCL2/RF5

// CONFIG3
#pragma config WPDIS = WPDIS    // Segment Write Protection Disable->Disabled
#pragma config WPFP = WPFP127    // Write Protection Flash Page Segment Boundary->Page 127 (0x1FC00)
#pragma config WPEND = WPENDMEM    // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory
#pragma config BOREN = OFF    // Brown-out Reset Enable->Brown-out Reset Disable
#pragma config WPCFG = WPCFGDIS    // Write Protect Configuration Page Select->Disabled
#pragma config SOSCSEL = OFF    // SOSC Selection bits->SOSC circuit disabled
#pragma config WDTWIN = PS25_0    // Window Mode Watchdog Timer Window Width Select->Watch Dog Timer Window Width is 25 percent

// CONFIG2
#pragma config POSCMD = NONE    // Primary Oscillator Select->Primary Oscillator Disabled
#pragma config IESO = OFF    // Internal External Switchover->Disabled
#pragma config FNOSC = FRCPLL    // Initial Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config ALTCVREF = CVREF_RB    // External Comparator Reference Location Select bit->CVREF+/CVREF- are mapped to RB0/RB1
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC
#pragma config WDTCMX = WDTCLK    // WDT Clock Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config FCKSM = CSECMD    // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config OSCIOFCN = OFF    // OSCO Pin Configuration->OSCO/CLKO/RC15 functions as port I/O (RC15)
#pragma config ALTADREF = AVREF_RB    // External 12-Bit A/D Reference Location Select bit->AVREF+/AVREF- are mapped to RB0/RB1

// CONFIG1
#pragma config WDTPS = PS1024    // Watchdog Timer Postscaler Select->1:1024
#pragma config LPCFG = ON    // Low power regulator control->Enabled
#pragma config ICS = PGx1    // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC1/PGED1
#pragma config FWPSA = PR128    // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF    // Windowed WDT Disable->Standard Watchdog Timer
#pragma config GWRP = OFF    // General Segment Write Protect->Disabled
#pragma config GCP = OFF    // General Segment Code Protect->Code protection is disabled
#pragma config FWDTEN = WDT_SW    // Watchdog Timer Enable->WDT controlled with the SWDTEN bit
#pragma config JTAGEN = OFF    // JTAG Port Enable->Disabled


//**********************************************************************************************************************
// Main routine - start of executable code

int main(void)
{

	initBoard(); //Initialize the pins and peripherals

	while (1) {
		APP_Tasks();
		Idle(); //Idle until an interrupt is generated
		RCONbits.IDLE = 0;
	}

	//End of while(1) main loop
	return(true);
}

//**********************************************************************************************************************
// Initialize the pins and peripherals

void initBoard(void)
{
	/****************************************************************************
	 * Oscillator Init
	 * Clocking is setup at 32MHz sys clock and to allow USB functionality
	 * Self-tune on SOF is enabled if USB is enabled and connected to host
	 ***************************************************************************/
	// DOZEN disabled; DOZE 1:16; CPDIV 1:1; RCDIV FRC/1; PLLEN disabled; ROI disabled;
	CLKDIV = 0x4000;
	while (OSCCONbits.LOCK == 0); //wait for PLL lock

	// STSRC USB; STEN enabled; STOR disabled; STORPOL Interrupt when STOR is 1; STLOCK disabled; STLPOL Interrupt when STLOCK is 1; STSIDL disabled; TUN Center frequency; 
	OSCTUN = 0x9000;

	//Enable low voltage retention sleep mode
	RCONbits.RETEN = 1;

#ifdef SET_PMD_BITS    //see config.h, Application settings section
	/****************************************************************************
	 * PMD bits - setting a bit disables clocking to that peripheral
	 * (drops operating current by about 2 mA when used in this app)
	 ***************************************************************************/
	PMD1bits.T4MD = 1;

#if !defined (USE_SLEEP) || defined (SLEEP_MODE_RTCC)    //see config.h, Application settings section
	PMD1bits.T1MD = 1;
#endif

	PMD1bits.I2C1MD = 1;
	PMD1bits.U2MD = 1;
	PMD1bits.SPI2MD = 1;
	PMD1bits.SPI1MD = 1;
	PMD3bits.TXMMD = 1;

#if !defined (USE_SLEEP) || !defined (SLEEP_MODE_RTCC)    //see config.h, Application settings section
	PMD3bits.RTCCMD = 1;
#endif

	PMD3bits.PMPMD = 1;
	PMD3bits.CRCMD = 1;
	PMD3bits.DAC1MD = 1;
	PMD3bits.U3MD = 1;
	PMD3bits.I2C2MD = 1;
	PMD2 = 0xFFFF;
	PMD4bits.UPWMMD = 1;
	PMD4bits.U4MD = 1;
	PMD4bits.REFOMD = 1;
	PMD4bits.CTMUMD = 1;
	PMD4bits.HLVDMD = 1;
	PMD5 = 0xFFFF;
	PMD6 = 0xFFFF;
	PMD7 = 0xFFFF;
#endif

	/****************************************************************************
	 * GPIO Init
	 ***************************************************************************/
	ANSB = 0x00;
	ANSD = 0x00;
	ANSE = 0x00;
	ANSF = 0x00;
	ANSG = 0x00;

	CNPU1 = 0;
	CNPU2 = 0;
	CNPU3 = 0;
	CNPU4 = 0;
	CNPU5 = 0;
	CNPU6 = 0;

	CNPD1 = 0;
	CNPD2 = 0;
	CNPD3 = 0;
	CNPD4 = 0;
	CNPD5 = 0;
	CNPD6 = 0;

	CNEN1 = 0x0000;
	CNEN2 = 0x0000;

	ODCB = 0x0000;
	ODCC = 0x0000;
	ODCD = 0x0000;
	ODCE = 0x0000;
	ODCF = 0x0000;
	ODCG = 0x0000;

	//Potentiometer Setup
	POT_TRIS = 1; // Set pin to input
	POT_AN = 1; // Analog mode

	//Input Voltage Sense Setup
	V_SENSE_TRIS = 1; // Set pin to input
	V_SENSE_AN = 1; // Analog mode

	// Set switches as INPUT, Enable CN interrupt
	SWITCH_S1_TRIS = 1;
	SWITCH_S1_CNIE = 1;
	SWITCH_S2_TRIS = 1;
	SWITCH_S2_CNIE = 1;
	SWITCH_S3_TRIS = 1;
	SWITCH_S3_CNIE = 1;
	SWITCH_S4_TRIS = 1;
	SWITCH_S4_CNIE = 1;
	IEC1bits.CNIE = 1;

	// LEDs are outputs and off
	LED1 = 0;
	LED2 = 0;
	LED3 = 0;
	LED4 = 0;
	LED5 = 0;
	LED6 = 0;
	LED7 = 0;
	LED_TRIS1 = 0;
	LED_TRIS2 = 0;
	LED_TRIS3 = 0;
	LED_TRIS4 = 0;
	LED_TRIS5 = 0;
	LED_TRIS6 = 0;
	LED_TRIS7 = 0;

	//RN4020 module - UART1
	BT_WAKE_HW = 1; //Dormant line is set high
	BT_WAKE_HW_TRIS = 0; //Dormant line is output

	BT_WAKE_SW = 0; //keep low until after UART is initialized
	BT_WAKE_SW_TRIS = 0;

	BT_CMD = 0; //Command mode on
	BT_CMD_TRIS = 0;

	BT_WS_TRIS = 1;
	BT_MLDP_EV_TRIS = 1;
	BT_CONNECTED_TRIS = 1;

	U1CTS_TRIS = 1;
	U1RX_TRIS = 1;
	U1RTS_LAT = 0;
	U1RTS_TRIS = 0;
	U1TX_TRIS = 0;

	//5V power supply for 5V Click Boards
	PS_5V_EN = MCP1642B_EN; //Enable or Disable power supply - see config.h
	PS_5V_EN_TRIS = 0; //enable is output
	PS_5V_PG_TRIS = 1; //Power good is input

	//USB (not implemented in this firmware but may be used for power)
	TRISGbits.TRISG2 = 1; //D+
	TRISGbits.TRISG3 = 1; //D-
	TRISFbits.TRISF7 = 1; //VBUS

	//N/Cs set to low outputs
	LATBbits.LATB7 = 0;
	TRISBbits.TRISB7 = 0;
	LATCbits.LATC12 = 0;
	TRISCbits.TRISC12 = 0;
	LATCbits.LATC15 = 0;
	TRISCbits.TRISC15 = 0;
	LATFbits.LATF3 = 0;
	TRISFbits.TRISF3 = 0;

	//Mikrobus header - unused; output low
	//Modify as needed to use Click Boards
	//AN
	LATBbits.LATB3 = 0;
	TRISBbits.TRISB3 = 0;

	//PWM
	LATDbits.LATD3 = 0;
	TRISDbits.TRISD3 = 0;

	//RST
	LATDbits.LATD4 = 0;
	TRISDbits.TRISD4 = 0;

	//INT
	LATDbits.LATD8 = 0;
	TRISDbits.TRISD8 = 0;

	//SDA (I2C)
	LATDbits.LATD9 = 0;
	TRISDbits.TRISD9 = 0;

	//SCL (I2C)
	LATDbits.LATD10 = 0;
	TRISDbits.TRISD10 = 0;

	//RX (UART)
	LATFbits.LATF4 = 0;
	TRISFbits.TRISF4 = 0;

	//TX (UART)
	LATFbits.LATF5 = 0;
	TRISFbits.TRISF5 = 0;

	//SCK (SPI)
	LATGbits.LATG6 = 0;
	TRISGbits.TRISG6 = 0;

	//MISO (SPI)
	LATGbits.LATG7 = 0;
	TRISGbits.TRISG7 = 0;

	//MOSI (SPI)
	LATGbits.LATG8 = 0;
	TRISGbits.TRISG8 = 0;

	//CS (SPI)
	LATGbits.LATG9 = 0;
	TRISGbits.TRISG9 = 0;

	/****************************************************************************
	 * PPS Init - Peripheral Pin Select
	 * Click Boards using PPS-controlled peripherals will require additional
	 * setup here
	 ***************************************************************************/
	__builtin_disi(0x3FFF); //disable interrupts

	//unlock registers
	__builtin_write_OSCCONL(OSCCON & 0xBF);
	// Configure Input Functions (Table 9-1)
	// Assign U1CTS to RP23
	RPINR18bits.U1CTSR = U1CTS_RP_NUM;
	// Assign U1RX to RP24
	RPINR18bits.U1RXR = U1RX_RP_NUM;

	// Configure Output Functions (Table 9-2) 
	// Assign U1TX To Pin RP11
	U1TX_RP_REG = 3;
	// Assign U1RTS To Pin RP12
	U1RTS_RP_REG = 4;
	// RB15->CMP2:C2OUT
	V_SENSE_OUT_RP_REG = 2;
	//lock registers
	__builtin_write_OSCCONL(OSCCON | 0x40);

	__builtin_disi(0); //enable interrupts

	/****************************************************************************
	 * Interrupt Priorities
	 * Interrupt-enabled peripherals being used for Click Boards should be
	 * configured here as well
	 ***************************************************************************/
	//    UERI: U1E - UART1 Error
	//    Priority: 6
	IPC16bits.U1ERIP = 6;

	//    UTXI: U1TX - UART1 Transmitter
	//    Priority: 5
	IPC3bits.U1TXIP = 5;

	//    URXI: U1RX - UART1 Receiver
	//    Priority: 5
	IPC2bits.U1RXIP = 5;

	//USB
	IPC21bits.USB1IP = 4;

	//    TI: T5 - Timer5
	//    Priority: 3
	IPC7bits.T5IP = 3;

	//    TI: T3 - Timer3
	//    Priority: 3
	IPC2bits.T3IP = 3;

	//    TI: T1 - Timer1
	//    Priority: 3
	IPC0bits.T1IP = 3;

	//    RTCI: RTCC - Real-Time Clock and Calendar
	//    Priority: 2
	IPC15bits.RTCIP = 2;

	//    CN: Switches - change notification
	//    Priority: 2
	IPC4bits.CNIP = 2;

	//    ADI: ADC1 - Pipeline A/D Converter 1
	//    Priority: 1
	IPC3bits.AD1IP = 1;
}
