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
 * File:        config.h
 * Date:        January 20, 2015
 * Compiler:    XC16 v1.23
 *
 * General definitions for the project
 * 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <xc.h>

#define APP_VERSION_STR "1.5"       //This firmware version
/* for relay click board */

/*******************************************************************************
 * Application settings - these will change application behavior
 ******************************************************************************/

//Selectively set PMD bits if this is defined
//Disables clocking to peripherals that are not used in this demo code
//Reduces operating current by about 2 mA
//Risk is trying to add code that uses a disabled peripheral and
//forgetting that it is disabled by PMD
//The bits are selectively set in main.c --> initBoard()
//#define SET_PMD_BITS

//Enables sleep mode based on an inactivity timer.
//Switch presses and potentiometer changes will reset inactivity timer window.
//D6 will go solid on when device first goes to sleep and then will
//pulse periodically during sleep.
//Any switch press will wake device from sleep.
//Reduces power consumption but might not be wanted, depending on use case
//Sleep mode will be enabled when this is defined
//Inactivity time-out is set below in Application timers section
//#define USE_SLEEP

//Determines mode of sleep operation, to illustrate different possible methods.
//When defined, sleep mode will clock switch to the LPRC for the system clock
//and will use the RTCC (clocked by LPRC) for periodic wake-up to flash the status LED.
//If not defined, sleep mode will use Timer 1 (clocked by LPRC) for periodic wakeup
//to flash the status LED. Each mode has its strengths.
//If timing accuracy is required using RTCC mode, the RTCC should be clocked
//from the external crystal on SOSC instead.
//Setting has no effect if USE_SLEEP is not defined.
//#define SLEEP_MODE_RTCC

//A change in potentiometer reading greater than this will reset the sleep inactivity timer
//A threshold greater than 1 prevents potentiometer noise / ADC drift from keeping board awake
//Has no effect if sleep is disabled
#define POT_KEEP_AWAKE_DELTA    5               //ADC count delta after oversampling, averaging, and 10-bit conversion

//Enable / Disable the MCP1642B 5V boost power supply for 5V power pin on MikroBUS header
//Set to 0 to disable; 1 to enable (Enable this for 5V Click Boards)

//*!*!*!*!*!*!*!*!*!*!*!*!*   NOTE   *!*!*!*!*!*!*!*!*!*!*!*!*
//If using a 5V Click Board with analog output, also set S7 on the PCB to the "5V"
//setting to enable the analog voltage divider which will scale the output to 0 - 3.3V
//#define MCP1642B_EN    0
#define MCP1642B_EN    1

// UART baud rate - RN module defaults to 115200
#define BRG_115200 (FCY/(4*115200) - 1)        //BRG value for 115,200 baud with BRGH = 1 (with rounding)

//Comparator voltage reference CVR setting
#define CVR_BITS            18                 //Determines base voltage threshold for low battery indication

//Number of samples to average for potentiometer ADC reading, in addition to hardware oversampling
#define ADC_NUM_AVGS        5

//If defined, the RN4020's firmware version will be checked as part of initialization.
//If the version is not equal to the version specified below, the board will
//error out with the initialization error code display and not operate
#define VERIFY_RN_FW_VER
#define RN_FW_VER_MAJOR     1           //Require 1.23.5
#define RN_FW_VER_MINOR     23          //These values can be changed as needed
#define RN_FW_VER_PATCH     5

#define RN_FW_VER_MAJOR133     1 
#define RN_FW_VER_MINOR133     33 
#define RN_FW_VER_PATCH133     4

//Application timers
#define SLEEP_TIME          TIMER_5MIN_PERIOD_PS256     //inactivity timer for sleep - applies only when USE_SLEEP is defined
#define DEBOUNCE_MS         75          //debounce time for switches 1 - 4
#define ADC_REFRESH_MS      50          //delay between ADC reads
#define POT_TX_MS           500         //delay between transmitting new pot values
#define LED_BLINK_MS        500         //LED blink rate for advertise mode
#define BT_TX_MS            150         //minimum time between consecutive BTLE message transmissions
#define BAT_CHK_DELAY_MS    30000       //delay between input voltage checks
#define BAT_CHK_WAIT_MS     10          //CVref & CMP stabilization time
//Periods for timer 1 sleep mode (for periodic sleep wakeup); 31KHz LPRC; 1:256 prescale
#define T1_WAKE_PERIOD      2       //2 ~= 16 ms
#define T1_SLEEP_PERIOD     605     //605 ~= 5 seconds

//Buffer sizes
#define SIZE_RxBuffer   256               //UART RX software buffer size in bytes
#define SIZE_TxBuffer   256               //UART TX software buffer size in bytes

#define BT_RX_PKT_SZ    100               //Max receive packet length
#define BT_TX_PKT_SZ    100               //Max transmit packet length

//BTLE services
#define PRIVATE_SERVICE "28238791ec55413086e0002cd96aec9d"
#define PRIVATE_CHAR_SWITCHES "8f7087bdfdf34b87b10fabbf636b1cd5"
#define PRIVATE_CHAR_POTENTIOMETER "362232e5c5a94af6b30ce208f1a9ae3e"
#define PRIVATE_CHAR_LEDS "cd8306093afa4a9da58b8224cd2ded70"
#define PRIVATE_CHAR_RELAYS "cd83060a3afa4a9da58b8224cd2ded70"

//attribute for ISRs that do not alter PSV registers
#define _ISR_NO_AUTO_PSV __attribute__((interrupt,no_auto_psv))

/*******************************************************************************
 * End application configuration settings
 * Hardware settings below
 ******************************************************************************/

// Clock frequency
#define FCY (16000000)                              //8MHz FRC x 4 PLL = 32 MHz (2 clocks per instruction)

//MCP1642 5V boost for Mikrobus
#define PS_5V_EN_TRIS      TRISDbits.TRISD6        //Enable pin
#define PS_5V_EN           LATDbits.LATD6
#define PS_5V_PG_TRIS      TRISDbits.TRISD7        //Power good signal
#define PS_5V_PG           PORTDbits.RD7

//RN4020 BTLE
#define BT_WAKE_HW      LATEbits.LATE1                       //Hardware wake from dormant state; BT_WAKE_HW
#define BT_WAKE_HW_TRIS TRISEbits.TRISE1

#define BT_WAKE_SW      LATEbits.LATE7                       //Deep sleep wake; BT_WAKE_SW
#define BT_WAKE_SW_TRIS TRISEbits.TRISE7

#define BT_CMD      LATEbits.LATE6                 //Place RN4020 module in command mode, low for MLDP mode
#define BT_CMD_TRIS TRISEbits.TRISE6

#define BT_CONNECTED        PORTEbits.RE4                     //RN4020 module is connected to central device
#define BT_CONNECTED_TRIS   TRISEbits.TRISE4

#define BT_WS       PORTEbits.RE2                         //RN4020 module is awake and active
#define BT_WS_TRIS  TRISEbits.TRISE2

#define BT_MLDP_EV      PORTEbits.RE3                         //RN4020 module in MLDP mode has a pending event
#define BT_MLDP_EV_TRIS TRISEbits.TRISE3

//UART
#define U1CTS_TRIS      TRISDbits.TRISD2
#define U1CTS_RP_NUM    23

#define U1RTS_TRIS      TRISDbits.TRISD11
#define U1RTS_LAT       LATDbits.LATD11
#define U1RTS_RP_REG    RPOR6bits.RP12R

#define U1RX_TRIS   TRISDbits.TRISD1  //BT_RX
#define U1RX_PORT   PORTDbits.RD1
#define U1RX_RP_NUM 24

#define U1TX_TRIS   TRISDbits.TRISD0  //BT_TX
#define U1TX_LAT    LATDbits.LATD0
#define U1TX_RP_REG RPOR5bits.RP11R

#define UART_RX_IF      IFS0bits.U1RXIF
#define UART_TX_IF      IFS0bits.U1TXIF
#define UART_ER_IF      IFS4bits.U1ERIF

#define UART_TX_IE      IEC0bits.U1TXIE
#define UART_RX_IE      IEC0bits.U1RXIE
#define UART_ER_IE      IEC4bits.U1ERIE
#define UART_TX_EN      U1STAbits.UTXEN
#define UART_EMPTY      U1STAbits.TRMT
#define UART_FULL       U1STAbits.UTXBF

#define UART_TX_BUF     U1TXREG
#define UART_RX_BUF     U1RXREG

// Switch inputs
#define SWITCH_S1 PORTBbits.RB4                         //Switches are active low
#define SWITCH_S2 PORTBbits.RB14
#define SWITCH_S3 PORTEbits.RE0
#define SWITCH_S4 PORTDbits.RD5

#define SWITCH_S1_TRIS  TRISBbits.TRISB4
#define SWITCH_S2_TRIS  TRISBbits.TRISB14
#define SWITCH_S3_TRIS  TRISEbits.TRISE0
#define SWITCH_S4_TRIS  TRISDbits.TRISD5

#define SWITCH_S1_CNIE  CNEN1bits.CN6IE
#define SWITCH_S2_CNIE  CNEN3bits.CN32IE
#define SWITCH_S3_CNIE  CNEN4bits.CN58IE
#define SWITCH_S4_CNIE  CNEN1bits.CN14IE

// RELAY outputs
#define RELAY1	LATDbits.LATD3
#define RELAY2	LATDbits.LATD9
#define RELAY2C	LATGbits.LATG9
#define RELAY3	LATDbits.LATD10 // output 0 (low) turns on relay
#define RELAY4	LATDbits.LATD4
//#define RELAY_LOW_LOGIC


// LED outputs
#define LED1 LATBbits.LATB13
#define LED2 LATBbits.LATB12
#define LED3 LATFbits.LATF0
#define LED4 LATFbits.LATF1
#define LED5 LATBbits.LATB6
#define LED6 LATEbits.LATE5
#define LED7 LATBbits.LATB15

#define LED_TRIS1 TRISBbits.TRISB13
#define LED_TRIS2 TRISBbits.TRISB12
#define LED_TRIS3 TRISFbits.TRISF0
#define LED_TRIS4 TRISFbits.TRISF1
#define LED_TRIS5 TRISBbits.TRISB6
#define LED_TRIS6 TRISEbits.TRISE5
#define LED_TRIS7 TRISBbits.TRISB15

//Potentiometer
#define POT_TRIS        TRISBbits.TRISB5
#define POT_AN          ANSBbits.ANSB5
#define POT_AN_CHAN     5

//Input voltage sense
#define V_SENSE_TRIS        TRISBbits.TRISB2
#define V_SENSE_AN          ANSBbits.ANSB2
#define V_SENSE_CMP_CHAN    0b00                //C2INB select
#define V_SENSE_OUT_RP_REG  RPOR14bits.RP29R    //for C2OUT

//Timer initialization
#define TIMER_OFF 0
#define TIMER_ON_PRESCALE1      0x8000
#define TIMER_ON_PRESCALE8      0x8010
#define TIMER_ON_PRESCALE64     0x8020
#define TIMER_ON_PRESCALE256    0x8030

//Timer periods
//32-bit mode with 1:256 postscale below
#define TIMER_5MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 300 - 1))
#define TIMER_1MIN_PERIOD_PS256 ((uint32_t)((FCY / 256) * 60 - 1))
#define TIMER_10S_PERIOD_PS256  ((uint32_t)((FCY / 256) * 10 - 1))
#define TIMER_1S_PERIOD_PS256   ((uint16_t)(FCY / 256 - 1))

//16-bit mode with 1:1 postscale below
#define TIMER_1MS_PERIOD        ((uint16_t)(FCY / 1000 - 1))
#define TIMER_100US_PERIOD      ((uint16_t)(FCY / 10000 - 1))
#define TIMER_500US_PERIOD      ((uint16_t)(FCY / 2000 - 1))

#endif //CONFIG_H
