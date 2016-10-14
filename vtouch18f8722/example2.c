
// PIC18F8722 Configuration Bit Settings

#include <p18f8722.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer (WDT enabled)
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = ON   // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)


/*
 Viision terminal code */
/*
 * This program converts the rs-232 output from a ELO 1939L E224864 CarrollTouch controller type LCD monitor 
 * to a format that can be used with the Varian Viision 80 Implanter with ADYIN CRT monitor
 * The Carroll touchscreen will be  programmed to the correct touch response and configuration at program start
 *
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * Fred Brooks, Microchip Inc , Oct 2016
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * ET-BASE PIC8722 board and  device
 *
 * usart1	connected to implant host computer at 9600
 * usart2       connected to touchscreen  serial post at 9600
 *
 *
 * V1.00	First release
 * V1.01	pre compute X/Y scale values
 * V1.02	Clean up port setup code and WDT (remove clrwdt from isr)
 * V1.03        New touchscreen support.
 * V1.04	xmit rs232 uses int mode in ISR
 * V1.05	Configuration for touch code repeats sending to host
 * V1.06	Set touch repeats to zero
 * V1.07        Set Z touch value to max 0x0F
 * V1.08	Set Z to 1 and remove multi-touch testing
 * V2.00        Code rewrite
 * V3.00	add code for smartset again
 * V3.01	mode checks and lamps for proper smartset to emulation operation
 * V3.10	working Intellitouch version
 * V3.20	code cleanup
 * V3.30	unified Viision/E220/E500 driver
 *
 *
 *
 * HOST RS-232  5-1     uC port1
 * Female       2-2-tx
 *              3-3-rx
 * LCD  RS-232  5-1     uC port2
 * Male         2-3-rx
 *              3-2-tx
 */

/* E220/E500 terminal code
 /*
 * This program converts the rs-232 output from a ELO Carroll-Touch& accutouch SmartSet touch-screen controller
 * to a format that can be used with the Varian E220/E500 Implanter
 * The touch controller must be first programmed
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * PRORTA, PORTE Camera, aux switching with touchs in target box
 * PORTJ		LED bar display
 * PORTH0		run flasher led onboard.
 * 8 led status lights.
 *
 * Fred Brooks, Microchip Inc , Aug 2009,2016
 * Gresham, Oregon
 *
 *
 * This application is designed for use with the
 * ET-BASE PIC8722 board and  device.
 * HOST RS-232  5-1     uC port1
 * Female       2-2-tx
 *              3-3-rx
 * LCD  RS-232  5-1     uC port2
 * Male         2-3-rx
 *              3-2-tx
 * 
 * VGA converter box relay
 * Omron 
 * G6k-2P bottom view
 * Pin		8 - gnd, wire tag 0, to RELAY output	pin 2 on connector for RA1, RE1 PORT OUTPUT
 * Pin		1 + 5vdc,		Power PIN	pin 9 connector for RA or RE PORT VCC		 
 */
//				***
//				E0.94		Clean up the code and comments
//				E0.95		Add startup delay for remote service terminals
//				E0.96		status reporting and monitor testing
//				E0.97		port bit testing
//				E0.98		fix cylon led roll.
//				E0.99		debug 1,2 Screen size code results changed with portD bit 0,1
//				E1.00
//				E1.01		debug 8 on single/tracking touch modes portD bit 7
//				E1.02		debug 7 on flash LCD while processing. portD bit 6
//				E1.03		code fixes/updates
//				E1.04		add delay in status/touched host send routines
//				E1.05		add interlocks for touch input from screen
//				E1.06		add WDT counter test switch input and checks for valid ts inputs.
//				E1.07		screen connect restart code via WDT timeout.
//				E1.08		Learn touchs and set with special touch sequence.
//				E1.09		Code for 2 special touchs and remove the delay switch. bit3 learn1, bit2 learn2
//				E1.10		External output to led/relay on PORTE, RE0,RE7 mirrors HA0 led
//				E1.11		add JB define for switch board missing.
//				E1.12-13	fix LCD display
//				E1.14		fix rs-232 flags
//				E1.15		Small coding  cleanups
//				E1.16		remove WDT calls in ISR, check for proper comms with the touch screen and controller.
//				E1.17		auto init touchscreen code.
//				E1.18		Code for new LCD screens and debug capture.
//				E1.19		recode ISR to remove library define functions
//				E1.20		VGA/CAM switcher code.
//				E1.21		Timed camera for left press, software smells
//				E1.22		Support for SmartSet commands on newer touch panels
//				E1.23		refactor
//				E1.24		adjust newer screen size for better touch fit
//				V3.30		converted to unified driver
//				V3.31		bug fixes, cleanup
//				***

#include <usart.h>
#include <delays.h>
#include <timers.h>
#include <stdlib.h>
#include <EEP.h>
#include <GenericTypeDefs.h>

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
/*unsigned types*/
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
/*signed types*/
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;
#endif

void rxtx_handler(void);

#define	DO_CAP	FALSE			// E220 save data for usarts 1&2, save to eeprom
#define	TS_TYPE	0			// E220 0 for old CRT type screens, 1 for newer LCD screens with Carroll-Touch
#define SET_EMU	TRUE			// E220 emulate old CRT with SmartSet LCD touch
#define BUF_SIZE 64
#define	CAP_SIZE 256

#define	CMD_SIZE 2
#define	CMD_OVERFLOW	CMD_SIZE*12
#define ELO_SIZE 12
#define ELO_SEQ 10
#define ELO_REV_H	4096
#define ELO_SS_H_SCALE	0.483
#define ELO_SS_V_SCALE	0.370
#define	BLINK_RATE_E220	20000
#define AUTO_RESTART	FALSE
#define SINGLE_TOUCH	FALSE
#define GOOD_MAX	128		// max number of chars from TS without expected frames seen
#define MAX_CAM_TIME	5
#define MAX_CAM_TIMEOUT	30
#define MAX_CAM_TOUCH	5
#define CAM_RELAY	LATAbits.LATA1
#define CAM_RELAY_AUX	LATEbits.LATE1
#define CAM_RELAY_TIME	LATEbits.LATE2
#define	TIMERPACKET	41000

#define BUF_SIZE_V80 16
#define	CMD_SIZE_V80 4				// CT size of command in bytes from touch screen
#define	CMD_SIZE_SS_V80 6			// E281A-4002 software emulation Binary size of command
#define	HOST_CMD_SIZE_V80	6		// tool command size
#define	CMD_OVERFLOW_V80 HOST_CMD_SIZE_V80*2
#define ELO_SEQ_V80 10				// max smartset sequences
#define ELO_SIZE_V80 14				// number of bytes to send from elocodes_s configuration string
#define ELO_SIZE_I_V80 10			// max smartset sequence size
#define FALSE	0
#define TRUE	1
#define	BLINK_RATE_V80	25000			// BSG timing
#define	X_SCALE	1.90				// scaling factor to host screen X logical coords
#define	Y_SCALE 1.75				// scaling factor to host screen Y logical coords
#define	X_SCALE_SS	0.905			// scaling factor to host screen X logical coords
#define	Y_SCALE_SS	0.650			// scaling factor to host screen Y logical coords
#define	X_LOGICAL	119			// LCD touchscreen logical X frame coords
#define	Y_LOGICAL	94			// LCD touchscreen logical Y frame coords
#define	X_TOOL		202
#define	Y_TOOL		164

#define	BLINK_RATE_OTHER	15000		// BSG timing

#define	TIMEROFFSET	26474			// timer0 16bit counter value for 1 second to overflow
#define	TIMERFAST	58974			// fast flash or testing
#define	COMM_CHK_TIME	30			// LCD comm heartbeat
#define	LCD_CHK_TIME	36			// LCD heartbeat timeout

const rom int8_t *build_date = __DATE__, *build_time = __TIME__,
	build_version[] = " V3.31 8722 Varian VE touch-screen converter. Fred Brooks, Microchip Inc.";

typedef uint8_t packettype[8];

typedef struct reporttype {
	uint8_t headder, status;
	uint16_t x_cord, y_cord, z_cord;
	uint8_t checksum;
	uint8_t tohost;
} volatile reporttype;

typedef struct statustype {
	int32_t alive_led, touch_count, resync_count, rawint_count, status_count;
} volatile statustype;

volatile uint16_t c_idx = 0, speedup = 0;

volatile uint8_t CATCH = FALSE, TOUCH = FALSE, UNTOUCH = FALSE, LCD_OK = FALSE, comm_check = 0, init_check = 0,
	SCREEN_INIT = FALSE,
	CATCH46 = FALSE, CATCH37 = FALSE, TSTATUS = FALSE, NEEDSETUP = FALSE,
	DATA1 = FALSE, DATA2 = FALSE, LEARN1 = FALSE,
	LEARN2 = FALSE, CORNER1 = FALSE, CORNER2 = FALSE, CAM = FALSE, do_emu_ss = SET_EMU, ACK = FALSE, INPACKET = FALSE;

enum screen_type_t {
	DELL_E224864, DELL_E215546, OTHER
} screen_type;

enum emulat_type_t {
	VIISION, E220, OTHER
} emulat_type;

volatile uint8_t touch_good = 0, cam_time = 0, do_cap = DO_CAP;
volatile int32_t touch_count = 0, resync_count = 0, rawint_count = 0, status_count = 0;
volatile int32_t j = 0;
volatile float xs = X_SCALE, ys = Y_SCALE, xs_ss = X_SCALE_SS, ys_ss = Y_SCALE_SS; // defaults
volatile uint16_t timer0_off = TIMEROFFSET;

#pragma idata bigdata
volatile uint8_t elobuf[BUF_SIZE], elobuf_out[BUF_SIZE_V80], elobuf_in[BUF_SIZE_V80], testing_data, xl = X_LOGICAL, yl = Y_LOGICAL;
volatile uint8_t ssbuf[BUF_SIZE];

volatile struct reporttype ssreport;
volatile struct statustype status;

const rom uint8_t elocodes_s_v[] = {
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x25, 0x29, 0x44, 0x3d, 0x2a, 0x37
}; // initial carrol-touch config codes, tracking, add end point modifier, get frame size report

const rom uint8_t elocodes[ELO_SEQ_V80][ELO_SIZE_I_V80] = {// elo 2210/2216 program codes
	'U', 'M', 0x00, 0x87, 0x40, '0', '0', '0', '0', '0', // initial touch,stream Point,untouch,Z-axis,no scaling, tracking
	'U', 'S', 'X', 0x00, 0x0ff, 0x00, 0x01, '0', '0', '0', // scale x: X,Y,Z scaling Not Used
	'U', 'S', 'Y', 0x00, 0x0ff, 0x00, 0x01, '0', '0', '0', // scale y
	'U', 'S', 'Z', 0x00, 0x01, 0x00, 0x0f, '0', '0', '0', // scale z
	'U', 'B', 5, 20, 0x00, 0x00, 0x0f, '0', '0', '0', // packet delays to match old terminal
	'U', 'E', '1', '6', '0', '0', '0', '0', '0', '0', // emulation E281A-4002 Binary (Z=1-255 on touch, Z=0 on untouch)
	'U', 'N', '1', '7', '0', '0', '0', '0', '0', '0', // nvram save
	'U', 'R', '2', '0', '0', '0', '0', '0', '0', '0', // nvram reset
}; // initial intelli-touch codes

uint8_t elocodes_m_e[] = {// 5 char, soft-reset,touch scanning off, report transfer on, (0x26) tracking mode, report transfer on, clear touch buffer, touch scanning on
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x26, 0x44, 0x3d, 0x2a
}; // initial touch config codes, tracking
uint8_t elocodes_s_e[] = {// same as above ex (0x25) enter point mode
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x25, 0x44, 0x3d, 0x2a
}; // initial touch config codes, single


// SmartSet codes 0 command, 1 status, 2 low byte, 3 high byte, etc ...
uint8_t elocodes_e0[] = {
	'U', 'B', 0x01, 0x4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // smartset timing and spacing setup
};
uint8_t elocodes_e1[] = {
	'U', 'Q', 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e2[] = {
	'U', 'M', 0x00, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e3[] = {
	'U', 'S', 'X', 0x01, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e4[] = {
	'U', 'S', 'Y', 0x01, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e5[] = {
	'U', 'i', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e6[] = {
	'U', 'g', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
uint8_t elocodes_e7[] = {// dummy packet
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

volatile uint16_t tchar, uchar, debug_port = 0, restart_delay = 0, touch_saved = 0, touch_sent = 0, touch_corner1 = 0,
	touch_corner_timed = 0, corner_skip = 0;

#pragma idata

#pragma idata sddata
volatile uint8_t host_rec[CAP_SIZE] = "H";
volatile uint8_t scrn_rec[CAP_SIZE] = "S";
#pragma idata 

volatile uint8_t host_write = FALSE, scrn_write = FALSE;


#pragma idata

#pragma code touch_int = 0x8

void touch_int(void)
{
	_asm goto rxtx_handler _endasm
}
#pragma code

#pragma interrupt rxtx_handler

void rxtx_handler(void) // all timer & serial data transform functions are handled here
{
	static union Timers timer0;
	static uint8_t junk = 0, c = 0, *data_ptr,
		i = 0, data_pos, data_len, tchar, uchar;
	uint16_t x_tmp, y_tmp, uvalx, lvalx, uvaly, lvaly;
	static uint16_t scrn_ptr = 0, host_ptr = 0;
	static uint8_t sum = 0xAA + 'U', idx = 0;

	if (INTCONbits.RBIF) {
		junk = PORTB;
		INTCONbits.RBIF = 0;
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for host link */
	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		if (data_pos >= data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = 0; // stop data xmit
			}
		} else {
			LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led
			TXREG1 = *data_ptr; // send data and clear PIR1bits.TX1IF
			data_pos++; // move the data pointer
			data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR1bits.RCIF) { // is data from host COMM1
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = 0; //	clear overrun
			RCSTA1bits.CREN = 1; // re-enable
		}
		if (do_cap) {
			if (host_ptr < CAP_SIZE) {
				host_rec[host_ptr] = RCREG1; // read data from touchscreen
				while (!TXSTA2bits.TRMT) {
				}; // wait until the usart is clear
				TXREG2 = host_rec[host_ptr];
				host_ptr++;
				LATJbits.LATJ1 = !LATJbits.LATJ1; // flash  led
			} else {
				tchar = RCREG1; // read from host
				host_write = TRUE;
				LATJbits.LATJ3 = !LATJbits.LATJ3; // flash  led
			}
		} else {
			tchar = RCREG1; // read from host
			DATA1 = TRUE; // usart is connected to data
			if ((tchar == (uint8_t) 0x46)) { // send one report to host
				CATCH46 = TRUE;
				touch_good = 0;
			}
			if ((tchar == (uint8_t) 0x37)) { // start of touch scan read
				CATCH37 = TRUE;
				touch_good = 0;
			}
			if ((tchar == (uint8_t) 0x3C)) { // touch reset from host
				NEEDSETUP = FALSE;
			}
		};
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer
		//check for TMR0 overflow
		idx = 0; // reset packet char index counter
		ssreport.tohost = FALSE; // when packets stop allow for next updates
		INTCONbits.TMR0IF = 0; //clear interrupt flag
		WriteTimer0(timer0_off);

		if (LCD_OK)
			LATF = 0xff;

		if (!LCD_OK && (init_check++ >LCD_CHK_TIME)) {
			init_check = 0; // reset screen init code counter
			SCREEN_INIT = TRUE; // set init code flag so it can be sent in main loop
			LATEbits.LATE3 = 0; // init  led ON
		}

		if ((comm_check++ >COMM_CHK_TIME) && !CATCH) { // check for LCD screen connection
			comm_check = 0; // reset connect heartbeat counter
			LCD_OK = FALSE; // reset the connect flag while waiting for response from controller.
			while (!TXSTA2bits.TRMT) {
			}; // wait until the usart is clear
			if (screen_type == DELL_E224864) {
				TXREG2 = 0x37; // send frame size request to LCD touch
			}
			if (screen_type == DELL_E215546) {
				LATF = 0xff;
			}
			LATEbits.LATE1 = !LATEbits.LATE1; // flash  led
			LATEbits.LATE2 = 1; // connect  led OFF
		}
		LATEbits.LATE6 = 1;
		LATEbits.LATE5 = 1;
	}

	if (emulat_type == E220) {
		if (PIR3bits.RC2IF) { // is data from touchscreen
			timer0.lt = TIMERPACKET; // set timer to charge rate time
			TMR0H = timer0.bt[1]; // Write high byte to Timer0
			TMR0L = timer0.bt[0]; // Write low byte to Timer0
			// clear  TMR0  flag
			INTCONbits.TMR0IF = 0; //clear interrupt flag
			if (CAM && (cam_time > MAX_CAM_TIME)) {
				CAM_RELAY_TIME = 0;
				CAM_RELAY_AUX = 0; // clear video switch
				CAM_RELAY = 0; // clear video switch
				CAM = FALSE;
			}

			c = RCREG2; // read data from touchscreen
			if (do_cap) {
				if (scrn_ptr < CAP_SIZE) {
					scrn_rec[scrn_ptr] = RCREG2; // read data from touchscreen
					while (!TXSTA1bits.TRMT) {
					}; // wait until the usart is clear
					TXREG1 = scrn_rec[scrn_ptr];
					scrn_ptr++;
					LATJbits.LATJ2 = !LATJbits.LATJ2; // flash  led
				} else {
					scrn_write = TRUE;
					LATJbits.LATJ4 = !LATJbits.LATJ4; // flash  led
				}
			} else {
				if (screen_type == DELL_E215546) {
					ssbuf[idx] = c;
					switch (idx++) {
					case 0: // start of touch controller packet, save data and compute checksum
						sum = 0xaa;
						if (c != 'U') {
							idx = 0;
							LATEbits.LATE6 = !LATEbits.LATE6;
						}
						break;
					case 9: // end of touch controller packet
						LATEbits.LATE0 = 1; // flash external led
						LATEbits.LATE7 = LATEbits.LATE0; // flash external led
						idx = 0;
						if (c != sum) { // bad checksum
							LATEbits.LATE6 = !LATEbits.LATE6;
							break;
						}
						if (ssbuf[1] == 'T') {
							LATEbits.LATE4 = !LATEbits.LATE4;
							restart_delay = 0;
							CATCH = TRUE;
							if (!ssreport.tohost) {
								ssreport.x_cord = (ELO_REV_H - (((uint16_t) ssbuf[3])+(((uint16_t) ssbuf[4]) << 8))) >> 4;
								ssreport.y_cord = (((uint16_t) ssbuf[5])+(((uint16_t) ssbuf[6]) << 8)) >> 4;
							}
							LATF = ssreport.y_cord;
						} else if (ssbuf[1] == 'A') {
							restart_delay = 0;
							LATJbits.LATJ6 = 0; // led 6 touch-screen connected
							speedup = -10000;
						}
						break;
					}
					sum += c;
					DATA2 = TRUE; // usart is connected to data

				}
				if (screen_type == DELL_E224864) {
					touch_good++; // chars received before a status report
					LATEbits.LATE0 = 1; // flash external led
					LATEbits.LATE7 = LATEbits.LATE0; // flash external led
					DATA2 = TRUE; // usart is connected to data
					if (TOUCH) {
						elobuf[c_idx++] = c;
						LATEbits.LATE0 = 1; // flash external led
						LATEbits.LATE7 = LATEbits.LATE0; // flash external led
						if (c == 0xFF && TOUCH) { // end of report
							CATCH = TRUE;
							restart_delay = 0;
							TOUCH = FALSE; // stop buffering touchscreen data.
						};
					};
					if (c == 0xFE && (!CATCH)) { // looks like a touch report
						TOUCH = TRUE;
						TSTATUS = TRUE;
						restart_delay = 0;
						CATCH = FALSE;
						c_idx = 0;
						LATEbits.LATE0 = 1; // flash external led
						LATEbits.LATE7 = LATEbits.LATE0; // flash external led
					};
					if (c == 0xF5) { // looks like a status report
						TSTATUS = TRUE;
						restart_delay = 0;
						LATJbits.LATJ6 = 0; // led 6 touch-screen connected
						speedup = -10000;
						LATEbits.LATE0 = 1; // flash external led
						LATEbits.LATE7 = LATEbits.LATE0; // flash external led
					};
					if (c_idx > (BUF_SIZE - 2)) {
						c_idx = 0; // stop buffer-overflow
						TOUCH = FALSE;
						CATCH = FALSE;
					};
					if (touch_good > GOOD_MAX) { // check for max count and no host to get touch data
						LATEbits.LATE0 = 1; // LED off
						LATEbits.LATE7 = LATEbits.LATE0;
						while (TRUE) { // lockup for reboot
							touch_good++;
						};
					}
				}
			}
		}
	}

	if (emulat_type == VIISION && screen_type == DELL_E215546) { // This is for the newer SMARTSET intellitouch screens
		if (PIR3bits.RC2IF) { // is data from screen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}

			/* Get the character received from the USART */
			c = RCREG2;

			if (((c & 0xc0) == 0xc0) || CATCH) { // start of touch sequence
				LATFbits.LATF0 = 0;
				CATCH = TRUE; // found elo touch command start of sequence
				j = 0; // reset led timer
				elobuf[i++] = c; // start stuffing the command buffer
			}
			if (i == CMD_SIZE_SS_V80) { // see if we should send it
				LATFbits.LATF5 = !LATFbits.LATF5;
				i = FALSE; // reset i to start of cmd
				uchar = 0; /* check for proper touch format */
				if ((elobuf[0]& 0xc0) == 0xc0) /* binary start code? */
					uchar = TRUE;

				LATFbits.LATF1 = 0;
				CATCH = FALSE; // reset buffering now

				/* munge the data for proper Varian format */
				if (elobuf[5]) { // TOUCH
					TOUCH = TRUE; // first touch sequence has been sent
					uvalx = elobuf[0]&0x3f; // prune the data to 6-bits
					lvalx = elobuf[1]&0x3f;
					uvaly = elobuf[2]&0x3f;
					lvaly = elobuf[3]&0x3f;
					x_tmp = lvalx | (uvalx << 6); // 12-bit X value
					y_tmp = lvaly | (uvaly << 6); // 12-bit Y value
					x_tmp = 4095 - x_tmp; // FLIP X
					y_tmp = 4095 - y_tmp; // FLIP Y	
					x_tmp = (uint16_t) ((float) x_tmp * (float) xs_ss); // X rescale range
					y_tmp = (uint16_t) ((float) y_tmp * (float) ys_ss); // Y rescale
					x_tmp = (x_tmp >> (uint16_t) 4); // rescale x to 8-bit value
					y_tmp = (y_tmp >> (uint16_t) 4); // rescale y				
					elobuf_in[1] = x_tmp; // X to 8-bit var
					elobuf_in[2] = y_tmp; // Y 
					elobuf_out[0] = 0xc0 + ((elobuf_in[1]&0xc0) >> 6); // stuff into binary 4002 format
					elobuf_out[1] = 0x80 + (elobuf_in[1]&0x3f);
					elobuf_out[2] = 0x40 + ((elobuf_in[2]&0xc0) >> 6);
					elobuf_out[3] = 0x00 + (elobuf_in[2]&0x3f);
					elobuf_out[4] = 0x00;
					elobuf_out[5] = 0x0f;
					LATFbits.LATF6 = 0;
				}

				if (!elobuf[5]) { //UNTOUCH
					UNTOUCH = TRUE; // untouch seqence found
					elobuf_out[0] = 0xc0; // restuff the buffer with needed varian untouch sequence
					elobuf_out[1] = 0x80;
					elobuf_out[2] = 0x40;
					elobuf_out[3] = 0x00;
					elobuf_out[4] = 0x00;
					elobuf_out[5] = 0x00;
					LATFbits.LATF7 = 0;
				}

				if (TOUCH || UNTOUCH) { // send both
					LATFbits.LATF2 = 0;
					if (uchar) { /* only send valid data */
						LATFbits.LATF3 = 0;
						data_ptr = elobuf_out;
						data_pos = 0;
						data_len = HOST_CMD_SIZE_V80;
						PIE1bits.TX1IE = 1; // start sending data
					}
					LCD_OK = TRUE; // looks like a screen controller is connected
					SCREEN_INIT = FALSE; // command code has been received by lcd controller
					LATEbits.LATE3 = 1; // init  led OFF
					init_check = 0; // reset init code timer
					LATEbits.LATE2 = 0; // connect  led ON

					if (UNTOUCH) { // After untouch is sent dump buffer and clear all.
						TOUCH = FALSE;
						UNTOUCH = FALSE;
						CATCH = FALSE;
						LATFbits.LATF4 = 0;
						WriteTimer0(timer0_off);
					}
				}
			}

			/* Clear the interrupt flag */
			if (i > CMD_OVERFLOW_V80) {
				i = 0; // just incase i is greater than CMD_SIZE*2 somehow
				CATCH = FALSE;
				TOUCH = FALSE;
				UNTOUCH = FALSE;
				LATF = 0xff;
			}

			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE6 = !LATEbits.LATE6; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led
		}
	}

	if (emulat_type == VIISION && screen_type == DELL_E224864) { // This is for the DELL ELO Carroltouch screen.
		if (PIR3bits.RC2IF) { // is data from touchscreen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}
			/* Get the character received from the USART */
			c = RCREG2;

			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE5 = !LATEbits.LATE5; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led

			// touch 'FE X Y FF',    untouch 'FD X Y FF' from screen,    'F4 X Y FF' frame size report

			if (CATCH || (c == 0xFE) || (c == 0xFD) || (c == 0xF4)) { // in frame or start of touch or untouch sequence or frame size report
				LATFbits.LATF0 = 0;
				CATCH = TRUE; // found elo CT touch command start of sequence, we hope
				elobuf_in[i++] = c; // start stuffing the command buffer
				j = 0; // reset led timer
			}

			if ((i == CMD_SIZE_V80) && (elobuf_in[3] == 0xFF)) { // see if we should send it, right size and end char
				LATFbits.LATF5 = !LATFbits.LATF5;
				i = 0; // reset i to start of cmd frame
				CATCH = FALSE; // reset buffering now
				uchar = elobuf_in[i]; //  load into uchar

				if (uchar == 0xFE) { // touch sequence found restuff the buffer with varian touch sequence
					TOUCH = TRUE; // set TOUCH flag after first touch
					elobuf_in[2] = yl - elobuf_in[2]; // FLIP Y
					elobuf_in[1] = (uint8_t) ((float) elobuf_in[1]* (float) xs); // X scale
					elobuf_in[2] = (uint8_t) ((float) elobuf_in[2]* (float) ys); // Y scale
					elobuf_out[i ] = 0xc0 + ((elobuf_in[1]&0xc0) >> 6); // stuff into binary 4002 format
					elobuf_out[i + 1] = 0x80 + (elobuf_in[1]&0x3f);
					elobuf_out[i + 2] = 0x40 + ((elobuf_in[2]&0xc0) >> 6);
					elobuf_out[i + 3] = 0x00 + (elobuf_in[2]&0x3f);
					elobuf_out[i + 4] = 0x00;
					elobuf_out[i + 5] = 0x15; // Z value = 15 "hard touch"
					LATFbits.LATF6 = 0;
				}

				if (uchar == 0xFD) { // this is a untouch command
					UNTOUCH = TRUE; // untouch sequence found
					elobuf_out[i] = 0xc0; // restuff the buffer with varian binary 4002 untouch sequence
					elobuf_out[i + 1] = 0x80;
					elobuf_out[i + 2] = 0x40;
					elobuf_out[i + 3] = 0x00;
					elobuf_out[i + 4] = 0x00;
					elobuf_out[i + 5] = 0x00;
					LATFbits.LATF7 = 0;
				}

				if (!TOUCH && !UNTOUCH) { // check for proper touch frames
					if (uchar == 0xF4) { // check for frame size report
						LCD_OK = TRUE; // looks like a screen controller is connected
						SCREEN_INIT = FALSE; // command code has been received by lcd controller
						LATEbits.LATE3 = 1; // init  led OFF
						init_check = 0; // reset init code timer
						LATEbits.LATE2 = 0; // connect  led ON
					}
				}

				if (TOUCH || UNTOUCH) { // send both
					LATFbits.LATF2 = 0;
					data_ptr = elobuf_out;
					data_pos = 0;
					data_len = HOST_CMD_SIZE_V80;
					PIE1bits.TX1IE = 1; // start sending data
				}

				if (UNTOUCH) { // cleanup and reset for next touch
					TOUCH = FALSE;
					UNTOUCH = FALSE;
					CATCH = FALSE;
					LATFbits.LATF4 = 0;
					WriteTimer0(timer0_off);
				}
			}

			if (i > CMD_OVERFLOW_V80) {
				i = 0; // just incase i is greater than overflow somehow
				CATCH = FALSE;
				TOUCH = FALSE;
				UNTOUCH = FALSE;
				LATF = 0xff;
			}

			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE6 = !LATEbits.LATE6; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led
		}
	}

	if (emulat_type == OTHER) {
		if (PIR3bits.RC2IF) { // is data from touchscreen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}
			/* Get the character received from the USART */
			c = RCREG2;

			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE5 = !LATEbits.LATE5; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led


		}
	}
}

void touch_cam(void)
{

	//	check for corner presses
	if (CATCH) {
		if ((elobuf[0] <= (uint8_t) 0x06) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for left bottom corner
			touch_corner1++;
			touch_corner_timed = TRUE;
		};

		if ((elobuf[0] >= (uint8_t) 0x72) && (elobuf[1] >= (uint8_t) 0x5a)) { // check for right bottom corner
			touch_corner1++;
		};
	};

	if (touch_corner1 >= MAX_CAM_TOUCH) { // we have several corner presses 
		CAM = TRUE;
		cam_time = 0;
		CAM_RELAY_TIME = 1;
		touch_corner1 = 0;
		CAM_RELAY_AUX = 1; // set secondary VGA/CAM switch
		CAM_RELAY = 1; // set primary VGA/CAM switch
		elobuf[0] = 0;
		elobuf[1] = 0;
	};
}

void wdtdelay(uint32_t delay)
{
	uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void elocmdout(uint8_t * elostr)
{
	LATJbits.LATJ5 = !LATJbits.LATJ5; // touch screen commands led
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(elostr[0]);
	while (Busy2USART()) {
	}; // wait until the usart is clear
	LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
	LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
	LATEbits.LATE7 = LATEbits.LATE0; // flash external led
	wdtdelay(30000);
}

void eloSScmdout(uint8_t elostr)
{
	LATJbits.LATJ5 = !LATJbits.LATJ5; // touch screen commands led
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(elostr);
	while (Busy2USART()) {
	}; // wait until the usart is clear
	LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
	LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
	LATEbits.LATE7 = LATEbits.LATE0; // flash external led
	wdtdelay(10000); // inter char delay
}

void elopacketout(uint8_t *strptr, uint8_t strcount, uint8_t slow)
{
	uint8_t i, c, sum = 0;

	for (i = 0; i < strcount; i++) {
		switch (i) {
		case 0:
			c = 'U';
			sum = 0xAA + 'U';
			break;
		case 9:
			c = sum;
			break;
		default:
			sum += (c = strptr[i]);
		}

		eloSScmdout(c);
	};
	if (slow) wdtdelay(30000);
}

void elocmdout_v80(const rom uint8_t * elostr)
{
	int16_t e;
	int8_t elo_char;
	for (e = 0; e < ELO_SIZE_V80; e++) { // send buffered data
		while (Busy2USART()) {
		}; // wait until the usart is clear
		elo_char = elostr[e];
		putc2USART(elo_char); // send to LCD touch
		LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
		LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
		LATEbits.LATE7 = LATEbits.LATE0; // flash external led
		wdtdelay(10000); // inter char delay
	}
	wdtdelay(50000); // wait for LCD controller reset
}

void setup_lcd(void)
{
	uint16_t code_count;
	uint8_t single_t = SINGLE_TOUCH;

	if (do_emu_ss) {
		elopacketout(elocodes_e0, ELO_SEQ, 0); // set touch packet spacing and timing
	} else {

		if (TS_TYPE == 1) single_t = FALSE;
		for (code_count = 0; code_count < ELO_SIZE_V80; code_count++) {
			if (single_t) {
				elocmdout(&elocodes_s_e[code_count]);
			} else {
				elocmdout(&elocodes_m_e[code_count]);
			}
		};
	}

	NEEDSETUP = FALSE;
}

void putc1(uint16_t c)
{
	while (Busy1USART()) {
	}; // wait until the usart is clear
	putc1USART(c);
}

void putc2(uint16_t c)
{
	while (Busy2USART()) {
	}; // wait until the usart is clear
	putc2USART(c);
}

void start_delay(void)
{
	wdtdelay(100000);
}

uint8_t Test_Screen(void)
{
	while (Busy2USART()) {
	}; // wait until the usart is clear
	if (screen_type == DELL_E224864) return TRUE;
	putc2(0x46);
	wdtdelay(30000);
	if (DATA2) {
		setup_lcd(); // send lcd touch controller setup codes
		return TRUE;
	} else {
		setup_lcd(); // send lcd touch controller setup codes
		return FALSE;
	}
}

void Cylon_Eye(uint8_t invert)
{
	static uint8_t cylon = 0xfe, LED_UP = TRUE;
	static int32_t alive_led = 0xfe;

	if (invert) { // screen status feedback
		LATD = ~cylon; // roll leds cylon style
	} else {
		LATD = cylon; // roll leds cylon style (inverted)
	}

	if (LED_UP && (alive_led != 0)) {
		alive_led = alive_led * 2;
		cylon = cylon << 1;
	} else {
		if (alive_led != 0) alive_led = alive_led / 2;
		cylon = cylon >> 1;
	}
	if (alive_led < 2) {
		alive_led = 2;
		LED_UP = TRUE;
	} else {
		if (alive_led > 128) {
			alive_led = 128;
			LED_UP = FALSE;
		}
	}
}

void main(void)
{
	uint8_t z, check_byte;
	uint16_t eep_ptr;
	uint8_t scaled_char;
	float rez_scale_h = 1.0, rez_parm_h, rez_scale_v = 1.0, rez_parm_v;
	float rez_scale_h_ss = ELO_SS_H_SCALE, rez_scale_v_ss = ELO_SS_V_SCALE;

	INTCON = 0;
	INTCON3bits.INT1IE = 0;
	INTCON3bits.INT2IE = 0;
	INTCON3bits.INT3IE = 0;
	screen_type = DELL_E215546;
	emulat_type = VIISION;
	/* Configure  PORT pins for output */
	TRISA = 0;
	LATA = 0;
	/* check for touchscreen configuration data and setup switch on port B */
	INTCON2bits.RBPU = 0;
	TRISB = 0xff; // inputs
	LATB = 0xff;
	z = PORTB;
	wdtdelay(7000);
	if (z != PORTB) // glitch check
		z = 0xff;
	TRISB = 0; // outputs
	Busy_eep();
	check_byte = Read_b_eep(0);

	if (z != 0xff) {
		Busy_eep();
		Write_b_eep(0, 0x57);
		Busy_eep();
		Write_b_eep(1, z);

	}

	Busy_eep();
	check_byte = Read_b_eep(0);
	if (check_byte == 0x57) {
		Busy_eep();
		z = Read_b_eep(1);
		if (z == 0b11111110) {
			screen_type = DELL_E215546;
			emulat_type = VIISION;
		}
		if (z == 0b11111010) {
			screen_type = DELL_E224864;
			emulat_type = VIISION;
		}
		if (z == 0b11111101) {
			screen_type = DELL_E215546;
			emulat_type = E220;
		}
		if (z == 0b11111001) {
			screen_type = DELL_E224864;
			emulat_type = E220;
		}
		if (z == 0b11111100) {
			screen_type = DELL_E215546;
			emulat_type = OTHER;
		}
		if (z == 0b11111000) {
			screen_type = DELL_E224864;
			emulat_type = OTHER;
		}
	}

	TRISB = 0; // outputs
	TRISC = 0;
	LATC = 0;
	TRISD = 0;
	LATD = z; // show EEPROM configuration data here
	TRISE = 0;
	LATE = 0xFF;
	TRISF = 0;
	LATF = 0xFF;
	TRISG = 0;
	LATG = 0;
	TRISH = 0;
	LATH = 0;
	TRISJ = 0;
	LATJ = 0;

	CAM_RELAY_TIME = 0;
	CAM_RELAY = 0;
	touch_count = 0;
	CAM = 0;
	ssreport.tohost = TRUE;

	LATEbits.LATE3 = 0; // init  led ON
	wdtdelay(700000); // wait for LCD controller reset on power up
	LATEbits.LATE3 = 1; // init  led OFF

	if (emulat_type == OTHER) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */
		Open1USART(USART_TX_INT_ON &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		/* TouchScreen */
		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		WriteTimer0(timer0_off); //	start timer0 at 1 second ticks
	}

	if (emulat_type == VIISION) {
		/*
		 * Open the USART configured as
		 * 8N1, 9600 baud, in /transmit/receive INT mode
		 */
		/* Host */
		Open1USART(USART_TX_INT_ON &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		/* TouchScreen */
		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
		WriteTimer0(timer0_off); //	start timer0 at 1 second ticks

		if (screen_type == DELL_E224864) {
			elocmdout_v80(elocodes_s_v); // send touchscreen setup data, causes a frame size report to be send from screen
		}
		if (screen_type == DELL_E215546) {
			elocmdout_v80(&elocodes[7][0]); // reset;
			wdtdelay(700000); // wait for LCD touch controller reset
			/* program the display */
			elocmdout_v80(&elocodes[0][0]);
			elocmdout_v80(&elocodes[4][0]);
			elocmdout_v80(&elocodes[5][0]);
			elocmdout_v80(&elocodes[6][0]);
		}
	}

	if (emulat_type == E220) {
		/*
		 * Open the USART configured as0
		 * 8N1, 9600 baud, in receive INT mode
		 */
		Open1USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64);
		// 40mhz osc HS: 9600 baud, USART_BRGH_LOW and 64, for 9600 baud @ 40MHz   (0.16% ERROR IN RATE)

		Open2USART(USART_TX_INT_OFF &
			USART_RX_INT_ON &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_LOW, 64);

		OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
		WriteTimer0(TIMERPACKET); //	start timer0 

		if (!do_cap) {
			setup_lcd(); // send lcd touch controller setup codes
		}
	}

	/* Display a prompt to the USART */
	putrs1USART(build_version);

	while (DataRdy1USART()) { // dump rx data
		z = Read1USART();
	};
	while (DataRdy2USART()) { // dump rx data
		z = Read2USART();
	};

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	PIR1bits.RCIF = 0;
	PIR3bits.RC2IF = 0;
	PIR1bits.TX1IF = 0;
	PIR3bits.TX2IF = 0;
	INTCONbits.GIEL = 0; // disable low ints
	INTCONbits.GIEH = 1; // enable high ints

	if (emulat_type == E220) {
		LATJ = 0xff; // set leds to off at powerup/reset
		DATA1 = FALSE; // reset COMM flags.
		DATA2 = FALSE; // reset touch COMM flag
		// leds from outputs to ground via resistor.
		while (do_cap) {
			ClrWdt();
			LATJbits.LATJ6 = !LATJbits.LATJ6; // flash  led
			if (host_write || scrn_write) {
				eep_ptr = 0;
				while (eep_ptr <= 255) {
					LATJbits.LATJ0 = !LATJbits.LATJ0; // flash  led
					INTCONbits.GIE = 0; // global int enable
					INTCONbits.PEIE = 0; // enable all unmasked int
					Busy_eep();
					Write_b_eep(eep_ptr, host_rec[eep_ptr]); //  data
					ClrWdt(); // reset the WDT timer
					Busy_eep();
					Write_b_eep(eep_ptr + CAP_SIZE, scrn_rec[eep_ptr]); //  data
					Busy_eep();
					INTCONbits.GIE = 1; // global int enable
					INTCONbits.PEIE = 1; // enable all unmasked int
					ClrWdt(); // reset the WDT timer
					eep_ptr++;
				}
				INTCONbits.GIE = 0; // global int enable
				INTCONbits.PEIE = 0; // enable all unmasked int
				while (do_cap) {
					LATJ = 0x00; // all on
					ClrWdt(); // reset the WDT timer
				}
			}
		}

		Test_Screen();
		LATD = 0xff;
		/* Loop forever */
		while (TRUE) {
			if (j++ >= (BLINK_RATE_E220 + speedup)) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
				Cylon_Eye(LCD_OK);
				if (cam_time > MAX_CAM_TIMEOUT) {
					CAM_RELAY_TIME = 0;
					if (touch_corner_timed) {
						touch_corner_timed = FALSE;
						CAM_RELAY_TIME = 0;
						CAM_RELAY_AUX = 0; // clear video switch
						CAM_RELAY = 0; // clear video switch
						CAM = FALSE;
					}
				}
				cam_time++;
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
				/*		For the auto-restart switch						*/
				if (AUTO_RESTART) { // enable auto-restarts
					if ((restart_delay++ >= (uint16_t) 60) && (!TSTATUS)) { // try and reinit lcd after delay
						start_delay();
						setup_lcd(); // send lcd touch controller setup codes
						start_delay();
						while (TRUE) {
						}; // lockup WDT counter to restart
					} else {
						if ((restart_delay >= (uint16_t) 150) && (TSTATUS)) { // after delay restart TS status.
							TSTATUS = FALSE; // lost comms while connected
							restart_delay = 0;
						};
					};
				}
				j = 0;
			}

			LATJbits.LATJ4 = !LATJbits.LATJ4; // toggle bits program run led
			touch_cam(); // always check the cam touch

			if (CATCH46) { // flag to send report to host
				LATJbits.LATJ0 = 1; // flash status led
				if (CATCH) { // send the buffered touch report
					Delay10KTCYx(75); // 75 ms
					putc1(0xFE); // send position report header to host
					if (do_emu_ss) {
						ssreport.tohost = TRUE;
						rez_parm_h = ((float) (ssreport.x_cord)) * rez_scale_h_ss;
						rez_parm_v = ((float) (ssreport.y_cord)) * rez_scale_v_ss;
						ssreport.tohost = FALSE;
						scaled_char = ((uint16_t) (rez_parm_h));
						elobuf[0] = scaled_char;
						putc1(scaled_char); // send h scaled touch coord
						scaled_char = ((uint16_t) (rez_parm_v));
						elobuf[1] = scaled_char;
						putc1(scaled_char); // send v scaled touch coord
					} else {
						rez_parm_h = ((float) (elobuf[0])) * rez_scale_h;
						scaled_char = ((uint16_t) (rez_parm_h));
						putc1(scaled_char); // send h scaled touch coord
						rez_parm_v = ((float) (elobuf[1])) * rez_scale_v;
						scaled_char = ((uint16_t) (rez_parm_v));
						putc1(scaled_char); // send v scaled touch coord
						c_idx = 0;
					}
					putc1(0xFF); // send end of report to host
					touch_count++;
					CATCH = FALSE;
					CATCH46 = FALSE;
				} else { // just send status
					Delay10KTCYx(65); // 65 ms
					putc1(0xF5); // send status report
					putc1(0xFF); // end of report
					status_count++;
					CATCH46 = FALSE;
				};
			};

			if (CATCH37) { // send screen size codes
				LATJbits.LATJ7 = 0; // off blink for rez codes sent
				Delay10KTCYx(75); // 75 ms
				rez_scale_h = 1.0; // LCD touch screen real H/V rez
				rez_scale_v = 1.0;
				if (!do_emu_ss)
					putc2(0x3D); // send clear buffer to touch

				putc1(0xF4); // send status report
				if (TS_TYPE == 0) { // CRT type screens
					putc1(0x77); // touch parm
					putc1(0x5f); // touch parm
				}
				if (TS_TYPE == 1) { // new LCD type screens
					putc1(0x71); // touch parm 113
					putc1(0x59); // touch parm 89
				}
				putc1(0xFF); // end of report
				resync_count++;
				CATCH37 = FALSE;
			};

			if (TOUCH) {
				// do nothing now.
			};

			if (NEEDSETUP) setup_lcd(); // send lcdsetup codes to screen
			/*	check for port errors and clear if needed	*/
			if (RCSTA1bits.OERR) {
				LATJ = 0xFF; // all leds off with error
				RCSTA1bits.CREN = 0;
				RCSTA1bits.CREN = 1;
			}
			if (RCSTA2bits.OERR) {
				LATJ = 0xFF; // all leds off with error
				RCSTA2bits.CREN = 0;
				RCSTA2bits.CREN = 1;
			}
			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == VIISION) {
		PORTD = 0x00;
		/* Loop forever */
		PORTB = 0x0f;
		while (TRUE) { // busy loop BSG style
			if (j++ >= BLINK_RATE_V80) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led

				if (LCD_OK) { // screen status feedback
					Cylon_Eye(LCD_OK); // roll leds cylon style
					timer0_off = TIMEROFFSET;
				} else {
					Cylon_Eye(LCD_OK); // roll leds cylon style (inverted)
					timer0_off = TIMERFAST;
				}
				j = 0;
				if ((screen_type == DELL_E224864) && SCREEN_INIT && !PIE1bits.TX1IE) { // if this flag is set send elo commands
					INTCONbits.GIEH = 0;
					elocmdout_v80(elocodes_s_v); // send touchscreen setup data, causes a frame size report to be send from screen
					INTCONbits.GIEH = 1;
					SCREEN_INIT = FALSE; // commands sent, now wait for reply to set LCD_OK flag
					LATEbits.LATE3 = 1; // init  led OFF
				}
			}
			ClrWdt(); // reset the WDT timer
		}
	}

	if (emulat_type == OTHER) {
		PORTD = 0x00;
		/* Loop forever */
		PORTB = 0x0f;
		while (TRUE) { // busy loop BSG style
			if (j++ >= BLINK_RATE_OTHER) { // delay a bit ok
				LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
				LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
				LATEbits.LATE7 = LATEbits.LATE0; // flash external led
				INTCONbits.GIEH = 0;
				Cylon_Eye((screen_type == DELL_E224864));
				j = 0;
			}
			ClrWdt(); // reset the WDT timer
		}
	}
}
