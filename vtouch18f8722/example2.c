
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
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
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
 * This program converts the rs-232 output from a ELO 1939L E224864 CarrollTouch controller type LCD monitor 
 * to a format that can be used with the Varian Viision 80 Implanter with ADYIN CRT monitor
 * The Carroll touchscreen will be  programmed to the correct touch response and configuration at program start
 *
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 *
 * Fred Brooks, Microchip Inc , Feb 2013
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
 * V3.01	mode checks and lamps
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

#include <usart.h>
#include <timers.h>
#include <stdlib.h>
#include <EEP.h>

void rxtx_handler(void);

#define BUF_SIZE 16
#define	CMD_SIZE 4					// size of command in bytes from touch screen
#define	HOST_CMD_SIZE	6
#define	CMD_OVERFLOW	CMD_SIZE*2
#define ELO_SEQ 10
#define ELO_SIZE 14					// number of bytes to send from elocodes_s configuration string
#define ELO_SIZE_I 10
#define FALSE	0
#define TRUE	1
#define	BLINK_RATE	35000
#define	X_SCALE	1.90				// scaling factor to host screen X logical coords
#define	Y_SCALE 1.75				// scaling factor to host screen Y logical coords
#define	X_LOGICAL	119				// LCD touchscreen logical X frame coords
#define	Y_LOGICAL	94				// LCD touchscreen logical Y frame coords
#define	X_TOOL	202
#define	Y_TOOL	164

#define	TIMEROFFSET	26474			// timer0 16bit counter value for 1 second to overflow
#define	TIMERFAST	58974			// fast flash or testing
#define	COMM_CHK_TIME	30			// LCD comm heartbeat
#define	LCD_CHK_TIME	36			// LCD heartbeat timeout
#define TOUCH_SKIP		0		// max touchs to skip before a untouch.

const rom char version[] = "VERSION 3.01";
volatile unsigned char CATCH = FALSE, LED_UP = TRUE, TOUCH = FALSE, UNTOUCH = FALSE, LCD_OK = FALSE, comm_check = 0, init_check = 0,
	SCREEN_INIT = FALSE;

enum screen_type_t {
	DELL_E224864, DELL_OTHER, OTHER, SMARTSET
} screen_type;

long alive_led = 0;
volatile long j = 0;
volatile float xs = X_SCALE, ys = Y_SCALE; // defaults
volatile unsigned int timer0_off = TIMEROFFSET;

#pragma idata bigdata
volatile unsigned char elobuf[BUF_SIZE], elobuf_out[BUF_SIZE], elobuf_in[BUF_SIZE], testing_data, xl = X_LOGICAL, yl = Y_LOGICAL;

const rom unsigned char elocodes_s[] = {
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x3c, 0x2b, 0x44, 0x25, 0x29, 0x44, 0x3d, 0x2a, 0x37
}; // initial carrol-touch config codes, tracking, add end point modifier, get frame size report

const rom unsigned char elocodes[ELO_SEQ][ELO_SIZE_I] = {// elo 2210/2216 program codes
	'U', 'M', 0x00, 0x85, 0x08, '0', '0', '0', '0', '0', // initial touch,Single Point,untouch,Z-axis,scaling
	'U', 'S', 'x', 0xf6, 0xff, 0x04, 0x01, 0xff, 0x0f, '0', // scale x
	'U', 'S', 'y', 0xf6, 0xff, 0xc3, 0x00, 0xff, 0x0f, '0', // scale y
	'U', 'S', 'z', 0x01, 0x00, 0x0e, 0x00, 0xff, 0x00, '0', // scale z
	'U', 'E', '1', '6', '0', '0', '0', '0', '0', '0', // emulation E281A-4002 Binary (Z=0 on untouch)
	'U', 'N', '1', '7', '0', '0', '0', '0', '0', '0' // nvram save
}; // initial intelli-touch codes												//

const rom char *build_date = __DATE__, *build_time = __TIME__, build_version[] = " V3.01 8722 Varian touch-screen converter. Fred Brooks, Microchip Inc.";
#pragma idata

#pragma code rx_interrupt = 0x8

void rx_int(void)
{
	_asm goto rxtx_handler _endasm
}
#pragma code

#pragma interrupt rxtx_handler

void rxtx_handler(void) // all serial data transform functions are handled here
{
	static unsigned char c1 = 0, c2 = 0, c = 0, *data_ptr,
		i = 0, data_pos, data_len, tchar, uchar;

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for host link */
	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		if (data_pos >= data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = 0; // stop data xmit
			}
		} else {
			TXREG1 = *data_ptr; // send data and clear PIR1bits.TX1IF
			LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led
			data_pos++; // move the data pointer
			data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR1bits.RCIF) { // is data from host COMM1
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = 0; //	clear overrun
			RCSTA1bits.CREN = 1; // re-enable
		}
		c1 = RCREG1;
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer
		//check for TMR0 overflow
		INTCONbits.TMR0IF = 0; //clear interrupt flag
		WriteTimer0(timer0_off);
		LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
		LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
		LATEbits.LATE7 = LATEbits.LATE0; // flash external led

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
			if (screen_type == SMARTSET) {
				LATF = 0xff;
			}
			LATEbits.LATE1 = !LATEbits.LATE1; // flash  led
			LATEbits.LATE2 = 1; // connect  led OFF
		}
		LATEbits.LATE6 = 1;
		LATEbits.LATE5 = 1;
	}

	if (screen_type == SMARTSET) { // This is for a future SCREEN
		if (PIR3bits.RC2IF) { // is data from screen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}

			/* Get the character received from the USART */
			c = RCREG2;
			tchar = c;

			if (((tchar & 0xc0) == 0xc0) || CATCH) { // start of touch sequence
				LATFbits.LATF0 = 0;
				CATCH = TRUE; // found elo touch command start of sequence
				j = 0; // reset led timer
				elobuf[i++] = c; // start stuffing the command buffer
			}
			if (i == CMD_SIZE) { // see if we should send it
				i = 0; // reset i to start of cmd
				LATFbits.LATF1 = 0;
				CATCH = FALSE; // reset buffering now
				if (elobuf[0] == 0xc0) elobuf[5]=0x0f; // touch value for untouch
				UNTOUCH = TRUE; // untouch seqence found
//				elobuf[6] = 0xc0; // restuff the buffer with varian untouch sequence
//				elobuf[7] = 0x80;
//				elobuf[8] = 0x40;
//				elobuf[9] = 0x00;
//				elobuf[10] = 0x00;
//				elobuf[11] = 0x00;

				TOUCH = TRUE;

				if ((TOUCH) && (UNTOUCH == FALSE)) { // return on touch stream and reset index i
					i = 0; // need single-point mode so don't process more touch streams
					LATFbits.LATF2 = 0;
				} else {
					LATFbits.LATF3 = 0;
					data_ptr = elobuf;
					data_pos = 0;
					data_len = HOST_CMD_SIZE;
					PIE1bits.TX1IE = 1; // start sending data

					i = 0;
					TOUCH = TRUE; // first touch sequence has been sent

					LCD_OK = TRUE; // looks like a screen controller is connected
					SCREEN_INIT = FALSE; // command code has been received by lcd controller
					LATEbits.LATE3 = 1; // init  led OFF
					init_check = 0; // reset init code timer
					LATEbits.LATE2 = 0; // connect  led ON

					if (UNTOUCH) { // After untouch is sent dump buffer and clear all.
						TOUCH = FALSE;
						UNTOUCH = FALSE;
						CATCH = FALSE;
						i = 0;
						LATFbits.LATF4 = 0;
					}
				}
			}

			/* Clear the interrupt flag */
			if (i > CMD_OVERFLOW) {
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

	if (screen_type == DELL_E224864) { // This is for the DELL ELO Carroltouch screen.
		if (PIR3bits.RC2IF) { // is data from touchscreen COMM2
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0; //	clear overrun
				RCSTA2bits.CREN = 1; // re-enable
			}
			/* Get the character received from the USART */
			c2 = RCREG2;
			tchar = c2;

			LATEbits.LATE0 = !LATEbits.LATE0; // flash external led
			LATEbits.LATE5 = !LATEbits.LATE5; // flash external led
			LATEbits.LATE7 = LATEbits.LATE0; // flash external led

			// touch 'FE X Y FF',    untouch 'FD X Y FF' from screen,    'F4 X Y FF' frame size report

			if (CATCH || (tchar == 0xFE) || (tchar == 0xFD) || (tchar == 0xF4)) { // in frame or start of touch or untouch sequence or frame size report
				CATCH = TRUE; // found elo CT touch command start of sequence, we hope
				elobuf_in[i++] = tchar; // start stuffing the command buffer
				j = 0; // reset led timer
			}

			if ((i == CMD_SIZE) && (elobuf_in[3] == 0xFF)) { // see if we should send it, right size and end char

				i = 0; // reset i to start of cmd frame
				CATCH = FALSE; // reset buffering now
				uchar = elobuf_in[i]; //  load into uchar

				if (uchar == 0xFE) { // touch sequence found restuff the buffer with varian touch sequence
					TOUCH = TRUE; // set TOUCH flag after first touch
					elobuf_in[2] = yl - elobuf_in[2]; // FLIP Y
					elobuf_in[1] = (unsigned char) ((float) elobuf_in[1]* (float) xs); // X scale
					elobuf_in[2] = (unsigned char) ((float) elobuf_in[2]* (float) ys); // Y scale
					elobuf_out[i ] = 0xc0 + ((elobuf_in[1]&0xc0) >> 6); // stuff into binary 4002 format
					elobuf_out[i + 1] = 0x80 + (elobuf_in[1]&0x3f);
					elobuf_out[i + 2] = 0x40 + ((elobuf_in[2]&0xc0) >> 6);
					elobuf_out[i + 3] = 0x00 + (elobuf_in[2]&0x3f);
					elobuf_out[i + 4] = 0x00;
					elobuf_out[i + 5] = 0x15; // Z value = 15 "hard touch"
				}

				if (uchar == 0xFD) { // this is a untouch command
					UNTOUCH = TRUE; // untouch sequence found
					elobuf_out[i] = 0xc0; // restuff the buffer with varian binary 4002 untouch sequence
					elobuf_out[i + 1] = 0x80;
					elobuf_out[i + 2] = 0x40;
					elobuf_out[i + 3] = 0x00;
					elobuf_out[i + 4] = 0x00;
					elobuf_out[i + 5] = 0x00;
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

				if (TOUCH || UNTOUCH) {
					data_ptr = elobuf_out;
					data_pos = 0;
					data_len = HOST_CMD_SIZE;
					PIE1bits.TX1IE = 1; // start sending data
				}

				if (UNTOUCH) { // cleanup and reset for next touch
					TOUCH = FALSE;
					UNTOUCH = FALSE;
					CATCH = FALSE;
				}
			}

			if (i > CMD_OVERFLOW) {
				i = 0; // just incase i is greater than overflow somehow
				CATCH = FALSE;
				TOUCH = FALSE;
				UNTOUCH = FALSE;
				LATF = 0xff;
			}
		}
	}
}

void wdtdelay(unsigned long delay)
{
	unsigned long dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void elocmdout(const rom unsigned char *elostr)
{
	int e;
	char elo_char;
	for (e = 0; e < ELO_SIZE; e++) { // send buffered data
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

void main(void)
{
	unsigned char cylon = 0xfe, z;

	screen_type = SMARTSET;

	/* Configure  PORT pins for output */
	TRISA = 0;
	LATA = 0;

	TRISB = 0;
	LATB = 0;

	TRISC = 0;
	LATC = 0;

	TRISD = 0;
	LATD = 0;

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

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;

	/* Make receive interrupt high priority */
	IPR1bits.RC1IP = 1;
	IPR3bits.RC2IP = 1;

	while (DataRdy1USART()) { // dump rx data
		z = Read1USART();
	};
	while (DataRdy2USART()) { // dump rx data
		z = Read2USART();
	};

	/* Display a prompt to the USART */
	putrs1USART(
		(const far rom char *) "\n\rVarian touch-screen converter. Fred Brooks, Microchip Inc. \n\r");

	LATEbits.LATE3 = 0; // init  led ON
	wdtdelay(700000); // wait for LCD controller reset on power up
	if (screen_type == DELL_E224864) {
		elocmdout(elocodes_s); // send touchscreen setup data, causes a frame size report to be send from screen
	}
	if (screen_type == SMARTSET) {
		/* program the display */
		elocmdout(&elocodes[0][0]);
		elocmdout(&elocodes[1][0]);
		elocmdout(&elocodes[2][0]);
		elocmdout(&elocodes[3][0]);
		elocmdout(&elocodes[4][0]);
		elocmdout(&elocodes[5][0]);
	}
	LATEbits.LATE3 = 1; // init  led OFF

	/* Enable all high priority interrupts */
	INTCONbits.TMR0IE = 1;
	INTCONbits.RBIE = 0; // disable B int
	INTCONbits.INT0IE = 0; // clear external ints
	INTCON3bits.INT1IE = 0;
	INTCON3bits.INT2IE = 0;
	INTCON3bits.INT3IE = 0;
	PIR1bits.RCIF = 0;
	PIR3bits.RC2IF = 0;
	INTCONbits.GIEL = 0; // disable low ints
	INTCONbits.GIEH = 1; // enable high ints

	PORTD = 0x00;
	alive_led = 0xfe;

	/* Loop forever */
	PORTB = 0x0f;
	while (TRUE) { // busy loop BSG style

		if (j++ >= BLINK_RATE) { // delay a bit ok
			INTCONbits.GIEH = 0;
			if (LCD_OK) { // screen status feedback
				LATD = ~cylon; // roll leds cylon style
				timer0_off = TIMEROFFSET;
			} else {
				LATD = cylon; // roll leds cylon style (inverted)
				timer0_off = TIMERFAST;
			}
			INTCONbits.GIEH = 1;

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
			j = 0;
			ClrWdt(); // reset the WDT timer
			if ((screen_type == DELL_E224864) && SCREEN_INIT && !PIE1bits.TX1IE) { // if this flag is set send elo commands
				INTCONbits.GIEH = 0;
				elocmdout(elocodes_s); // send touchscreen setup data, causes a frame size report to be send from screen
				INTCONbits.GIEH = 1;
				SCREEN_INIT = FALSE; // commands sent, now wait for reply to set LCD_OK flag
				LATEbits.LATE3 = 1; // init  led OFF
			}
		}
		ClrWdt(); // reset the WDT timer
	};
}
