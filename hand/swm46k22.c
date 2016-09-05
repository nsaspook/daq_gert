#pragma	config	FOSC = INTIO67
#pragma config	PLLCFG=ON
#pragma config	WDTEN = OFF, WDTPS = 1024
#pragma config	CCP2MX = PORTC1, PBADEN = OFF, CCP3MX = PORTE0, T3CMX = PORTC0, P2BMX = PORTD2

/*
 *
 *				E0.20	RGBA LED PWM color mixer CTMU touch driver
 *				***		background I/O using timer0/timer2 and adc interrupts
 *				Timer3 counter/buffer used for ATOMIC 16bit reads and writes of touch data
 * PORTA		AN0 touch input, AN2 DAC output
 * PORTB		4LED RGBA bar display
 *
 *
 *
 * This application is designed for the
 * pic18F46K22  device with CTMU module.
 *
 */

#include <p18F46K22.h>
#include <pwm.h>
#include <delays.h>
#include <stdlib.h>
#include <eep.h>
#include <timers.h>
#include <adc.h>
#include <ctmu.h>

void high_handler (void);	//reads the CTMU voltage using a ADC channel, interrupt driven
void low_handler (void);	// led pwm isr

#define RNG_UPDATE	128
#define	RNG_SPEED_F	10
#define	RNG_SPEED_S	9
#define	MAX_UP	254
#define PUF_SIZE 128
#define LEDA	LATBbits.LATB5
#define LEDR	LATBbits.LATB4
#define LEDG	LATBbits.LATB7
#define LEDB	LATBbits.LATB6
#define	LEDBLK	LATB
#define	PDELAY	9999

#define FALSE	0
#define TRUE	1
#define	ON		1
#define ALLON	0xff
#define	OFF		0
#define ALLOFF	0x00

//	CTMU section
unsigned int touch_base_calc(unsigned char);
void touch_channel(unsigned char);
unsigned int ctmu_touch(unsigned char,unsigned char);
int	ctmu_setup(unsigned char,unsigned char);

#define	TIMERCHARGE_BASE_X10	65523		// 5.5 uA time, large plate ~150us
#define	TIMERCHARGE_BASE_1		64000		// .55 uA time, large plate max sens ~700us
#define	TIMERCHARGE_BASE_2		61543		// .55 uA time, large plate low sens ~1000us
#define	TIMERCHARGE_BASE_3		65000		// .55 uA time, small plate max sens ~200us
#define	TIMERCHARGE_BASE_4		62543		// .55 uA time, small plate low sens ~750us
#define	TIMERDISCHARGE			41000		// discharge and max touch data update period 1.8ms

#define TRIP 32  //Difference between pressed
//and un-pressed switch
#define HYST 8 //amount to change
//from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0
#define	CHOP_BITS	0						// remove this many bits to reduce noise from the touch sensor

//	Random number generator section
#pragma udata sddata
far unsigned char		sram_key[PUF_SIZE], sram_key_masked[PUF_SIZE];
#pragma udata

#pragma	idata
struct	colortype	{
volatile	unsigned char 		rval,gval,bval,aval;
};
volatile	struct	colortype	col={15,15,15,15};

struct	btype	{
        unsigned int	seed,sram_raw,eeprom_save,eeprom_mask,eeprom_key;
};
struct	btype	puf_bits={0,0,0,0,0};

unsigned char			vspeed=RNG_SPEED_F,MAXA=255,MAXR=255,MAXG=255,MAXB=255,ctmu_button,PEAK_READS=2;
volatile unsigned char	CTMU_ADC_UPDATED=FALSE,TIME_CHARGE=FALSE,CTMU_WORKING=FALSE;
volatile unsigned int 	touch_base[16],switchState=UNPRESSED,charge_time[16]; 	//storage for reading parameters


/******************************************************************************
*  Make CRC 16 Look-up Table.                                            *
******************************************************************************/

const rom unsigned int far crc_table[0x100] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


#pragma code high_interrupt = 0x8
void high_int (void)
{
        _asm goto high_handler _endasm
}
#pragma code

#pragma code low_interrupt = 0x18
void low_int (void)
{
        _asm goto low_handler _endasm
}
#pragma code

#pragma interrupt low_handler
void low_handler (void)
{
	static unsigned char a=0;
        _asm nop _endasm
        if ( PIR1bits.TMR2IF ) {
        	PIR1bits.TMR2IF = 0;							// clear TMR2 int flag
			WriteTimer2(PDELAY);
				if (!(a++)) {
					LEDBLK=ALLON;
					if (col.aval >MAXA) col.aval=MAXA;
					if (col.rval >MAXR) col.rval=MAXR;
					if (col.gval >MAXG) col.rval=MAXG;
					if (col.bval >MAXB) col.bval=MAXB;
				} else {
        			if (a>=col.aval) LEDA=OFF;
            		if (a>=col.rval) LEDR=OFF;
            		if (a>=col.gval) LEDG=OFF;
            		if (a>=col.bval) LEDB=OFF;
				}
		}
}

#pragma interrupt high_handler
void high_handler (void)
{
	static	union Timers	timer;
	static	unsigned char	channel,i=0;
	static	unsigned int	touch_peak=1024;					// max CTMU voltage

        _asm nop _endasm
        if ( INTCONbits.TMR0IF ) {								// check timer0 irq 
			if (!CTMUCONHbits.IDISSEN) {						// charge cycle timer0 int, because not shorting the CTMU voltage.
				CTMUCONLbits.EDG1STAT = 0; 						// Stop charging touch circuit
				TIME_CHARGE=FALSE;								// clear charging flag
				CTMU_WORKING=TRUE;								// set working flag, doing touch ADC conversion
				// configure ADC for next reading
				channel=(TMR3H>>4)&0x0f;						// ADC channel is TMR3H [4..7] bits
				ADCON0bits.CHS=channel; 						// Select ADC channel, TMR3H[4..7]
				ADCON0bits.ADON=1; 								// Turn on ADC
				ADCON0bits.GO=1; 								// and begin A/D conv, will set adc int flag when done.
			} else {											// discharge cycle timer0 int, because CTMU voltage is shorted 
				CTMUCONHbits.IDISSEN = 0; 						// end drain of touch circuit
				TIME_CHARGE=TRUE;								// set charging flag
				CTMU_WORKING=TRUE;								// set working flag, doing 
       			WriteTimer0 ( charge_time[channel] );			// set timer to charge rate time
				CTMUCONLbits.EDG1STAT = 1; 						// Begin charging the touch circuit
			}
           	// clr  TMR0 int flag
        	INTCONbits.TMR0IF = 0; 								//clear interrupt flag
		}
		if (PIR1bits.ADIF) {									// check ADC irq
			PIR1bits.ADIF = 0; 									// clear ADC int flag
			if (ADRES < touch_peak) touch_peak=ADRES;			// find peak value
			if (i++ >= PEAK_READS) {
				timer.lt = touch_peak; 									// Get the value from the A/D
				timer.lt = (timer.lt >>CHOP_BITS)&0x03ff;				// toss lower bit noise and mask
				if((timer.lt) < (touch_base[channel]-TRIP)) {			// see if we have a pressed button
					switchState = PRESSED;
				} else if((timer.lt) > (touch_base[channel]-TRIP+HYST)) {
					switchState = UNPRESSED;
				}
				TMR3H=timer.bt[1]|((channel<<4)&0xf3);			// copy high byte/channel data [4..7] bits
				TMR3L=timer.bt[0];								// copy low byte and write to timer counter
				i=0;
				touch_peak=1024;
				CTMU_ADC_UPDATED=TRUE;							// New data is in timer3 counter, set to FALSE in main program flow
			}
			CTMU_WORKING=FALSE;									// clear working flag, ok to read timer3 counter.
			// config CTMU for next reading
			CTMUCONHbits.CTMUEN = 1; 							// Enable the CTMU
			CTMUCONLbits.EDG1STAT = 0; 							// Set Edge status bits to zero
			CTMUCONLbits.EDG2STAT = 0;
			CTMUCONHbits.IDISSEN = 1; 							// drain charge on the circuit
       		WriteTimer0 ( TIMERDISCHARGE );						// set timer to discharge rate
		}
}

unsigned int touch_base_calc(unsigned char channel)
{
long	t_avg=0,i;
	touch_channel(channel);
	CTMU_ADC_UPDATED=FALSE;
	while (!CTMU_ADC_UPDATED) ClrWdt();							// wait for touch update cycle
	for (i=0;i<8;i++) {
		CTMU_ADC_UPDATED=FALSE;
		while (!CTMU_ADC_UPDATED) ClrWdt();							// wait for touch update cycle
		t_avg+=(ctmu_touch(channel,FALSE)&0x03ff);
	}
	touch_base[channel]= (unsigned int) (t_avg>>3);
	return touch_base[channel];
}

void	touch_channel(unsigned char channel)
{
	CTMU_ADC_UPDATED=FALSE;
	while (!CTMU_ADC_UPDATED);							// wait for touch update cycle
	TMR3H=channel<<4;									// set channel
	TMR3L=0;											// write to timer3 counter
	CTMU_ADC_UPDATED=FALSE;
	while (!CTMU_ADC_UPDATED);							// wait for touch update cycle
}

int ctmu_setup(unsigned char current, unsigned char channel)
{
		//CTMUCONH/1 - CTMU Control registers
		CTMUCONH = 0x00; //make sure CTMU is disabled
		CTMUCONL = 0x90;
		//CTMU continues to run when emulator is stopped,CTMU continues
		//to run in idle mode,Time Generation mode disabled, Edges are blocked
		//No edge sequence order, Analog current source not grounded, trigger
		//output disabled, Edge2 polarity = positive level, Edge2 source =
		//source 0, Edge1 polarity = positive level, Edge1 source = source 0,
		//CTMUICON - CTMU Current Control Register
		CTMUICON = 0x01; 						//.55uA, Nominal - No Adjustment default

		switch (current) {
		case 11:
			charge_time[channel]=TIMERCHARGE_BASE_1;
			break;
		case 12:
			charge_time[channel]=TIMERCHARGE_BASE_2;
			break;
		case 13:
			charge_time[channel]=TIMERCHARGE_BASE_3;
			break;
		case 14:
			charge_time[channel]=TIMERCHARGE_BASE_4;
			break;
		default:
			charge_time[channel]=TIMERCHARGE_BASE_3;			// slower
			break;
		}
		if (current == 0x02) {
			CTMUICON = 0x02; 	//5.5uA, Nominal - No Adjustment
			charge_time[channel]=TIMERCHARGE_BASE_X10;	// faster
		}
		/**************************************************************************/
		//Set up AD converter;
		/**************************************************************************/

		// Configure AN0-1 as an analog channels
		ANSELAbits.ANSA0=1;
		TRISAbits.TRISA0=1;
		ANSELAbits.ANSA1=1;
		TRISAbits.TRISA1=1;

		// ADCON2
		ADCON2bits.ADFM=1; // Results format 1= Right justified
		ADCON2bits.ACQT=7; // Acquition time 7 = 20TAD 2 = 4TAD 1=2TAD
		ADCON2bits.ADCS=6; // Clock conversion bits 6= FOSC/64 2=FOSC/32
		// ADCON1
		ADCON1bits.PVCFG0 =0; // Vref+ = AVdd
		ADCON1bits.NVCFG1 =0; // Vref- = AVss
		// ADCON0
		ADCON0bits.CHS=0; 	// Select ADC channel
		ADCON0bits.ADON=1; 	// Turn on ADC
		PIE1bits.ADIE=1;	// enable ADC int

		// timer3 register used for atomic data transfer
		T3CONbits.TMR3ON=0;		// Timer is off
		T3CONbits.T3RD16=1;		// enable 16 bit reads/writes
		TMR3H=0;
		TMR3L=0;
	return 0;
}

unsigned int ctmu_touch(unsigned char channel, unsigned char NULL)
{
	static unsigned int	ctmu_change=0, last=0, null=0;
	static	union Timers	timer;
	
	if (CTMU_ADC_UPDATED) {
		timer.bt[0]=TMR3L;									// read low byte and read 16bits from timer counter into TMR3 16bit buffer
		timer.bt[1]=TMR3H;									// read high byte
		if (!NULL) {
			return (timer.lt&0x03ff);
		}
		if (((timer.lt&0x03ff))< (touch_base[channel]&0x03ff) ) {
			ctmu_change=(touch_base[channel]&0x03ff)-(timer.lt&0x03ff);		// read diff 
			if (NULL) {
				if (ctmu_change >255) ctmu_change=1;
			}
		}
		LATD= ~ctmu_change;
		LATC= ~ctmu_change;
		if ((null == 0) && NULL) null=ctmu_change;
		last=ctmu_change;
		return (unsigned int) ctmu_change;
	} else {
		LATD= ~last;
		LATC= ~last;
		return (unsigned int) last;
	}
}

void color_check(void)
{
	long int	delay;

		LEDBLK=ALLOFF;
		for (delay=0;delay<=500000;delay++)	ClrWdt();	// reset the WDT timer
		LEDA=ON;
		for (delay=0;delay<=500000;delay++)	ClrWdt();	// reset the WDT timer
		LEDA=OFF;
		LEDR=ON;
		for (delay=0;delay<=500000;delay++)	ClrWdt();	// reset the WDT timer
		LEDR=OFF;
		LEDG=ON;
		for (delay=0;delay<=500000;delay++)	ClrWdt();	// reset the WDT timer
		LEDG=OFF;
		LEDB=ON;
		for (delay=0;delay<=500000;delay++)	ClrWdt();	// reset the WDT timer
		LEDB=OFF;
}

/*	Count bits 	*/

unsigned char	bit_count(unsigned char input)
{
        unsigned char counter=0,i;

        for	(i=0; i<8; i++) {
                if ( (input & (1<<i)) != 0 ) counter++;
        }
        return (counter);
}


/******************************************************************************
*  Make CRC 16 Calculation Function.                                     *
******************************************************************************/

unsigned int Make_Crc16 (unsigned char *data, unsigned int length)
{
        unsigned int accum,len;

        /* Pre-conditioning */
        accum = 0xFFFF;
        len=length;


        ClrWdt();									// reset the WDT timer
        while (len--) {
                accum = ((accum & 0x00FF) << 8) ^
                        crc_table[( (accum >> 8) ^ *data ++ ) & 0x00FF];
        }

        ClrWdt();									// reset the WDT timer
        return(accum);
}


/* get random data for sram powerup bits */
unsigned int puf_sram(unsigned char cmode, unsigned char kmode)			// look at random SRAM data for PRNG seeds and ID key
{																		// uses a udata section of memeory 2xPUF_SIZE and eeprom from
        unsigned int	e;												// 0 to 3xPUF_SIZE+sizeof(puf_bits) to store key, diff data, masked data and seed data in EEPROM
        static unsigned int	seeda=0, seedk=0;							// if cmode is TRUE the stored key diff data will be zeroed and will return 0
        unsigned char	entr_s,entr_r,entr_d, *seedp;					// if kmode is TRUE the seed will be generated from the masked contents of sram
        																// if kmode is FALSE the seed will be generated from the raw sram contents
        																// kmode should be stable after about 10 or more power cycles

		puf_bits.sram_raw=0;
		puf_bits.eeprom_save=0;
		puf_bits.eeprom_mask=0;
		puf_bits.eeprom_key=0;
        for (e=0; e<PUF_SIZE; e++) {
                seeda = seeda + sram_key[e];
                entr_s=sram_key[e];
				puf_bits.sram_raw+=bit_count(entr_s);
                Busy_eep();
                entr_r = Read_b_eep ( e );					// read eeprom seed data
				puf_bits.eeprom_save+=bit_count(entr_r);
                Busy_eep();
                entr_d = Read_b_eep ( e+PUF_SIZE );			// read unstuck bits data
                entr_d = entr_d | (entr_s^entr_r);			// XOR to look for diff bits
				puf_bits.eeprom_mask+=bit_count(entr_d);
                sram_key_masked[e]=sram_key[e] & (~entr_d);	// store unstuck bits
				puf_bits.eeprom_key+=bit_count(sram_key_masked[e]);

                if (!cmode) {									// cmode  will zero key data history
                        Busy_eep();
                        Write_b_eep ( e, entr_s );				// write eeprom new key data
                        Busy_eep();
                        Write_b_eep ( e+PUF_SIZE, entr_d );		// write eeprom key unstuck bits data
                        Busy_eep();
                        Write_b_eep ( e+PUF_SIZE+PUF_SIZE, sram_key_masked[e] );		// write eeprom key stuck bits data
                } else {
                        Busy_eep();
                        Write_b_eep ( e, entr_s );				// write eeprom key data
                        Busy_eep();
                        Write_b_eep ( e+PUF_SIZE, 0 );			// write zeros to  eeprom key unstuck bits data
                        Busy_eep();
                        Write_b_eep ( e+PUF_SIZE+PUF_SIZE, 0 );	// write zeros to  eeprom key masked data
                }
                ClrWdt();										// reset the WDT timer
        }

        puf_bits.seed=Make_Crc16(sram_key,PUF_SIZE);									// make seed from crc16 of sram
        if (kmode) puf_bits.seed=Make_Crc16(sram_key_masked,PUF_SIZE);					// make seed from crc16 of sram masked with unstuck bits
        if (cmode) puf_bits.seed=0; 													// cmode return 0

		seedp =  (unsigned char*) &puf_bits;											// set seedp pointer to address of data
		for (e=0;e<sizeof(puf_bits); e++) {
        	Busy_eep();
        	Write_b_eep (e+PUF_SIZE+PUF_SIZE+PUF_SIZE, (unsigned char) (*seedp++)  );					// Write seed byte to EEPROM
		}
        return (puf_bits.seed);
}

unsigned char	get_vspeed(void)
{
	if ((unsigned int) rand() >16000) {
		return RNG_SPEED_F;
	} else {
		return RNG_SPEED_S;
	}
}

void main (void)
{
        unsigned int	delayc=0, spinner=0,dc0=256,dc1=256,dc2=256,dc3=256;
        unsigned int	s0=50,s1=50,s2=50,s3=50,sc0=0,sc1=0,sc2=0,sc3=0,touch_tmp;
        unsigned char   RUP=TRUE,GUP=TRUE,BUP=TRUE,AUP=TRUE;
		unsigned int	touch_zero,touch1,touch_avg,i=0,recal=0;
		unsigned long	delay;
		TRISA = 0xff;				//	inputs 
        TRISB = 0x00;				//	outputs
		ANSELB=0;
        TRISC = 0x00;				//	outputs
		TRISD = 0x00;				// led display
		TRISE = 0x00;				// all outputs
		LATC=0xff;
		LATD=0x00;					// all on
        OSCCON = 0x70;				// internal osc
		OSCTUNE = 0xC0;
		SLRCON=0x00;				// set slew rate to max

        OpenTimer0 ( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1 );
        WriteTimer0 ( TIMERDISCHARGE );						//	start timer0 

		OpenPWM5(0x10,CCP_5_SEL_TMR12);
        OpenTimer2(TIMER_INT_ON & T2_PS_1_1 & T2_POST_1_8 );
		IPR1bits.TMR2IP=0;									// set timer2 low pri interrupt
		WriteTimer2(PDELAY);

		TRISAbits.TRISA2=0;			// DAC output

        /* Enable interrupt priority */
        RCONbits.IPEN = 1;
        /* Enable all high priority interrupts */
        INTCONbits.GIEH = 1;


        srand(puf_sram(FALSE,FALSE));					// use unmasked seed for prng

        col.rval=rand();
        col.gval=rand();
        col.bval=rand();
        col.aval=rand();

		LEDA=OFF;
		LEDR=OFF;
		LEDG=OFF;
		LEDB=OFF;
		color_check();
        /* Enable all low priority interrupts */
        INTCONbits.GIEL = 1;

//		CTMU setups
		ctmu_button=0;							// select start touch input
		ctmu_setup(13,ctmu_button);				// config the CTMU for touch response 1X for normal .55ua, 2 for higher 5.5ua char0ge current
		touch_zero=touch_base_calc(ctmu_button);

        /* Loop forever */
        while (TRUE) {											// cycle colors at random speeds

                if ((sc0++) >= s0) {
                        if (AUP) {								// count pwm duty-cycle up
                                col.aval++;
                                if (col.aval >= MAX_UP) AUP=FALSE;	//	at max count reverse count
                        } else {								// count pwm duty-cycle down
                                col.aval--;
                                if (col.aval <=1) AUP=TRUE;		//	at min count reverse count
                        }
                        sc0=0;									// reset change rate counter clock
                        if (dc0++ >RNG_UPDATE) {				// check update rate counter
                                dc0=0;							// reset update rate
                                s0=(unsigned int)((unsigned int)rand() >> get_vspeed());			// set new change rate counter
                        }
                }

                if ((sc1++) >= s1) {
                        if (RUP) {
                                col.rval++;
                                if (col.rval >= MAX_UP) RUP=FALSE;
                        } else {
                                col.rval--;
                                if (col.rval <=1) RUP=TRUE;
                        }
                        sc1=0;
                        if (dc1++ >RNG_UPDATE) {
                                dc1=0;
                                s1=(unsigned int)((unsigned int)rand() >> get_vspeed());
                        }
                }

                if ((sc2++) >= s2) {
                        if (GUP) {
                                col.gval++;
                                if (col.gval >= MAX_UP) GUP=FALSE;
                        } else {
                                col.gval--;
                                if (col.gval <=1) GUP=TRUE;
                        }
                        sc2=0;
                        if (dc2++ >RNG_UPDATE) {
                                dc2=0;
                                s2=(unsigned int) ((unsigned int)rand() >> get_vspeed());
                        }
                }

                if ((sc3++) >= s3) {
                        if (BUP) {
                                col.bval++;
                                if (col.bval >= MAX_UP) BUP=FALSE;
                        } else {
                                col.bval--;
                                if (col.bval <=1) BUP=TRUE;
                        }
                        sc3=0;
                        if (dc3++ >RNG_UPDATE) {
                                dc3=0;
                                s3=(unsigned int) ((unsigned int) rand() >> get_vspeed());
                        }
                }

//				for (delay=0;delay<=255;delay++) {
//					ClrWdt();	// reset the WDT timer
//				}

				CTMU_ADC_UPDATED=FALSE;
				while (!CTMU_ADC_UPDATED) ClrWdt();							// wait for touch update cycle
				touch_tmp=ctmu_touch(ctmu_button,TRUE);
				if (touch_tmp >3 ) {
					MAXA=MAXR=MAXG=MAXB=touch_tmp*4;
				}

				if (FALSE && switchState == PRESSED) {
        			col.rval=rand();
        			col.gval=rand();
        			col.bval=rand();
       				col.aval=rand();
					MAXA=MAXR=MAXG=MAXB=255;
				}

                ClrWdt();								// reset the WDT timer
        }
ClosePWM5();
}
#pragma idata
