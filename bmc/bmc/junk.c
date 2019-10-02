/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

int k;

void main()
{
	osccon = 0x70; // set osc
	ansel = 0; //  set analog
	intcon.inte = 1; // enable RB0/INT

	trisb.f0 = 1; //set RB0 as input
	trisb.f3 = 0; //set RB3 as output

	for (;;) {
start:
		while (portb.f0 == 1) { //  led blink till RB0=0
			portb.f3 = 0;
			delay_ms(400);
			portb.f3 = 1;
			delay_ms(400);
		}
		if (portb.f0 == 0) {
			k = 1;
			delay_ms(3000);
			if (portb.f0 == 0 && k == 1) { //check RB0=0 for more than 3secs and then sleep
				intcon.gie = 0; //clear GIE before sleep
				portb.f3 = 0;
				asm sleep;
			} else {
				k = 0;
				goto start;
			}
		}
	}
}