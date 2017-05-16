//
// 4x4 keypad handler with partial 2-key rollover, autorepeat and I2C interface
//
// Target: MSP430G2553
//
// v1.0 / 2017-05-14 / Io Engineering / Terje
//

/*

Copyright (c) 2017, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <msp430.h> 

#include <stdint.h>
#include <stdbool.h>

#define NUMKEYS 20
#define I2CADDRESS 0x49

#define SCL BIT6						// P1.6 I2C
#define SDA BIT7						// P1.7 I2C

#define BUTDRIVE (BIT0|BIT1|BIT2|BIT3) 	// P1.0-3
#define BUTINPUT (BIT0|BIT1|BIT2|BIT3)	// P2.0-3
#define	BUTINTR BIT4					// P2.4

//#define LASERMAPPING

unsigned char keycode;
unsigned char i2cData[3];
unsigned volatile int i2cRXCount;
unsigned volatile char *pi2cData;
volatile bool keyclick = false;

// Valid scan codes

const uint8_t scanCode[NUMKEYS + 1] = {
  0x11,
  0x21,
  0x41,
  0x81,

  0x12,
  0x22,
  0x42,
  0x82,

  0x14,
  0x24,
  0x44,
  0x84,

  0x18,
  0x28,
  0x48,
  0x88,

// "two button rollover" scan codes
  0xC3,
  0xC6,
  0x63,
  0x66,
  0xFF
};

// Scan code to key code mappings

#ifdef LASERMAPPING

bool getpower = false;
unsigned int laserpower = 10;

const uint8_t keyCode[NUMKEYS] = {
  'U',  // Coolant (toggle)
  'C',	// Down
  'B',	// Left
  'A',	// Back

  'D',  // Exhaust fan (toggle)
  'R',	// Forward
  'H',  // Home
  'L',	// Power +

  'E',	// Air assist (toggle)
  '+' , // Up
  'F',	// Right
  '-',	// Power -

  '\r',
  '-',
  '+',
  '0',

  's',  // SE 4 + 8
  'r',  // SW 4 + 2
  'q',  // NW 6 + 8
  't'   // NE 6 + 2
};

#else

const uint8_t keyCode[NUMKEYS] = {
  0x0B,
  '9',
  '8',
  '7',

  0x1B,
  '6',
  '5',
  '4',

  0x18,
  '3',
  '2',
  '1',

  '\r',
  '-',
  '+',
  '0',

  's',  // 4 + 8
  't',  // 4 + 2
  'q',  // 6 + 8
  'r'   // 6 + 2
};

#endif

// System Routines

unsigned char KeyScan (void);

void initI2C (void)
{
	P1SEL  |= SCL|SDA;				// Assign I2C pins to
	P1SEL2 |= SCL|SDA;				// USCI_B0 (P1)

	IE2 &= ~UCB0TXIE;  				// Disable TX interrupt
	UCB0CTL1 |= UCSWRST;			// Enable SW reset
	UCB0CTL0 = UCMODE_3|UCSYNC;		// I2C Slave, synchronous mode
	UCB0I2COA = I2CADDRESS;			// Own Address is 048h
	UCB0CTL1 &= ~UCSWRST;			// Clear SW reset, resume operation
	UCB0I2CIE |= UCSTPIE| UCSTTIE;	// Enable STT, STP and
	IE2 |= UCB0RXIE|UCB0TXIE;		// RX/TX interrupts
}

void sleep (unsigned int time)
{
	TACTL = TACLR;            // SMCLK/8, Clear TA
	TACTL = TASSEL1|ID0|ID1;  // SMCLK/8, Clear TA
	TACCTL0 = CCIE;           // Enable CCR0 interrupt
	TACCR0 = time;            // Sleep duration
	TACTL |= MC0;             // Start TA in up mode
	LPM0;                     // Sleep...

	TACTL = TACLR;            // Stop and clear TA
	TACCTL0 = 0;              // Clear CCTL0 register
}

void main(void)
{
	unsigned int autorepeat;
	unsigned char lastkey = 0;

	DCOCTL = 0;
	WDTCTL = WDTPW+WDTHOLD;		// Stop watchdog timer
	DCOCTL = CALDCO_16MHZ;      // Set DCO for 16MHz using
	BCSCTL1 = CALBC1_16MHZ;     // calibration registers
	BCSCTL2 = DIVS0|DIVS1;		// SMCLK = MCLK / 8

	_EINT();					// Enable interrupts

	sleep(50);

	initI2C();

	P2DIR |= BUTINTR;						// Enable P2 button pressed out (keydown signal)
	P2DIR &= ~BUTINPUT;						// P2.x button inputs
	P2OUT &= ~BUTINPUT;						// Enable button input pull downs
	P2REN |= BUTINPUT;						// ...
	P2IES &= ~BUTINPUT;						// and set L-to-H interrupt

	P2IFG = 0;								// Clear any pending flags
	P2IE |= BUTINPUT;						// Enable interrupts

	P1DIR |= BUTDRIVE;						// Enable P1 outputs
	P1OUT |= BUTDRIVE;						// for keypad

	P3DIR = 0xFF;							// Set P3 pins to outputs
	P3OUT = 0xFF;							// and set high

	while(1) {

		keycode = '\0';
		lastkey = '\0';
		keyclick = false;
		LPM0; 								// Sleep until key pressed

		if((lastkey = KeyScan()) != '\0') 	// Key still pressed?
			sleep(10000); 					// Yes, debounce

		if(lastkey != '\0' && lastkey == KeyScan()) { // if same key pressed after debounce continue, else go back to sleep

			keycode = lastkey;

#ifdef LASERMAPPING
			autorepeat = 0;

			switch(keycode) {

				case 'A':
				case 'E':
				case 'H':
				case 'C':
					break;

				default:
					autorepeat = 1;
					break;

			}

			if(!(keycode == '-' || keycode == '+' || keycode == 'H'))
				P2OUT |= BUTINTR;
#else
			autorepeat = 1;
#endif

			do {

				if(lastkey != keycode && P2OUT & BUTINTR) {
					P2OUT &= ~BUTINTR;
					sleep(200);
					P2OUT |= BUTINTR;
				}

				keycode = lastkey;

#ifdef LASERMAPPING

				switch(keycode) {

					case '-':
						if(laserpower > 0) {
							laserpower--;
							autorepeat++;
							sleep(autorepeat < 3 ? 64000 : 10000);
						}
						break;

					case '+':
						if(laserpower < 100) {
							laserpower++;
							autorepeat++;
							sleep(autorepeat < 3 ? 64000 : 10000);
						}
						break;

					case 'H':
						autorepeat++;
						sleep(64000);
						break;

				}
#endif

				if(!autorepeat) {
					sleep(50); // 200us
					P2OUT &= ~BUTINTR;
				}

				sleep(5000); // Wait for ~20ms before transmitting again

			} while(((lastkey = KeyScan()) != '\0'));     // Keep transmitting while button held down

#ifdef LASERMAPPING
			if(keycode == '-' || keycode == '+') {
				keycode = 'P';
				P2OUT |= BUTINTR;
				sleep(50); // 200us
			} else if(keycode == 'H') {
				if(autorepeat < 5)
					keycode = 'h';
				P2OUT |= BUTINTR;
				sleep(50); // 200us
			}
#endif
			P2OUT &= ~BUTINTR;

		}
	}
}

unsigned char KeyScan (void) {

	unsigned char scancode = 0, index = BIT0;	// Initialize row mask

	if(!(P2IN & BUTINPUT))					// Keys still pressed?
		return false;						// no, exit

	while(index & BUTDRIVE) {				// Loop through all rows

		P1OUT &= ~BUTDRIVE;					// Stop driving rows

		P2DIR |= BUTINPUT;					// Temporarily set column pins to output
		P2OUT &= ~BUTINPUT;					// and switch low to bleed off charge
		P2DIR &= ~BUTINPUT;					// in order to avoid erroneous reads
		P2OUT &= ~BUTINPUT;					// Enable pull down resistors
		P2REN |= BUTINPUT;                  // on column inputs

		P1OUT |= index;						// Drive row

		if(P2IN & BUTINPUT) {				// If any key pressed:
			scancode |= index << 4;			// set bit for row scanned
			scancode |= (P2IN & BUTINPUT);	// set bit(s) for column(s)
		}

		index = index << 1;                 // Next row

	}

	P1OUT |= BUTDRIVE;						// Drive all rows high again

	index = NUMKEYS;

	if(scancode != 0)                      				// If key(s) were pressed
		while(index && scanCode[--index] != scancode);	// then try to resolve to legal entry in lookup table

	return scanCode[index] == scancode ? keyCode[index] : '\0';
}

// P2.x Interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
	P2IFG &= ~BUTINPUT;						// Clear button interrupt flag(s)

	if((P2IN & BUTINPUT) && !keyclick) {	// Button interrupt while sleeping?
		keyclick = true;					// Yep - set event semaphore and
		LPM0_EXIT;                          // exit LPM0 on return
	}
}

// CCR0 Interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void)
{
	TACTL &= ~(MC0|MC1|TAIFG);	// Stop timer and clear interrupt
	LPM0_EXIT;					// Exit LPM0 on return
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{

#ifdef LASERMAPPING
	if(IFG2 & UCB0TXIFG) {
		UCB0TXBUF = getpower ? laserpower : keycode;      // Transmit current keycode
		getpower = true;
		if(keycode == 'P')
			keycode = 0;
	}
#else
	if(IFG2 & UCB0TXIFG)
		UCB0TXBUF = keycode;     // Transmit current keycode
#endif

	if(IFG2 & UCB0RXIFG) {		 // Save any received bytes - not used in this version

		if(i2cRXCount < 3) {
			*pi2cData++ = UCB0RXBUF;
			i2cRXCount++;
		}

	}
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
	uint8_t intstate = UCB0STAT;

	if(intstate & UCGC) {		// General call?
		UCB0STAT &= ~UCGC;		// Clear interrupt flag
		return;					// and return
	}

	if(intstate & UCSTTIFG) {
#ifdef LASERMAPPING
		getpower = false;
#endif
		i2cRXCount = 0;
		pi2cData = i2cData;
	}

#ifdef LASERMAPPING

	if((intstate & UCSTPIFG) && (i2cRXCount > 1)) {

		switch(i2cData[0]) {

			case 2:	// Set power level
				laserpower = i2cData[1];
				break;

		}

		i2cRXCount = 0;
	}
#endif

	UCB0STAT &= ~intstate;		// Clear interrupt flags
}
