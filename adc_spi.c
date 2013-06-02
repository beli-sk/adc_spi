/*
 * adc_spi.c
 * 
 * This file is part of ADC SPI
 *
 * Copyright 2013, Michal Belica <devel@beli.sk>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define CTRL_PORT	DDRB
#define DATA_PORT	PORTB
#define PIN_PORT	PINB
#define SS_PIN		PB0
#define SS_PCINT	PCINT0
#define CLK_PIN		PB2
#define DO_PIN		PB1 // MISO

volatile unsigned char dataregH = 0xff;
volatile unsigned char dataregL = 0xff;
volatile unsigned char dataregL_hold = 0;
volatile unsigned char sendcount = 0;

#define ADCNR 1 // ADC noise reduction sleep mode
#define PWR_DOWN 2
#define IDLE 3
volatile unsigned char powersave = PWR_DOWN;

/*
 * Enable or disable communication
 */
void switch_comm(const unsigned char state)
{
	if( state ) {
		// configure DO pin
		CTRL_PORT |= _BV(DO_PIN); // OUTPUT
		// enable USI three wire mode
		USICR |= _BV(USIWM0);
	} else {
		// disable USI
		USICR &= ~_BV(USIWM0);
		// release DO pin
		CTRL_PORT &= _BV(DO_PIN); // INPUT
		DATA_PORT &= ~_BV(DO_PIN); // no pull up
	}
}

/*
 * Initialize USI for SPI slave
 */
void init_USI()
{
	CTRL_PORT |= _BV(DO_PIN); // INPUT (will be switched when enabling comm)
	CTRL_PORT &= ~_BV(CLK_PIN); // INPUT

	// Enable USI overflow interrupt, set clock to external
	// negative edge, but do not enable three wire mode yet.
	USICR = _BV(USIOIE) | _BV(USICS1) | _BV(USICS0);

	//Clear overflow flag
	USISR = _BV(USIOIF);
	
	USIDR = 0xff; // byte to send if value is not ready

	// setup pin change interrupt for SS
	GIMSK |= _BV(PCIE);
	PCMSK |= _BV(SS_PCINT);
	CTRL_PORT &= ~_BV(SS_PIN); //INPUT
}

void init_ADC()
{
	// set pin as input
	DDRB &= ~_BV(PB3);
	// disable digital input
	DIDR0 |= _BV(ADC3D);
	// Select voltage reference to internal 2.56V
	ADMUX |= _BV(REFS2) | _BV(REFS1);
	// Enable ADC3 (PB3) as input for ADC
	ADMUX |= _BV(MUX1) | _BV(MUX0);
	// clock prescaler division factor 64 (= 125kHz from 8MHz clock)
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	// ADC interrupt enable
	ADCSRA |= _BV(ADIE);
}

/*
 * ADC conversion complete interrupt
 */
ISR(ADC_vect)
{
	dataregL = ADCL;
	dataregH = ADCH;
	// ADC disable
	ADCSRA &= ~_BV(ADEN);
	// prepare first byte for sending
	USIDR = dataregH;
	sendcount = 1;
	// and hold second byte
	dataregL_hold = dataregL;
	// enable communication
	switch_comm(1);
	// set idle power while waiting for communication
	powersave = IDLE;
}

/*
 * USI overflow interrupt
 */
ISR(USI_OVF_vect)
{
	USISR = _BV(USIOIF);
	if( sendcount == 0x01 ) {
		USIDR = dataregL_hold;
		sendcount++;
	} else {
		USIDR = 0xff;
	}
}

/*
 * Pin change interrupt for SS channel
 */
ISR(PCINT0_vect) {
	if( !(PIN_PORT & _BV(SS_PIN)) ) { // SS is low
		// reset send counter
		sendcount = 0;
		// mark value registers as incomplete
		dataregL = 0xff;
		dataregH = 0xff;
		// ADC enable and start conversion
		ADCSRA |= _BV(ADEN) | _BV(ADSC);
		// start ADC by entering IDLE sleep mode
		powersave = ADCNR;
		// communication will be enabled on ADC complete
	} else { // SS is high
		powersave = PWR_DOWN;
		// disable communication
		switch_comm(0);
	}
}

int main(void)
{	
	init_USI();
	init_ADC();

	// disable timers 0 and 1 to save power
	PRR |= _BV(PRTIM1) | _BV(PRTIM0);

	// enable interrupts	
	sei();

    while(1) {
		// main loop enters sleep modes upon request from interrupt handlers
		if( powersave == ADCNR ) {
			set_sleep_mode(SLEEP_MODE_ADC);
			sleep_mode();
		} else if( powersave == PWR_DOWN ) {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_mode();
		} else if( powersave == IDLE ) {
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
    }
	return(1);
}
