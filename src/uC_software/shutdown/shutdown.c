#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define INT_SHUTDOWN PB4
#define KILL PB3
#define LED PB2

#define KILL_TIME 30						// Seconds

#define TIME_CALIBRATION 240				// Timer frequency [Hz]

volatile int shutting_down;
volatile int kill_count;
volatile uint8_t pwm;
volatile uint8_t dir;


int enable_timer(void) { 
   	TCNT1 = 0;         						// reset timer 
   	TCCR1 = (1 << CS10) |
   		  	(1 << CS12) ;					// ~ 123 Hz
   		   //	(1 << CS11) ;
   		   //	(1 << CS13); */  			// Prescale CS13-CS10 (slowing down to see output on a led) */
   	OCR1C = 255;
   	GTCCR = (1 << PWM1B);
   	TIFR =  (1 << TOV1); 
   	TIMSK = (1 << TOIE1);  					// enable TOIE1 interrupt 
   	kill_count = 0;
   	shutting_down = 1;
   	return 1; 
}

ISR(PCINT0_vect){				
	if (PINB & (1<<INT_SHUTDOWN)) { 		// Detect rising edge 
		if(!shutting_down)
			enable_timer();
		_delay_ms(20); 						// Simple debounce
		GIFR |= (1 << PCIF); 				// Clear interrupt flag
	}
}

ISR(TIMER1_OVF_vect) {
	kill_count++;
	if(dir)
		pwm++;
	else
		pwm--;
}

int main(int argc, const char* argv[]){
 	
 	DDRB = (1 << KILL);						// Set KILL as output
 	PORTB |= (1 << KILL);					// Set KILL high

 	shutting_down = 0;
 	kill_count = 0;
 	dir = 1;
 	pwm = 0;
 	DDRB |= (1 << LED);						// Set LED as output
	PORTB |= (1 << LED);					// Power on LED

	GIMSK |= (1 << PCIE);					// Enable PCINT
	PCMSK = (1 << INT_SHUTDOWN);			// Enable PCINT4
	sei();									// Enable Global Interrupt

	while(1){
		if(shutting_down){					// Dont think this is needed? but no harm
			if(pwm == 255 || pwm == 0)
				dir = !dir;

			// Software PWM
			if(TCNT1>pwm)
				PORTB |= (1<<LED);			// Turn on LED
			else
				PORTB &= ~(1<<LED);			// Turn off LED

			//if(kill_count > (KILL_TIME*TIME_CALIBRATION))
			//	PORTB &= ~(1 << KILL);		// Kill all power
		}
	}

	return 0;
}
