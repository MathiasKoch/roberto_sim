#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include "TWI_slave.h"
#include "AS5048A.h"

#define DIR_FORWARD 0
#define DIR_REVERSE 1

#define COMP_POS 0x00
#define COMP_NEG 0x01
#define COMP_NONE 0x02

#define TICKS_PR_ROT 16384
#define TIMERFREQ 1000

typedef struct{
	uint16_t raw_angle;
	uint16_t prev_angle;
	int16_t  dTicks;
	int16_t  dt; //ms
	uint16_t rotations;
} AS5048;



unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);

volatile bool update = false;

//uint8_t retryCnt = 0;
uint8_t loopCounter = 0;
//uint16_t busyCounter = 0;

unsigned char messageBuf[TWI_BUFFER_SIZE];

ISR(TIMER0_COMPA_vect) {
    update = true;
}

void togglePin(){
	PORTD ^= (1 << 6);
}

int main(int argc, const char* argv[]){


	// Setup address pins as input
	DDRD &= ~((1 << 2) | (1 << 3));		// PD2 & PD3

	// PD6 as output for togglePin()
	DDRD |= (1 << 6);		// PD6

  	unsigned char TWI_slaveAddress = (0x10 | (PIND & 0x04) | (PIND & 0x08));


	// Initialise TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise((unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_GEN_BIT))); 

	AS5048A_Init();

	AS5048 enc;
	enc.raw_angle = 0;
	enc.prev_angle = 0;
	enc.rotations = 0;
	enc.dt = 0;
	enc.dTicks = 0;
	
    // start timer0 at 10 kHz
    //16MHz/(8*(OCR0A+1))
	TCCR0A |= (1 << WGM01);
    OCR0A = 199;		
    TIMSK0=(1<<OCIE0A);
    TCCR0B |= (1<<CS01);// prescale 8 // | (1<<CS00); 
	               
	sei();
	
	// Start the TWI transceiver to enable reseption of the first command from the TWI Master.
	messageBuf[0] = 0x00;
	messageBuf[1] = 0x00;
	messageBuf[2] = 0x00;
	messageBuf[3] = 0x00;
	//messageBuf[4] = 0x00;
	TWI_Start_Transceiver_With_Data(messageBuf, TWI_BUFFER_SIZE);

	while(1){
		if(update){
			if (loopCounter++ >= 5){
				update = false;
				enc.raw_angle = AS5048A_getRawRotation();
				
				// Shift to 16 bit to match overflow from 14 bit encoder
				// Calculate speed and return to 14 bits again
				enc.dTicks += ((int16_t)((enc.raw_angle<<2) - (enc.prev_angle<<2)))>>2; //dTicks
				// Update lagged position
				enc.prev_angle = enc.raw_angle;
				enc.dt += 1;
				loopCounter = 0;
			}

			// Check if the TWI Transceiver has completed an operation.
			if(!TWI_Transceiver_Busy()){

				// Check if the last operation was successful
				if(TWI_statusReg.lastTransOK){
					// Check if the TWI Transceiver has already been started.
					// If not then restart it to prepare it for new receptions.             
					if(!TWI_Transceiver_Busy()){
						togglePin();
						int32_t speed = enc.dTicks/enc.dt;///enc.dt*10000;

						messageBuf[0] = (speed  >> 24) & 0xFF;
						messageBuf[1] = (speed  >> 16) & 0xFF;
						messageBuf[2] = (speed  >> 8) & 0xFF;
						messageBuf[3] =  speed  & 0xFF;
						/*messageBuf[4] =  0;
						messageBuf[5] =  (busyCounter  >> 8) & 0xFF;
						messageBuf[6] =  busyCounter  & 0xFF;*/

						TWI_Start_Transceiver_With_Data(messageBuf, TWI_BUFFER_SIZE);
						
						//if(TWI_statusReg.lastTransOK){
							// Reset counters
							enc.dTicks = 0;
							enc.dt = 0;
							//retryCnt = 0;
							//busyCounter = 0;
						//}
					}
				}else{ // Ends up here if the last operation completed unsuccessfully
					TWI_Act_On_Failure_In_Last_Transmission(TWI_Get_State_Info());
				}
			}/*else{
				busyCounter++;
			}*/
		}
		/*if(busyCounter >= 5000){
				

			TWI_Slave_Initialise((unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_GEN_BIT)));
			busyCounter = 0;
		}*/
	}
	return 0;
}

unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg){
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.

	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
  	//PORTB = TWIerrorMsg;
	//messageBuf[4] = ++retryCnt;
  	TWI_Start_Transceiver_With_Data(messageBuf, TWI_BUFFER_SIZE);
                    
  	return TWIerrorMsg; 
}