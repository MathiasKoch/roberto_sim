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

typedef struct{
	uint16_t raw_angle;
	uint16_t prev_angle;
	int16_t speed;
	uint16_t rotations;
} AS5048;



unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);
void compute(AS5048* enc);

volatile bool update = false;




ISR(TIMER0_COMPA_vect) {
    update = true;
}

int main(int argc, const char* argv[]){
	unsigned char messageBuf[TWI_BUFFER_SIZE];


	// Setup address pins as input
	DDRD &= ~((1 << 2) | (1 << 3));		// PD2 & PD3

  	unsigned char TWI_slaveAddress = (0x10 | (PIND & 0x04) | (PIND & 0x08));


	// Initialise TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise((unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_GEN_BIT))); 

	AS5048A_Init();

	AS5048 enc;
	enc.raw_angle = 0;
	enc.prev_angle = 0;
	enc.rotations = 0;

	

    // start timer0 at 1 kHz
	TCCR0A |= (1 << WGM01);
    OCR0A = 0xFA;		
    TIMSK0=(1<<OCIE0A);
    TCCR0B |= (1<<CS01) | (1<<CS00);
	               
	sei();
	


	// Start the TWI transceiver to enable reseption of the first command from the TWI Master.
	messageBuf[0] = 0x00;
	messageBuf[1] = 0x00;
	//messageBuf[2] = 0x00;
//	messageBuf[3] = 0x00;
//	messageBuf[4] = 0x00;
	TWI_Start_Transceiver_With_Data(messageBuf, 2);

	

	while(1){
		if(update){
			update = false;
			enc.raw_angle = AS5048A_getRawRotation();
			compute(&enc);
		}


		// Check if the TWI Transceiver has completed an operation.
		if(!TWI_Transceiver_Busy()){
			// Check if the last operation was successful
			if(TWI_statusReg.lastTransOK){
				// Check if the TWI Transceiver has already been started.
				// If not then restart it to prepare it for new receptions.             
				if(!TWI_Transceiver_Busy()){
					//messageBuf[0] = enc.rotations;
					//messageBuf[0] = (enc.raw_angle  >> 8) & 0xFF;
					//messageBuf[1] = enc.raw_angle  & 0xFF;
					messageBuf[0] = (enc.speed  >> 8) & 0xFF;
					messageBuf[1] = enc.speed  & 0xFF;
					TWI_Start_Transceiver_With_Data(messageBuf, 2);
				}
			}else{ // Ends up here if the last operation completed unsuccessfully
				TWI_Act_On_Failure_In_Last_Transmission(TWI_Get_State_Info());
			}
		}

	}

	return 0;
}

void compute(AS5048* enc){ 
	// Shift to 16 bit to match overflow from 14 bit encoder
	// Calculate speed and return to 14 bits again
	enc->speed = ((enc->raw_angle<<2) - (enc->prev_angle<<2)) >>2;

	// Update lagged position
	enc->prev_angle = enc->raw_angle;

	/*
	if(enc->raw_angle - enc->prev_angle >= 8192){
		enc->speed = (enc->prev_angle - enc->raw_angle);
	}
	if(enc->raw_angle - enc->prev_angle >= 8192){
		enc->speed = -(enc->prev_angle - enc->raw_angle);
	}
	else{
		//enc->speed = enc->raw_angle - enc->prev_angle;
	}
	*/

	/*enc->speed = (enc->raw_angle - enc->prev_angle);
	if((enc->raw_angle & 0x3000) == 0 && (enc->prev_angle & 0x3000) == 0x3000){
		enc->rotations++;
		enc->speed = (enc->raw_angle+0x2000 - enc->prev_angle-0x2000);
	}else if((enc->raw_angle & 0x3000) == 0x3000 && (enc->prev_angle & 0x3000) == 0){
		enc->rotations--;
		enc->speed = (enc->raw_angle-0x2000 - enc->prev_angle+0x2000);
	}*/
}


unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg){
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.

	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
  	//PORTB = TWIerrorMsg;
  	TWI_Start_Transceiver();
                    
  	return TWIerrorMsg; 
}