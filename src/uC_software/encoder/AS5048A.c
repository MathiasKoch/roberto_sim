
#include <avr/io.h>
#include "AS5048A.h"


/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A_Init(void){

	DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SS)|(1<<DD_SCK));
    // Define the following pins as output
    DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SS)|(1<<DD_SCK));

    
    SPCR = ((1<<SPE)|               // SPI Enable
            (0<<SPIE)|              // SPI Interupt Enable
            (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
            (1<<MSTR)|              // Master/Slave select   
            (0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate
            (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (1<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR = (1<<SPI2X);              // Double Clock Rate
}

/**
 * Closes the SPI connection
 */
void AS5048A_close(void){
	SPCR = (0<<SPE);
}

/**
 * Utility function used to calculate even parity of uint16_t
 */
uint8_t AS5048A_spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++){
		if (value & 0x1)
			cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A_getRotation(void){
	uint16_t data;
	int rotation;

	data = AS5048A_getRawRotation();
	rotation = (int)data - (int)position;
	if(rotation > 8191) 
		rotation = -((0x3FFF)-rotation); //more than -180
	//if(rotation < -0x1FFF) rotation = rotation+0x3FFF;

	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t AS5048A_getRawRotation(void){
	return AS5048A_read(AS5048A_ANGLE);
}

/**
 * returns the value of the state register
 * @return 16 bit uint16_t containing flags
 */
uint16_t AS5048A_getState(void){
	return AS5048A_read(AS5048A_DIAG_AGC);
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain(void){
	uint16_t data = AS5048A_getState();
	return (uint8_t) data & 0xFF;
}

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A_getErrors(void){
	return AS5048A_read(AS5048A_CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(uint16_t arg_position){
	position = arg_position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition(void){
	return position;
}

/*
 * Check if an error has been encountered.
 */
uint8_t AS5048A_error(void){
	return errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit uint16_t
 * Returns the value of the register
 */
uint16_t AS5048A_read(uint16_t registerAddress){

	//Send the command
	PORT_SPI &= ~(1<<DD_SS);
	spi_fast_shift((registerAddress >> 8) & 0xFF);
	spi_fast_shift(registerAddress & 0xFF);
	PORT_SPI |= (1<<DD_SS);

	//Now read the response
	PORT_SPI &= ~(1<<DD_SS);;
	uint8_t left_byte = spi_fast_shift(0xFF);
	uint8_t right_byte = spi_fast_shift(0xFF);
	PORT_SPI |= (1<<DD_SS);


	//Check if the error bit is set
	/*if (left_byte & 0x40) {
		errorFlag = TRUE;
	}else{
		errorFlag = FALSE;
	}*/

	//Return the data, stripping the parity and error bits
	return (((left_byte & 0xFF) << 8 ) | (right_byte & 0xFF)) & ~0xC000;
}


/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A_write(uint16_t registerAddress, uint16_t data) {

	uint16_t command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)AS5048A_spiCalcEvenParity(command)<<15);

	//Split the command into two uint8_ts
	uint8_t right_byte = command & 0xFF;
	uint8_t left_byte = ( command >> 8 ) & 0xFF;

	//Start the write command with the target address
	
	PORT_SPI &= ~(1<<DD_SS);
	spi_fast_shift(left_byte);
	spi_fast_shift(right_byte);
	PORT_SPI |= (1<<DD_SS);

	uint16_t dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((uint16_t)AS5048A_spiCalcEvenParity(dataToSend)<<15);
	left_byte = (dataToSend >> 8) & 0xFF;
	right_byte = dataToSend & 0xFF;

	//Now send the data packet
	PORT_SPI &= ~(1<<DD_SS);
	spi_fast_shift(left_byte);
	spi_fast_shift(right_byte);
	PORT_SPI |= (1<<DD_SS);

	//Send a NOP to get the new data in the register
	PORT_SPI &= ~(1<<DD_SS);
	left_byte =-spi_fast_shift(0xFF);
	right_byte = spi_fast_shift(0xFF);
	PORT_SPI |= (1<<DD_SS);

	//Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}

uint8_t spi_fast_shift (uint8_t data){
    SPDR = data;
    while((SPSR & (1<<SPIF))==0);
    return SPDR;
}
