#ifndef as5048_h
#define as5048_h
#define LIBRARY_VERSION 1.0.0


#include <avr/io.h>

#define AS5048A_CLEAR_ERROR_FLAG 0x0001
#define AS5048A_PROGRAMMING_CONTROL 0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH 0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW 0x0017
#define AS5048A_DIAG_AGC 0x3FFD
#define AS5048A_MAGNITUDE 0x3FFE
#define AS5048A_ANGLE 0x3FFF


#define FALSE 0
#define TRUE 1


// Pin defines

#define PORT_SPI    PORTB
#define DDR_SPI     DDRB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5


uint8_t errorFlag;

uint16_t position;


uint8_t spi_fast_shift (uint8_t data);

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A_Init(void);

/**
 * Closes the SPI connection
 */
void AS5048A_close(void);

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit uint16_t
 * Returns the value of the register
 */
uint16_t AS5048A_read(uint16_t registerAddress);

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A_write(uint16_t registerAddress, uint16_t data);

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5048A_getRotation(void);

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t AS5048A_getRawRotation(void);


/**
 * returns the value of the state register
 * @return 16 bit uint16_t containing flags
 */
uint16_t AS5048A_getState(void);


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain(void);

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A_getErrors(void);

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(uint16_t arg_position);

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition(void);

/*
 * Check if an error has been encountered.
 */
uint8_t AS5048A_error(void);


uint8_t AS5048A_spiCalcEvenParity(uint16_t value);

#endif