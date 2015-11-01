#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#include "I2C_slave.h"
#include "uart.h"

#define ADDR 0x4E
#define IODIR 0x00
#define GPIO 0x09

void I2C_init(uint8_t address) {
	// Initial I2C Slave
	TWAR = ADDR & 0xFE; // Set I2C Address, Ignore I2C General Address 0x00
	TWDR = 0x00;                 // Default Initial Value

	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

unsigned char regaddr;    // Store the Requested Register Address
unsigned char regdata;    // Store the Register Address Data

// Simulated OLAT Return Mode: 0-Same as GPIO, 1-Use an alternative LED pattern above
volatile unsigned char olat_mode;

enum SlaveRegisters {
	BAUDRATE=0,
	PORTDSTATE=1
};

/**
 * rw_status
 */
void i2c_slave_action(unsigned char rw_status)
{
	//static unsigned char iled=0;

	// rw_status: 0-Read, 1-Write
	switch (regaddr) {
		case BAUDRATE:
			if (regdata == 0) {
				// Disable USART0
				uart0_disable();
				// PD0 and PD1 into high impedance state
			}
			else
			{
				// Enable USART0
				// Set baudrate to regdata * 600
				uart0_init(regdata * 600);
			}
			break;

		case PORTDSTATE:
			if (rw_status) {
				PORTD=regdata;   // Write to PORTD
			} else {
				regdata=PORTD;   // Read PORTD *output* state
			}
			break;
	}
}

ISR(TWI_vect)
{
	static unsigned char i2c_state;
	unsigned char twi_status;

	// Disable Global Interrupt
	cli();  // TODO: Is this necessary?

	// Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
	twi_status=TWSR & 0xF8;

	switch(twi_status) {
		case TW_SR_SLA_ACK:      // 0x60: SLA+W received, ACK returned
			i2c_state=0;           // Start I2C State for Register Address required

			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			break;

		case TW_SR_DATA_ACK:     // 0x80: data received, ACK returned
			if (i2c_state == 0) {
				regaddr = TWDR;      // Save data to the register address
				i2c_state = 1;
			} else {
				regdata = TWDR;      // Save to the register data
				i2c_state = 2;
			}

			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			break;

		case TW_SR_STOP:         // 0xA0: stop or repeated start condition received while selected
			if (i2c_state == 2) {
				i2c_slave_action(1); // Call Write I2C Action (rw_status = 1)
				i2c_state = 0;	      // Reset I2C State
			}

			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			break;

		case TW_ST_SLA_ACK:      // 0xA8: SLA+R received, ACK returned
		case TW_ST_DATA_ACK:     // 0xB8: data transmitted, ACK received
			if (i2c_state == 1) {
				i2c_slave_action(0); // Call Read I2C Action (rw_status = 0)

				TWDR = regdata;      // Store data in TWDR register
				i2c_state = 0;	      // Reset I2C State
			}

			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			break;

		case TW_ST_DATA_NACK:    // 0xC0: data transmitted, NACK received
		case TW_ST_LAST_DATA:    // 0xC8: last data byte transmitted, ACK received
		case TW_BUS_ERROR:       // 0x00: illegal start or stop condition
		default:
			TWCR |= (1<<TWINT);    // Clear TWINT Flag
			i2c_state = 0;         // Back to the Begining State
	}

	// Enable Global Interrupt
	sei();  // Is this necessary?
}