#include <stdio.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#include "I2C_slave.h"
#include "uart.h"

//#define ADDR 0x4E
#define IODIR 0x00
#define GPIO 0x09

void I2C_init(uint8_t address) {
	// Initial I2C Slave
	TWAR = address << 1; // Ignore I2C General Address 0x00
	TWDR = 0x00;         // Default Initial Value

	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

void I2C_stop(void){
	// clear acknowledge and enable bits
	TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

// Simulated OLAT Return Mode: 0-Same as GPIO, 1-Use an alternative LED pattern above
volatile unsigned char olat_mode;

enum SlaveRegister {
    REGISTER_START = 0,
	REGISTER_BAUDRATE = 0,
	REGISTER_PORTDSTATE,
    REGISTER_END
};

enum SlaveRegister regaddr = 0;    // Store the Requested Register Address

void next_register();

static uint8_t baudcode = 0;

uint8_t i2c_read_action() {
    uint8_t result;
    switch (regaddr) {
        case REGISTER_BAUDRATE:
            printf("baudcode = %d", baudcode);
            result = baudcode;
            break;
        case REGISTER_PORTDSTATE:
            result = PORTD;
            break;
        default:
            result = 0;
            break;
    }
    next_register();
    return result;
}


void i2c_write_action(uint8_t regdata) {
    printf("%d", regdata);
    switch (regaddr) {
        case REGISTER_BAUDRATE:
            baudcode = regdata;
            if (regdata == 0) {
                // Disable USART0
                uart0_disable();
                // PD0 and PD1 into high impedance state
                // DDR PORT
                // 0    0  - input, high impedance
                DDRD &=  ~(1<<PORTD0 | 1<<PORTD1);
                PORTD &= ~(1<<PORTD0 | 1<<PORTD1);
            }
            else {
                // Enable USART0
                // Set baudrate to regdata * 600
                uart0_init(UART_BAUD_SELECT(baudcode * 600UL, F_CPU));
            }
            break;
        case REGISTER_PORTDSTATE:
            PORTD = regdata;
            break;
        default:
            break;
    }
    next_register();
}

void next_register() {
    ++regaddr;
    if (regaddr == REGISTER_END) {
        regaddr = REGISTER_START;
    }
}

enum State {
    STATE_START                      = 0,
    STATE_REGISTER_ADDRESS_JUST_READ = 1,
    STATE_REGISTER_DATA_JUST_READ    = 2
};

ISR(TWI_vect)
{
    static uint8_t regdata = 0;
	static enum State i2c_state = STATE_START;

	// Disable Global Interrupt
	cli();

	// Get TWI Status Register, mask the prescaler bits (TWPS1, TWPS0)
	uint8_t twi_status = TWSR & 0xF8;

	switch(twi_status) {
		case TW_SR_SLA_ACK:               // 0x60: SLA+W received, ACK returned
            uart0_putc('A');
			i2c_state = STATE_START;      //   Start I2C State for Register Address required
			TWCR |= (1<<TWINT);           //   Clear TWINT Flag
			break;

		case TW_SR_DATA_ACK:              // 0x80: data received, ACK returned
			if (i2c_state == STATE_START) {
                uart0_putc('B');
				regaddr = (enum SlaveRegister)TWDR;           //   Save data to the register address
				i2c_state = STATE_REGISTER_ADDRESS_JUST_READ;
			} else {
                uart0_putc('C');
				regdata = TWDR;           //   Save to the register data
				i2c_state = STATE_REGISTER_DATA_JUST_READ;
			}

			TWCR |= (1<<TWINT);           //   Clear TWINT Flag
			break;

		case TW_SR_STOP:                   // 0xA0: stop or repeated start condition received while selected
            uart0_putc('D');
			if (i2c_state == STATE_REGISTER_DATA_JUST_READ) {
                uart0_putc('S');
				i2c_write_action(regdata); //   Write I2C Action
				i2c_state = STATE_START;   //   Reset I2C State
			}
			TWCR |= (1<<TWINT);            //   Clear TWINT Flag
			break;

		case TW_ST_SLA_ACK:               // 0xA8: SLA+R received, ACK returned
		case TW_ST_DATA_ACK:              // 0xB8: data transmitted, ACK received
            uart0_putc('E');
			if (i2c_state == STATE_REGISTER_ADDRESS_JUST_READ) {
				TWDR = i2c_read_action(); //   Call Read I2C Action (rw_status = 0)
				i2c_state = STATE_START;  //   Reset I2C State
			}
			TWCR |= (1<<TWINT);           //   Clear TWINT Flag
			break;

		case TW_ST_DATA_NACK:             // 0xC0: data transmitted, NACK received
		case TW_ST_LAST_DATA:             // 0xC8: last data byte transmitted, ACK received
		case TW_BUS_ERROR:                // 0x00: illegal start or stop condition
		default:
            uart0_putc('F');
			TWCR |= (1<<TWINT);           //   Clear TWINT Flag
			i2c_state = STATE_START;      //   Back to the Begining State
	}

	// Enable Global Interrupt
	sei();  // Is this necessary?
}