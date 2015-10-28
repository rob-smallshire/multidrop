An ATMega1284 client for a multidrop serial bus.

Each client is connected to a master by both I2C and USART0 serial.
Information passed over the I2C bus is used to select a single
slave unit for USART communication with the master and disabling
USART communication on all other slaves, such that communication
over USART can only happen between the master and one slave at a
time.

