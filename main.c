#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
//#include <util/atomic.h>

#include "uart.h"

#ifdef __GNUC__
#  define UNUSED(x) UNUSED_ ## x __attribute__((__unused__))
#else
#  define UNUSED(x) UNUSED_ ## x
#endif

#ifdef __GNUC__
#  define UNUSED_FUNCTION(x) __attribute__((__unused__)) UNUSED_ ## x
#else
#  define UNUSED_FUNCTION(x) UNUSED_ ## x
#endif

int uart0_send_byte(char data, FILE* UNUSED(stream))
{
    if (data == '\n')
    {
        uart0_putc('\r');
    }
    uart0_putc((uint8_t)data);
    return 0;
}

int uart0_receive_byte(FILE* UNUSED(stream))
{
    uint8_t data = uart0_getc();
    return data;
}

static FILE uart0_stream = FDEV_SETUP_STREAM(
        uart0_send_byte,
        uart0_receive_byte,
        _FDEV_SETUP_RW);



enum {
 BLINK_DELAY_MS = 1000,
};


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main (void)
{
    sei();

    stdin = stdout = &uart0_stream;

    // USB Serial 0
    uart0_init(UART_BAUD_SELECT(9600, F_CPU));

    /* set pin 6 of PORTD for output*/
    DDRD |= _BV(DDD6);

    while (1) {
        /* set pin 20 high to turn led on */
        PORTD |= _BV(PORTD6);
        _delay_ms(BLINK_DELAY_MS);

        /* set pin 20 low to turn led off */
        PORTD &= ~_BV(PORTD6);
        _delay_ms(BLINK_DELAY_MS);

        printf("Hello, Multidrop!\n");
    }
}
#pragma clang diagnostic pop
