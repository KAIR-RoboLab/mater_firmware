#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico_uart_transports.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    uart_init(uart0, 115200);
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    uart_deinit(uart0);
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * errcode)
{
    for (size_t i = 0; i < len; i++)
    {
        uart_putc(uart0, buf[i]);
    }
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++)
    {
        int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0)
        {
            *errcode = 1;
            return i;
        }

        int character = uart_getc(uart0);
        if (character == PICO_ERROR_TIMEOUT)
        {
            *errcode = 1;
            return i;
        }
        buf[i] = character;
    }
    return len;
}