#ifndef MAKER_PI_TRANSPORT__H
#define MAKER_PI_TRANSPORT__H

#include <Arduino.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/time.h>

int clock_gettime(clockid_t unused, struct timespec *tp);
bool maker_pi_transport_open(struct uxrCustomTransport * transport);
bool maker_pi_transport_close(struct uxrCustomTransport * transport);
size_t maker_pi_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t maker_pi_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

#endif // MAKER_PI_TRANSPORT__H