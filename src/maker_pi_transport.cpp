#include "maker_pi_transport.h"

#include <Arduino.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#define micro_rollover_useconds 4294967295

int clock_gettime(clockid_t unused, struct timespec *tp)
{
  (void)unused;
  static uint32_t rollover = 0;
  static uint32_t last_measure = 0;

  uint32_t m = micros();
  rollover += (m < last_measure) ? 1 : 0;

  uint64_t real_us = (uint64_t) (m + rollover * micro_rollover_useconds);
  tp->tv_sec = real_us / 1000000;
  tp->tv_nsec = (real_us % 1000000) * 1000;
  last_measure = m;

  return 0;
}

bool maker_pi_transport_open(struct uxrCustomTransport * transport)
{
  Serial.begin(115200);
  return true;
}

bool maker_pi_transport_close(struct uxrCustomTransport * transport)
{
  Serial.end();
  return true;
}

size_t maker_pi_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
  (void)errcode;
  size_t sent = Serial.write(buf, len);
  return sent;
}

size_t maker_pi_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  (void)errcode;
  Serial.setTimeout(timeout);
  return Serial.readBytes((char *)buf, len);
}