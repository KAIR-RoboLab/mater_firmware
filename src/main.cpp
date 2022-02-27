#include <Arduino.h>
#include "maker_pi_pins.h"


void setup() {
  // initialise serial port
  Serial.begin(115200); 
  // initialise LED pin
  pinMode(maker_pi::GROOVE_1_A, OUTPUT);
}


void loop() {
  // turn on LED on pin assigned to GROOVE 1 pin num 0
  digitalWrite(maker_pi::GROOVE_1_A, HIGH);
  // print pin state
  printf("Pin is set to low\n");
  // wait half of a second
  delay(500);
  // turn of the LED
  digitalWrite(maker_pi::GROOVE_1_A, LOW);
  printf("Pin is set to high\n");
  delay(500);
}