#ifndef CD4051_H
#define CD4051_H

#include <Arduino.h>
#define NUMBER_OF_4051_DATA_PINS 3
#define NUMBER_OF_4051_ANALOG_INPUTS 8

typedef struct CD4051Data {
  int a_pin;
  int d_pin[NUMBER_OF_4051_DATA_PINS];
} CD4051Data;

void cd4051Setup(CD4051Data* c, int a, int d1, int d2, int d3) {
  c->a_pin = a;
  c->d_pin[0] = d1;
  c->d_pin[1] = d2;
  c->d_pin[2] = d3;

  pinMode(c->d_pin[0], OUTPUT);
  pinMode(c->d_pin[1], OUTPUT);
  pinMode(c->d_pin[2], OUTPUT);
}

uint32_t cd4051Read(CD4051Data* c, int index) {
  for(int i = 0; i < NUMBER_OF_4051_DATA_PINS; i++) {
    digitalWrite(c->d_pin[i], (index >> i) & 1);
  }
  delayMicroseconds(2);
  return analogRead(c->a_pin);
}

#endif