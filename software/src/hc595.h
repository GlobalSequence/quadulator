#ifndef HC595_H
#define HC595_H

#include <Arduino.h>

typedef struct HC595Data {
  int num_595;
  int num_595_outputs;
  // Define pin connections to 74HC595
  int dataPin;   // SER: Data input pin (pin 14 on 74HC595)
  int clockPin;  // SRCLK: Shift register clock pin (pin 11 on 74HC595)
  int latchPin; // RCLK: Latch clock pin (pin 12 on 74HC595)
} HC595Data;

void hc595Setup(HC595Data* h, int data, int clock, int latch, int num) {
  h->dataPin = data;
  h->clockPin = clock;
  h->latchPin = latch;
  h->num_595 = num;
  h->num_595_outputs = h->num_595 * 8;

  // Initialize pins as outputs
  pinMode(h->dataPin, OUTPUT);
  pinMode(h->clockPin, OUTPUT);
  pinMode(h->latchPin, OUTPUT);

  // Clear the shift register
  digitalWrite(h->latchPin, LOW);
  digitalWrite(h->clockPin, LOW);
  digitalWrite(h->dataPin, LOW);
}

// Function to send an 8-bit value to the shift register
void hc595ShiftOut(HC595Data* h, uint32_t value) {
  // Start by clearing the latch pin (low) to prevent output updates
  digitalWrite(h->latchPin, LOW);
  
  // Loop through the bits of the byte, starting from the MSB
  for (int i = h->num_595_outputs - 1; i >= 0; i--) {
    // Write the data bit
    bool bitValue = (value >> i) & 0x01; // Extract the bit
    digitalWrite(h->dataPin, bitValue);     // Set data pin high/low based on bit value

    // Toggle the clock pin to shift the bit into the register
    digitalWrite(h->clockPin, HIGH);  // Rising edge of clock
    delayMicroseconds(1);          // Short delay for stability
    digitalWrite(h->clockPin, LOW);   // Falling edge of clock
  }

  // Set the latch pin high to update the shift register output
  digitalWrite(h->latchPin, HIGH);
}

// #ifdef __cplusplus
//   extern "C" {
// #endif
// void hc595Setup(int data, int clock, int latch, int num);
// void hc595ShiftOut(uint32_t value);
// #ifdef __cplusplus
//   }
// #endif

#endif
