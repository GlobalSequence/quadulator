#ifndef PINS_H
#define PINS_H

#include "Arduino.h"
#include "state.h"

// important that this enum is matched to correct switches
typedef enum SwitchAssign {
  SW_MOD_1,
  SW_MOD_2,
  SW_MOD_3,
  SW_MOD_4,
  SW_MENU,

  NUMBER_OF_SWITCHES
} SwitchAssign;

/* 
* ##############################
* Pin Assignments
* ##############################
 */
#define SPI_SCK 18
#define SPI_MOSI 19
#define SPI_CS_1 16
#define SPI_CS_2 17
const int kspi_cs_pin[] = {SPI_CS_1, SPI_CS_2};

// const int cd4051_pin[] = {26, 20, 21, 22};
const int cd4051_pin[] = {26, 22, 21, 20};
const int pot_pin = 27;
const int menu_led_pin = 0;
const int khc595_pin[] = {15, 13, 14};
const int kswitch[NUMBER_OF_SWITCHES] = {2, 3, 4, 5, 1};
const uint8_t knumber_of_modulator_switches = NUMBER_OF_SWITCHES - 1;
const int kpulse_pin[NUMBER_OF_MODULATORS] = {9, 8, 7, 6};
const int pwmPin[NUMBER_OF_MODULATORS] = {19, 18, 17, 16}; 

#endif