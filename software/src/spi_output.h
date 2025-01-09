#ifndef SPI_OUTPUT_H
#define SPI_OUTPUT_H

#include <SPI.h>
#include "pins.h"

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
int dac_config[] = {DAC_config_chan_A, DAC_config_chan_B};

SPISettings spisettings(62500000, MSBFIRST, SPI_MODE0);

typedef enum SPIChipSelect {
    SPI_CHIP_1,
    SPI_CHIP_2,

    NUMBER_OF_SPI_CHIPS
} SPIChipSelect;

void spiSetup() {
    
    // SPI.setCS(SPI_CS_1);
    SPI.setSCK(SPI_SCK);
    SPI.setTX(SPI_MOSI);
    pinMode(SPI_CS_1, OUTPUT);
    pinMode(SPI_CS_2, OUTPUT);
    SPI.begin();
}

void spiOutput(SPIChipSelect chip, uint8_t channel, uint16_t sample) {

    SPI.beginTransaction(spisettings);
    digitalWrite(kspi_cs_pin[chip], LOW);
    SPI.transfer16(dac_config[channel] | sample);
    digitalWrite(kspi_cs_pin[chip], HIGH);
    SPI.endTransaction();
}

#endif