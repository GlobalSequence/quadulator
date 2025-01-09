#ifndef OUTPUT_H
#define OUTPUT_H

#include "Arduino.h"
#include "pins.h"
#include "state.h"

void outputSetup() {
    
    for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
        pinMode(pwmPin[i], OUTPUT);
    }
    analogWriteFreq(125000);
    analogWriteResolution(10);
    analogWriteRange(1023);
}

void outputProcess(ModulatorNumber n, uint16_t sample) {
    analogWrite(pwmPin[n], sample);
}

#endif