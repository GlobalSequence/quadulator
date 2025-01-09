#include "state.h"
#include <Arduino.h>

MachineState machineState;
ModulatorData mod[NUMBER_OF_MODULATORS];

const static uint8_t mod_init_param[NUMBER_OF_MODULATORS][NUMBER_OF_POTS] = {
  {0, 0, 0, 0, 0}, 
  {0, 0, 0, 0, 0}, 
  {0, 0, 0, 0, 0}, 
  {0, 0, 0, 0, 0}, 
};

MachineState stateGetMachineState() {return machineState;}
ModulatorData stateGetCurrentModulatorData() {return mod[machineState.current_modulator];}
ModulatorData stateGetModulatorData(ModulatorNumber n) {return mod[n];}
ModulatorNumber stateGetCurrentModulatorNumber() {return machineState.current_modulator;}
ModulatorType stateGetModulatorType(ModulatorNumber n) {return mod[n].mod_type;}
uint8_t stateGetParamAmount(ModulatorNumber n, uint8_t index) {return mod[n].param[index];}
bool stateGetFollowClockIn1(ModulatorNumber n) {return mod[n].follow_clock_in_1;}
bool stateGetLFOReset(ModulatorNumber n) {return mod[n].LFO_reset;}
bool stateGetEnvLoop(ModulatorNumber n) {return mod[n].env_loop;}

void stateInit() {
  machineState = (MachineState) {
    .current_page = PAGE_CONTROL,
    .current_modulator = MODULATOR_1
  };

  for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
    mod[i] = (ModulatorData) {
      .mod_number = i,
      .mod_type = MOD_TYPE_LFO,
      .cv_control_param = NUMBER_OF_CV_CONTROLLABLE_PARAMS,
      .follow_clock_in_1 = true,
      .clock_sync = true,
      .LFO_reset = false
    };
    for(int j = 0; j < NUMBER_OF_POTS; j++) {
      mod[i].param[j] = mod_init_param[i][j];
    }
  }
}

void stateSetCurrentModulator(ModulatorNumber n) {
  machineState.current_modulator = n;
}

void stateSetCurrentPage(StatePage s) {
  machineState.current_page = s;
}

void stateSetModType(ModulatorType t, ModulatorNumber n) {
  mod[n].mod_type = t;
}

void stateSetCVControlParam(uint8_t p, ModulatorNumber n) {
  mod[n].cv_control_param = p;
}

void stateSetFollowClockIn1(bool f, ModulatorNumber n) {
  mod[n].follow_clock_in_1 = f;
}

void stateSetClockSync(bool s, ModulatorNumber n) {
  mod[n].clock_sync = s;
}

void stateSetLFOReset(bool r, ModulatorNumber n) {
  mod[n].LFO_reset = r;
}

void stateSetEnvLoop(bool l, ModulatorNumber n) {
  mod[n].env_loop = l;
}

void stateSetParam(uint8_t amount, uint8_t p, ModulatorNumber n) {
  mod[n].param[p] = amount;
}

void stateSetCVValue(uint8_t amount, ModulatorNumber n) {
  mod[n].cv_value = amount;
}