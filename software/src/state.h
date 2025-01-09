#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

#define NUMBER_OF_POTS 5
#define NUMBER_OF_CV_CONTROLLABLE_PARAMS 4

/* 
*############################## 
* MACHINE STATE
*##############################
*/

typedef enum StatePage {
  PAGE_CONTROL,
  PAGE_MENU,
  NUMBER_OF_STATE_PAGES
} StatePage;

typedef enum ModulatorNumber {
  MODULATOR_1,
  MODULATOR_2,
  MODULATOR_3,
  MODULATOR_4,

  NUMBER_OF_MODULATORS
} ModulatorNumber;


typedef struct MachineState {
  StatePage current_page;
  ModulatorNumber current_modulator;
} MachineState;

/* 
*############################## 
* MODULATOR STATE
*##############################
*/

typedef enum ModulatorType {
  MOD_TYPE_LFO,
  MOD_TYPE_ENV,
  NUMBER_OF_MOD_TYPES
} ModulatorType;

typedef struct ModulatorData {
  ModulatorNumber mod_number;
  ModulatorType mod_type;
  uint8_t cv_control_param; 
  bool follow_clock_in_1;
  bool clock_sync;
  bool LFO_reset;
  bool env_loop;
  uint8_t param[NUMBER_OF_POTS];
  uint8_t cv_value;

} ModulatorData;

/* 
*############################## 
* FUNCTIONS
*##############################
*/


#ifdef __cplusplus
 extern "C" {
#endif

MachineState stateGetMachineState();
ModulatorData stateGetCurrentModulatorData();
ModulatorNumber stateGetCurrentModulatorNumber();
ModulatorData stateGetModulatorData(ModulatorNumber n);
ModulatorType stateGetModulatorType(ModulatorNumber n);
uint8_t stateGetParamAmount(ModulatorNumber n, uint8_t index);
bool stateGetFollowClockIn1(ModulatorNumber n);
bool stateGetLFOReset(ModulatorNumber n);
bool stateGetEnvLoop(ModulatorNumber n);
void stateInit();
void stateSetCurrentModulator(ModulatorNumber n);
void stateSetCurrentPage(StatePage s);
void stateSetModType(ModulatorType t, ModulatorNumber n);
void stateSetCVControlParam(uint8_t p, ModulatorNumber n);
void stateSetFollowClockIn1(bool f, ModulatorNumber n);
void stateSetClockSync(bool s, ModulatorNumber n);
void stateSetLFOReset(bool r, ModulatorNumber n);
void stateSetEnvLoop(bool l, ModulatorNumber n);
void stateSetParam(uint8_t amount, uint8_t p, ModulatorNumber n);
void stateSetCVValue(uint8_t amount, ModulatorNumber n);

#ifdef __cplusplus
 }
#endif

#endif