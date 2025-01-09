#include <Arduino.h>
#include <SPI.h>
// #include "spi_output.h"
#include "state.h"
#include "hc595.h"
#include "cd4051.h"
#include "lfo.h"
#include "envelope.h"
#include "pins.h"
#include "output.h"
//#include "pwm_setup.h"

//###################################################
//###################################################
// UI GLOBALS
//###################################################
//###################################################

/* 
* ##############################
* Enums
* ##############################
 */

// rearrange this in order of how hardware pins are set on pcb
typedef enum ShiftRegLEDAssign {
  // LED_PARAM_4,  
  // LED_PARAM_1,
  // LED_PARAM_2,
  // LED_PARAM_3,
  // LED_MOD_1,
  // LED_MOD_2,
  // LED_MOD_3,
  // LED_MOD_4,

  LED_MOD_4,
  LED_MOD_3,
  LED_MOD_2,
  LED_MOD_1,
  LED_PARAM_2,
  LED_PARAM_1,
  LED_PARAM_4,
  LED_PARAM_3,

  NUMBER_OF_SHIFT_REG_LEDS
} ShiftRegLEDAssign;

typedef enum MuxInputAssign {
  MUX_POT_1,
  MUX_POT_2,
  MUX_POT_3,
  MUX_POT_4,
  MUX_CV_1,
  MUX_CV_2,
  MUX_CV_3,
  MUX_CV_4,

  // MUX_POT_1,
  // MUX_CV_1,
  // MUX_POT_3,
  // MUX_CV_3,
  // MUX_POT_2,
  // MUX_CV_2,
  // MUX_POT_4,
  // MUX_CV_4

} MuxInputAssign;

const MuxInputAssign kmux_pot[] = {MUX_POT_1, MUX_POT_2, MUX_POT_3, MUX_POT_4};
const MuxInputAssign kmux_cv[] = {MUX_CV_1, MUX_CV_2, MUX_CV_3, MUX_CV_4};
const uint8_t kmux_cv_offset = MUX_CV_1;


/* 
* ##############################
* Declarations
* ##############################
 */
// Event timer
const unsigned long kevent_timer_length = 1; // 1 ms
unsigned long event_timer;
const uint8_t kswitch_bounce_counter_rollover = 50;
uint8_t switch_bounce_counter;

// Analog declarations
CD4051Data cd4051;

// LED declarations
HC595Data hc595;
typedef struct LEDData {
  bool led[NUMBER_OF_SHIFT_REG_LEDS];
  bool led_menu;
  uint8_t led_data;
} LEDData;
const uint8_t kled_mod_offset = 4;

// Switch declarations
uint8_t last_sw_state[NUMBER_OF_SWITCHES]; // TODO: FOR LOOP MAKE THIS TRUE
uint8_t sw_pressed_count = 0;
bool prevent_sw_action_cv_param;

// -- Switch timer
const unsigned long ksw_timer_length = 500L;
unsigned long sw_timer;
bool sw_timer_active;

// -- Assigning switch UI to modulator
const ModulatorNumber sw_to_mod[knumber_of_modulator_switches] = {MODULATOR_1, MODULATOR_2, MODULATOR_3, MODULATOR_4};
const SwitchAssign mod_to_sw[knumber_of_modulator_switches] = {SW_MOD_1, SW_MOD_2, SW_MOD_3, SW_MOD_4};
typedef struct ModSwitchAssign {
  ModulatorNumber m;
  SwitchAssign s;
}ModSwitchAssign;
ModSwitchAssign mod_switch[knumber_of_modulator_switches];

// -- lock parameter values until pot turned to threshold
bool param_lock_initiated = false;
bool param_locked[NUMBER_OF_POTS];
uint8_t locked_value[NUMBER_OF_POTS];
const uint8_t kpot_turn_threshold = 32;


/* 
* ##############################
* Functions
* ##############################
 */
void switchAndDisplayRoutine(/* MachineState ms, ModulatorData md */);
void handleSwitchPress(SwitchAssign s, StatePage sp, ModulatorNumber cm, ModulatorData md);
void handleSwitchRelease(SwitchAssign s, StatePage sp, ModulatorNumber cm, ModulatorData md, bool alarm_up);
void handleSwitchTimed(SwitchAssign s, StatePage sp, ModulatorNumber cm, ModulatorData md);
LEDData displayPageLEDs(MachineState ms, ModulatorData md);
void changeModType(ModulatorType t, ModulatorNumber n);
void handleAnalogInputs();

int32_t saturateValue(int value, int top, int bottom) {
  if(value >= top) return top;
  else if(value <= bottom) return bottom;
  return value;
}

int32_t mapValue(int32_t in_val, int32_t in_max, int32_t out_max) {
  return (in_val * out_max) / in_max;
}

void lockPotOnModulatorChange(uint8_t index, uint8_t value) {
  param_locked[index] = true;
  locked_value[index] = value;
}

void unlockPotCheck(uint8_t index, uint8_t value) {
  if(abs(value - locked_value[index]) >= kpot_turn_threshold) {
    param_locked[index] = false;
  }
}

//###################################################
//###################################################
// MODULATION GLOBALS
//###################################################
//###################################################

#define OUTPUT_BIT_WIDTH 10
#define MAX_OUTPUT_LEVEL ((1 << OUTPUT_BIT_WIDTH) - 1)
#define HALF_MAX_OUTPUT_LEVEL (1 << (OUTPUT_BIT_WIDTH - 1))
#define ZERO_OFFSET HALF_MAX_OUTPUT_LEVEL
#define LFO_TRUNCATION_BITS LFO_DEFAULT_RESOLUTION_BITS - OUTPUT_BIT_WIDTH
#define ENV_TRUNCATION_BITS ENV_DEFAULT_RESOLUTION_BITS - OUTPUT_BIT_WIDTH

/* 
* ##############################
* Constants
* ##############################
 */
//const float clock_div[] = {64.0, 48.0, 32.0, 24.0, 16.0, 12.0, 8.0, 6.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.25, 0.125, 0.0625};
const uint16_t kclock_div[] = {1024, 768, 512, 384, 256, 192, 128, 96, 64, 48, 32, 16, 8, 4, 2, 1};
// the unsigned clock_div values equal the float ones commented out when divided by this value
const uint8_t kdiv_clock_div = 4; 
const uint16_t kupdate_rate_in_us = 1000;
const uint32_t ksecond_in_us = 1000000;
const uint32_t kclock_div_numerator = ksecond_in_us * 100;
const uint16_t kupdate_rate_in_hz = ksecond_in_us / kupdate_rate_in_us;
const uint32_t kclock_div_denominator = kupdate_rate_in_hz * 100;
const unsigned long kext_clock_timeout = 4000000;
/* 
* ##############################
* Declarations
* ##############################
 */
// LFO Declarations
LFO_Params lfo[NUMBER_OF_MODULATORS];
Env_Params env[NUMBER_OF_MODULATORS];
unsigned long last_process_time;
volatile bool ext_clock_active[NUMBER_OF_MODULATORS];
volatile bool last_ext_clock_active[NUMBER_OF_MODULATORS];
volatile bool ext_pulse_occurred[NUMBER_OF_MODULATORS];
volatile uint32_t ext_clock_period[NUMBER_OF_MODULATORS];
volatile uint32_t last_ext_clock_period[NUMBER_OF_MODULATORS];
volatile unsigned long last_clock_time[NUMBER_OF_MODULATORS];
volatile bool lfo_reset_flag[NUMBER_OF_MODULATORS];
// volatile bool disable_follow_clock_1[NUMBER_OF_MODULATORS];
uint8_t tick_counter[NUMBER_OF_MODULATORS];
uint8_t last_div[NUMBER_OF_MODULATORS];

// Envelope Declarations
volatile bool gate_active[NUMBER_OF_MODULATORS] = {false, false, false, false};
volatile bool last_gate_event[NUMBER_OF_MODULATORS] = {false, false, false, false};

/* 
* ##############################
* Functions
* ##############################
 */
void lfoPulseCallback(ModulatorNumber mod_num);
void lfoISRHandler(ModulatorNumber n);
void lfoISR1();
void lfoISR2();
void lfoISR3();
void lfoISR4();
void (*lfoISR_P[NUMBER_OF_MODULATORS])(void) = 
  {lfoISR1, lfoISR2, lfoISR3, lfoISR4};
void envISRHandler(ModulatorNumber n);
void envISR1();
void envISR2();
void envISR3();
void envISR4();
void (*envISR_P[NUMBER_OF_MODULATORS])(void) = 
  {envISR1, envISR2, envISR3, envISR4};
void changeModInterrupt(ModulatorType mod_type, ModulatorNumber mod_num);
uint16_t processLFO(ModulatorNumber mod_num, uint8_t rate, uint8_t wave, uint8_t augment, uint8_t phase_mod);
uint16_t processEnv(ModulatorNumber mod_num, uint8_t attack, uint8_t decay, uint8_t sustain, uint8_t release);
void processModulators();

// pulse pin polling
// #define PULSE_PIN_POLLING

#ifdef PULSE_PIN_POLLING
bool last_pulse_status[NUMBER_OF_MODULATORS];

void pulsePinSetup() {

  for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
    pinMode(kpulse_pin[i], INPUT);
  }
}

void pulsePinPolling() {

  for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
    ModulatorNumber mod_num = (ModulatorNumber)i;
    ModulatorData m = stateGetModulatorData(mod_num);
    bool pulse_status = digitalRead(kpulse_pin[i]);
    if(pulse_status != last_pulse_status[i]) {
      last_pulse_status[i] = pulse_status;
      if(m.mod_type == MOD_TYPE_LFO) {
        if(!pulse_status) {
          if(i == 0) { 
            lfoISR1();
          } else {
            // if(!m.follow_clock_in_1) {
            //   lfoPulseCallback(mod_num);
            // }
            lfoISRHandler(mod_num);
          }
        }
      } else {
        if(i == 0) {
          gate_active[MODULATOR_1] = !pulse_status; 
          for(int i = 0; i < NUMBER_OF_MODULATORS - 1; i++) {
            ModulatorNumber j = (ModulatorNumber)(i + 1);
            ModulatorData md = stateGetModulatorData(j);
            if(md.follow_clock_in_1 && md.mod_type == MOD_TYPE_ENV) {
              gate_active[j] = !pulse_status;
            }
          }  
        } else {
          if(!m.follow_clock_in_1) {
            gate_active[i] = !pulse_status;
          }
        }
      }
    }
  }
}

#endif

void setup() {
  Serial.begin(115200);
  stateInit();

  cd4051Setup(&cd4051, cd4051_pin[0], cd4051_pin[1], cd4051_pin[2], cd4051_pin[3]);
  hc595Setup(&hc595, khc595_pin[0], khc595_pin[1], khc595_pin[2], 1);
  pinMode(menu_led_pin, OUTPUT);  
  for(int i = 0; i < NUMBER_OF_SWITCHES; i++) {
    pinMode(kswitch[i], INPUT_PULLUP);
    last_sw_state[i] = 1; // initialize last_sw_state as high/true since switch pins are pulled high
  }

  for(int i = 0; i < knumber_of_modulator_switches; i++) {
    mod_switch[i] = (ModSwitchAssign) {
      .m = sw_to_mod[i],
      .s = mod_to_sw[i],
    };
  }
  
#ifdef PULSE_PIN_POLLING
  pulsePinSetup();
#endif

  MachineState ms = stateGetMachineState();
  ModulatorData md = stateGetCurrentModulatorData();
  LEDData l = displayPageLEDs(ms, md);
  digitalWrite(menu_led_pin, l.led_menu);
  hc595ShiftOut(&hc595, l.led_data);

  for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
    ModulatorNumber mod_num = (ModulatorNumber)i;
    md = stateGetModulatorData(mod_num);
    changeModType(md.mod_type, mod_num);
    LFOSetup(&lfo[mod_num], analogRead(pot_pin) << (i + random(0, 8)));
    EnvInit(&env[mod_num]);
  }
  outputSetup();
  // spiSetup();
}

void loop() {

#ifdef PULSE_PIN_POLLING
  pulsePinPolling();
#endif

  if((millis() - event_timer) >= kevent_timer_length) {
    event_timer = millis();
    if(switch_bounce_counter >= kswitch_bounce_counter_rollover) {
      switch_bounce_counter = 0;
      switchAndDisplayRoutine();
    } else {
      switch_bounce_counter++;
    }

    handleAnalogInputs();
    processModulators();

  }
}

void switchAndDisplayRoutine(/* MachineState ms, ModulatorData md */) {
   // Declaring these at the top of scope to reuse for LEDs after switches change data
    MachineState ms = stateGetMachineState();
    ModulatorData md = stateGetCurrentModulatorData();

    bool change_display = false;
    for(int i = 0; i < NUMBER_OF_SWITCHES; i++) {
      uint8_t sw_state = digitalRead(kswitch[i]);
      if(sw_state != last_sw_state[i]) {
        last_sw_state[i] = sw_state;
        change_display = true;
        SwitchAssign s = (SwitchAssign) i;
        if(!sw_state) {
          if(!sw_pressed_count) {
            sw_timer_active = true;
            sw_timer = millis();
          } else {
            sw_timer_active = false;
          }
          sw_pressed_count++;
          handleSwitchPress(s, ms.current_page, ms.current_modulator, md);
        } else {
          bool timer_alarm = false;
          if(sw_timer_active) {
            if((millis() - sw_timer) >= ksw_timer_length) {
            timer_alarm = true;
            }
          }
          sw_pressed_count--;
          handleSwitchRelease(s, ms.current_page, ms.current_modulator, md, timer_alarm);
        }
      }
    }

    if(change_display) {
      // the data is different than before, so collect it before displaying it
      ms = stateGetMachineState();
      md = stateGetCurrentModulatorData();
      LEDData l = displayPageLEDs(ms, md);
      digitalWrite(menu_led_pin, l.led_menu);
      hc595ShiftOut(&hc595, l.led_data);
    }
}

/*
* ##############################
* UI Function Definitions
* ##############################
*/

void handleSwitchPress(SwitchAssign s, StatePage sp, ModulatorNumber cm, ModulatorData md) {

  if(s == SW_MENU) {
    // StatePage new_sp = sp == PAGE_CONTROL? PAGE_MENU : PAGE_CONTROL;
    // stateSetCurrentPage(new_sp);
  } else {
    if(sp == PAGE_CONTROL) {
      if(sw_to_mod[s] != cm) {
        stateSetCurrentModulator(sw_to_mod[s]);
        // make sure you don't change the cv param every time you pick a new modulator
        prevent_sw_action_cv_param = true; 
        param_lock_initiated = true;
      } else {
        prevent_sw_action_cv_param = false;
      }
    } else if (sp == PAGE_MENU) {
      switch(s) {
        case SW_MOD_1:
          changeModType(md.mod_type == MOD_TYPE_LFO? MOD_TYPE_ENV : MOD_TYPE_LFO, cm);
        break;

        case SW_MOD_2:
          stateSetFollowClockIn1(md.follow_clock_in_1 == true? false : true, cm);
        break;

        case SW_MOD_3:
          stateSetLFOReset(md.LFO_reset == true? false : true, cm);
        break;

        case SW_MOD_4:
          stateSetEnvLoop(md.env_loop == true? false : true, cm);
        break;
      }
    }
  }
}

void handleSwitchRelease(SwitchAssign s, StatePage sp, ModulatorNumber cm, ModulatorData md, bool alarm_up) {

  if(s!= SW_MENU /* && sp == PAGE_CONTROL */) {
    if(sp == PAGE_CONTROL) {
      // Serial.print(alarm_up);
      // Serial.print("\t");
      // Serial.print(sw_to_mod[s] == cm);
      // Serial.print("\t");
      // Serial.print(prevent_sw_action_cv_param);
      // Serial.print("\t");
      if(!alarm_up && sw_to_mod[s] == cm /* && !prevent_sw_action_cv_param */) {
        // uint8_t cvc = md.cv_control_param;
        // if(cvc >= NUMBER_OF_CV_CONTROLLABLE_PARAMS - 1) cvc = 0;
        // else cvc++;
        // stateSetCVControlParam(cvc, cm);
        if(!prevent_sw_action_cv_param) {
          uint8_t cvc = md.cv_control_param;
          // if(cvc >= NUMBER_OF_CV_CONTROLLABLE_PARAMS - 1) cvc = 0;
          // One additional option for no param cv controlled
          if(cvc >= NUMBER_OF_CV_CONTROLLABLE_PARAMS) cvc = 0;
          else cvc++;
          stateSetCVControlParam(cvc, cm);
        }
      } else { // if alarm is triggered
        ModulatorData amd = stateGetModulatorData(/* (ModulatorNumber) s */mod_switch[s].m);
        ModulatorType t = amd.mod_type == MOD_TYPE_LFO? MOD_TYPE_ENV : MOD_TYPE_LFO;
        // stateSetModType(t, mod_switch[s].m);
        changeModType(t, mod_switch[s].m);
        // Serial.print("m");
      }
      // Serial.println();
    }
  } else {
    if(!alarm_up) {
      StatePage new_sp = sp == PAGE_CONTROL? PAGE_MENU : PAGE_CONTROL;
      stateSetCurrentPage(new_sp);
    } else {
      for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
        lfo_reset_flag[i] = true;
      }
    }
  }
}

LEDData displayPageLEDs(MachineState ms, ModulatorData md) {

  LEDData l = (LEDData) {.led =  {0, 0, 0, 0, 0, 0, 0, 0}, .led_data = 0};
    // l.led_data |= 0b10000000 >> ms.current_modulator;
    const ShiftRegLEDAssign mod_led[] = {LED_MOD_1, LED_MOD_2, LED_MOD_3, LED_MOD_4};
    // l.led_data |= 0b00000001 << mod_led[ms.current_modulator];
    l.led_data |= 1 << mod_led[ms.current_modulator];

  if(ms.current_page == PAGE_CONTROL) {
    const uint8_t kled_cv_param_offset[] = {LED_PARAM_1, LED_PARAM_2, LED_PARAM_3, LED_PARAM_4};
    uint8_t cvc_num = md.cv_control_param;
    if(cvc_num != NUMBER_OF_CV_CONTROLLABLE_PARAMS) {
      l.led_data |= 1 << (kled_cv_param_offset[cvc_num]);
    }
  } else if (ms.current_page == PAGE_MENU) {
    l.led_menu = true;
    l.led_data |= md.mod_type << LED_PARAM_1;
    l.led_data |= md.follow_clock_in_1 << LED_PARAM_2;
    l.led_data |= md.LFO_reset << LED_PARAM_3;
    l.led_data |= md.env_loop << LED_PARAM_4;    
  }

  return l;
}

void handleAnalogInputs() {
  
  ModulatorNumber cm = stateGetCurrentModulatorNumber();
  for(int i = 0; i < NUMBER_OF_POTS; i++) {
    uint8_t read = 0;
    if(i <= 3) {
      read = cd4051Read(&cd4051, kmux_pot[i]) >> 2;
    } else {
      read = analogRead(pot_pin) >> 2;
    }
    // uint8_t read = cd4051Read(&cd4051, kmux_pot[i]) >> 2;
    // stateSetParam(read, i, cm);

    if(!param_lock_initiated) {
      if(!param_locked[i]) {
        stateSetParam(read, i, cm);
      } else {

        // if you want to set it so that the parameter changes when approaching
        // the saved parameter, you'll have to add the current modulator to this function
        // and then inside the function, check against the saved state modulator value
        // instead of the value set upon change in the lockPotOnModulatorChange function below
        // that function only has to then flip the pot_locked bool
        unlockPotCheck(i, read);
      }
    } else {
      // flip this bool on the final loop, otherwise it only locks the first value
      // if(i >= NUMBER_OF_4051_ANALOG_INPUTS - 1) param_lock_initiated = false;
      lockPotOnModulatorChange(i, read);
    }
  }

  if(param_lock_initiated) param_lock_initiated = false;

  // uint8_t read = analogRead(pot_pin) >> 2;
  // stateSetParam(read, 4, cm);

  for(int i = 0; i < NUMBER_OF_4051_ANALOG_INPUTS / 2; i++) {
    uint8_t cv = cd4051Read(&cd4051, kmux_cv[i]) >> 2;
    ModulatorData m = stateGetModulatorData((ModulatorNumber)i);
    uint8_t cvc = m.cv_control_param;
    uint8_t base_val = 0;
    if(cvc != NUMBER_OF_CV_CONTROLLABLE_PARAMS) {
      base_val = m.param[cvc];
    }
    cv = saturateValue(base_val + cv, 255, 0);
    stateSetCVValue(cv, (ModulatorNumber)i);
  }
  
  // for(int i = 0; i < NUMBER_OF_4051_ANALOG_INPUTS; i++) {
  //   Serial.print(cd4051Read(&cd4051, i) >> 2);
  //   Serial.print("\t");
  // }
  // Serial.println();
}

/*
* ##############################
* Modulator Function Definitions
* ##############################
*/

void lfoPulseCallback(ModulatorNumber mod_num) {
  // if(!ext_pulse_occurred[mod_num]){
  //   unsigned long now = micros();
  //   ext_clock_period[mod_num] = now - last_clock_time[mod_num];
  //   last_clock_time[mod_num] = now;
  //   ext_clock_active[mod_num] = true;
  //   ext_pulse_occurred[mod_num] = true;
  // }
  unsigned long now = micros();
  ext_clock_period[mod_num] = now - last_clock_time[mod_num];
  last_clock_time[mod_num] = now;
  ext_clock_active[mod_num] = true;
  ext_pulse_occurred[mod_num] = true;
}

void lfoISR1() { 
  if(!stateGetLFOReset(MODULATOR_1)) {
    lfoPulseCallback(MODULATOR_1); 
    for(int i = 0; i < NUMBER_OF_MODULATORS - 1; i++) {
      ModulatorNumber j = (ModulatorNumber)(i + 1);
      ModulatorData md = stateGetModulatorData(j);
      if(md.follow_clock_in_1 && md.mod_type == MOD_TYPE_LFO) {
        lfoPulseCallback(j);
      }
    }
  } else {
    lfo_reset_flag[MODULATOR_1] = true;
  }  
}

void lfoISRHandler(ModulatorNumber n) {

  // bool allow_reset = false;
  // if(!stateGetLFOReset(n)) {
  //   if(!stateGetFollowClockIn1(n)) {
  //     lfoPulseCallback(n);
  //   }
  // } else {
  //   lfo_reset_flag[n] = true;
  // }

  bool allow_reset = false;
  if(stateGetLFOReset(n)) {
    allow_reset = true;
  }

  bool allow_self_ext_clock = false;
  if(!stateGetFollowClockIn1(n)) {
    allow_self_ext_clock = true;
  }
  // if reset is on, do not allow self ext clock. If reset is off, check if following clock 1
  if(allow_reset) {
    lfo_reset_flag[n] = true;
  } else {
    if(allow_self_ext_clock) {
      lfoPulseCallback(n);
    }
  }
}

void lfoISR2() { lfoISRHandler(MODULATOR_2); }
void lfoISR3() { lfoISRHandler(MODULATOR_3); }
void lfoISR4() { lfoISRHandler(MODULATOR_4); }

// void lfoISR2() { if(!stateGetFollowClockIn1(MODULATOR_2))lfoPulseCallback(MODULATOR_2); }
// void lfoISR3() { if(!stateGetFollowClockIn1(MODULATOR_3))lfoPulseCallback(MODULATOR_3); }
// void lfoISR4() { if(!stateGetFollowClockIn1(MODULATOR_4))lfoPulseCallback(MODULATOR_4); }

// void lfoISR1() { lfoPulseCallback(MODULATOR_1); }
// void lfoISR2() { lfoPulseCallback(MODULATOR_2); }
// void lfoISR3() { lfoPulseCallback(MODULATOR_3); }
// void lfoISR4() { lfoPulseCallback(MODULATOR_4); }

// Gate considered 'high' on FALLING because hardware input is inverted
void envISR1() { 
  bool gate_is_active = !digitalRead(kpulse_pin[0]);
  gate_active[MODULATOR_1] = gate_is_active; 
  for(int i = 0; i < NUMBER_OF_MODULATORS - 1; i++) {
    ModulatorNumber j = (ModulatorNumber)(i + 1);
    ModulatorData md = stateGetModulatorData(j);
    if(md.follow_clock_in_1 && md.mod_type == MOD_TYPE_ENV) {
      gate_active[j] = gate_is_active;
    }
  }  
}

void envISRHandler(ModulatorNumber n) {
  if(!stateGetFollowClockIn1(n) && stateGetModulatorType(n) == MOD_TYPE_ENV) {
    gate_active[n] = !digitalRead(kpulse_pin[n]);
  } 
}

void envISR2() { envISRHandler(MODULATOR_2); }
void envISR3() { envISRHandler(MODULATOR_3); }
void envISR4() { envISRHandler(MODULATOR_4); }

// void envISR2() { if(!stateGetFollowClockIn1(MODULATOR_2)) gate_active[MODULATOR_2] = !digitalRead(kpulse_pin[1]); }
// void envISR3() { if(!stateGetFollowClockIn1(MODULATOR_3)) gate_active[MODULATOR_3] = !digitalRead(kpulse_pin[2]); }
// void envISR4() { if(!stateGetFollowClockIn1(MODULATOR_4)) gate_active[MODULATOR_4] = !digitalRead(kpulse_pin[3]); }

// void envISR2() { gate_active[MODULATOR_2] = !digitalRead(kpulse_pin[1]); }
// void envISR3() { gate_active[MODULATOR_3] = !digitalRead(kpulse_pin[2]); }
// void envISR4() { gate_active[MODULATOR_4] = !digitalRead(kpulse_pin[3]); }

uint16_t processLFO(ModulatorNumber mod_num, bool follow, uint8_t rate, uint8_t wave, uint8_t augment, uint8_t phase_mod) {

  // ModulatorData md = stateGetModulatorData(mod_num);
  // ModulatorNumber mod_num_follow ;
  // if(follow) mod_num_follow = MODULATOR_1;
  // else mod_num_follow = mod_num;

  wave = mapValue(wave, 255, NUMBER_OF_LFO_SHAPES);
  if(wave >= 4) wave = 4;
  uint16_t div = kclock_div[rate >> 4];
  uint16_t increment = (rate + 1) << 3;

  uint32_t period = (uint32_t)(ext_clock_period[mod_num] * div) >> kdiv_clock_div;
  uint8_t divided_div = div >> 4;
  bool dv_changed = false;
  if(divided_div != last_div[mod_num]) {
    last_div[mod_num] = divided_div;
    dv_changed = true;
  }
  if(divided_div <= 1) divided_div = 1;
  uint16_t phase_div = 65536 / divided_div;
  uint16_t ext_clock_increment = ((kclock_div_numerator / period) * 65536) / kclock_div_denominator;
  
  if(ext_clock_active[mod_num]) {
    increment = ext_clock_increment;
    if(ext_pulse_occurred[mod_num]) {
      
      // if(mod_num == MODULATOR_2) {
      //   // Serial.print(ext_pulse_occurred[1]);
      //   // Serial.print("\t");
      //   Serial.print(ext_clock_increment);
      //   // Serial.print(rate);
      //   // Serial.print("\t");
      //   // Serial.print(divided_div);
      //   // Serial.print("\t");
      //   // Serial.print("pulse");
      //   Serial.println();
      // }

      ext_pulse_occurred[mod_num] = false;
      if(divided_div > 1) {
        lfo[mod_num].phase_ = phase_div * tick_counter[mod_num];
      } else {
        LFOResetNow(&lfo[mod_num]);
      }
      if(tick_counter[mod_num] >= divided_div - 1) {
        tick_counter[mod_num] = 0;
      } else {
        tick_counter[mod_num]++;
      }
      if(dv_changed) {
        dv_changed = false;
        LFOResetNow(&lfo[mod_num]);
        tick_counter[mod_num] = 0;
      }
    }
    if((micros() - last_clock_time[mod_num]) >= kext_clock_timeout){
      ext_clock_active[mod_num] = false;
      // Serial.println("ext_clk_stop");
      // Serial.println(micros() - last_clock_time[mod_num]);
    }
  } 

  if(lfo_reset_flag[mod_num]) {
    lfo_reset_flag[mod_num] = false;
    LFOResetNow(&lfo[mod_num]);
    tick_counter[mod_num] = 0;
  }

  // LFOUpdate(&lfo[mod_num], increment, wave, 0, augment, phase_mod);
  LFOUpdate(&lfo[mod_num], increment, wave, phase_mod, augment, 0);
  return LFORender(&lfo[mod_num]);
}

uint16_t processEnv(ModulatorNumber mod_num, bool follow, uint8_t attack, uint8_t decay, uint8_t sustain, uint8_t release) {

  ModulatorNumber mod_num_follow;
  if(follow) mod_num_follow = MODULATOR_1;
  else mod_num_follow = mod_num;

  if(last_gate_event[mod_num] != gate_active[mod_num]) {
    last_gate_event[mod_num] = gate_active[mod_num];
    if(gate_active[mod_num]) {
      EnvTrigger(ATTACK, &env[mod_num]);
    } else {
      EnvTrigger(RELEASE, &env[mod_num]);
    }
  }

  if(stateGetEnvLoop(mod_num) && env[mod_num].stage_ == DEAD){
    EnvTrigger(ATTACK, &env[mod_num]);
  }

  // Serial.println(env[1].stage_);

  EnvUpdate(attack, decay, sustain, release, &env[mod_num]);
  return EnvRender(&env[mod_num]);
}

void changeModInterrupt(ModulatorType mod_type, ModulatorNumber mod_num) {

  detachInterrupt(digitalPinToInterrupt(kpulse_pin[mod_num]));
  if(mod_type == MOD_TYPE_LFO) {
    // Pulse on FALLING because hardware input is inverted
    attachInterrupt(digitalPinToInterrupt(kpulse_pin[mod_num]), lfoISR_P[mod_num], FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(kpulse_pin[mod_num]), envISR_P[mod_num], CHANGE);
  }
}

void changeModType(ModulatorType t, ModulatorNumber n) {
  stateSetModType(t, n);

#ifndef PULSE_PIN_POLLING
  changeModInterrupt(t, n);
#endif

}

void processModulators() {

  ModulatorData mod1 = stateGetModulatorData(MODULATOR_1);

  for(int i = 0; i < NUMBER_OF_MODULATORS; i++) {
    int16_t output_wave;
    ModulatorNumber mod_num = (ModulatorNumber)i;
    ModulatorData md = stateGetModulatorData(mod_num);
    ModulatorType t = md.mod_type;
    uint8_t level = 2; // for readability, lower down
    uint8_t p[5];

    bool follow = false;
    if(md.mod_type == mod1.mod_type && md.follow_clock_in_1) {
      follow = true;
    }

    for(int j = 0; j < 5; j++) {
      p[j] = md.param[j];
      // if(i == 0) {
      //   Serial.print(p[j]);
      //   Serial.print("\t");
      // }
      uint8_t cvc = md.cv_control_param;
      if(cvc != NUMBER_OF_CV_CONTROLLABLE_PARAMS) {
        if(j == md.cv_control_param) p[j] = md.cv_value;
      }
    }
    // if(i == 0) Serial.println();
    // uint8_t p[4];
    // p[0] = md.param[0];
    // p[1] = md.param[1];
    // uint8_t level = md.param[2];
    // p[2] = md.param[3];
    // p[3] = md.param[4];
    
    // Serial.print(md.cv_control_param);
    // Serial.print("\t");
    // MachineState m = stateGetMachineState();
    // if(mod_num == m.current_modulator) {
    //   Serial.print(mod_num);
    //   Serial.print("\t");
    //   Serial.print(t);
    //   Serial.print("\t");
    //   Serial.print(p[0]);
    //   Serial.print("\t");
    //   Serial.print(p[1]);
    //   Serial.print("\t");
    //   Serial.print(p[2]);
    //   Serial.print("\t");
    //   Serial.print(p[3]);
    //   Serial.print("\t");
    //   Serial.print(p[4]);
    //   Serial.println();
    // }

    if(t == MOD_TYPE_LFO) {
      output_wave = processLFO(mod_num, follow, p[0], p[1], p[3], p[4]) >> LFO_TRUNCATION_BITS;
      output_wave -= HALF_MAX_OUTPUT_LEVEL;
      output_wave = (output_wave * p[level]) >> 8;
    } else {
      output_wave = processEnv(mod_num, follow, p[0], p[1], p[4], p[3]) >> ENV_TRUNCATION_BITS;
      int8_t b_level = p[level] - 128;
      output_wave = (output_wave * b_level) >> 8;
    }
    output_wave = saturateValue(output_wave += ZERO_OFFSET, MAX_OUTPUT_LEVEL, 0);

    outputProcess((ModulatorNumber) i, output_wave);

    // // an old trick. 
    // SPIChipSelect chip = (SPIChipSelect)(i / 2);
    // uint8_t chan = i % 2;
    // spiOutput(chip, chan, output_wave);
    
    // pwmSetLevel((PWM_Channel)i, output_wave);
    // if(i == 0) {
    //   Serial.println(output_wave);
    // }
  }
  // Serial.println();
}