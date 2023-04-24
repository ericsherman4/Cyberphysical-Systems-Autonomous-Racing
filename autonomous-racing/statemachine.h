#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#define NX_STATE(val) next_state=(val); break;
#define STATE_AFTER_BEGIN S_HALLWAY3_STR

// 1/48/10^6*700000 is 15ms update
// 1/48/10^6*1200000 is 25ms update
//1/48/10^6*50000000 is 1.04s so divide by 10 is 100ms
// IF THE UPDATE IS NOT BEING USED, YOU NEED TO COMMENT IT OUT
 #define MS_25_UPDATE 1200000
//#define MS_15_UPDATE 700000
#define SYSTICK_UPDATE MS_25_UPDATE

#define MOTORFAST 8000
#define MOTORSLOW 6000
#define MOTORTURNSPEED 2200

#define ODO_INIT_XPOS 0 
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0
// #warning "DONT FORGET TO CHANGE THIS"
#define DISTANCE_1FT 304800


typedef enum states {
    BEGIN,
    S_HALLWAY1_STR, 
    S_HALLWAY1_ALIGN, 
    S_HALLWAY1_TO2,
    S_HALLWAY2_STR,
    S_HALLWAY2_ALIGN, 
    S_HALLWAY2_TO3,
    S_HALLWAY2_STR_END,
    S_HALLWAY3_ALIGN,
    S_HALLWAY3_STR,
    S_CRASH,
    S_STOP
} states_e;


void StateMachine_Store_Distances(uint32_t * distances);
void upon_entry(states_e state);
void upon_exit(states_e state);
void StateMachine_Main_Run(void);
void StateMachine_AO_Run(void);

#endif
