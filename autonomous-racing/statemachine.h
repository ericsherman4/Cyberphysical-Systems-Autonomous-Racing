#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#define NX_STATE(val) next_state=(val); break;

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
    S_STOP
} states_e;

extern uint32_t Distances[3];


void upon_entry(states_e state);
void upon_exit(states_e state);
void StateMachine_Main_Run(void);
void StateMachine_AO_Run(void);

#endif
