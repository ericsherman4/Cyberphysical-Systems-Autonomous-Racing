#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_


typedef enum states {
    BEGIN,
    S_HALLWAY1_STR, 
    S_HALLWAY1_ALIGN, 
    S_HALLWAY1_TO2,
    S_HALLWAY2_STR,
    S_HALLWAY2_ALIGN, 
    S_HALLWAY2_TO3,
    S_HALLWAY3_ALIGN,
    S_HALLWAY3_STR,
    S_STOP
} states_e;

#endif
