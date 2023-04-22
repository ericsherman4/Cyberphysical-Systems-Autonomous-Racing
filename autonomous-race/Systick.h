
#ifndef SYSTICK_H_
#define SYSTICK_H_


#include <stdint.h>
#include "msp.h"



// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void);


// Time delay using busy wait.
// The delay parameter is in units of the core clock.
// assumes 48 MHz bus clock
void SysTick_Wait(uint32_t delay);


// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay);


void SysTick_Wait1us(uint32_t delay);

#endif //SYSTICK_H_

