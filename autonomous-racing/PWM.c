#ifndef PWM_H_
#define PWM_H_

// PWM.c
// Runs on MSP432
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include "PWM.h"


//***************************PWM_Init34*******************************
// PWM outputs on P2.6, P2.7
// Inputs:  period (0.667us)
//          duty3
//          duty4
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 4/12MHz = 333.33ns
// period of P7.3 squarewave is 4*period*333.33ns = 50hz (corresponds to 100hz timer)
// P2.6=1 when timer equals TA0CCR3 on way down, P2.6=0 when timer equals TA0CCR3 on way up
// P2.7=1 when timer equals TA0CCR4 on way down, P2.7=0 when timer equals TA0CCR4 on way up
// Period of P2.6 is period*0.6667us, duty cycle is duty3/period
// Period of P2.7 is period*0.6667us, duty cycle is duty4/period
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    // write this as part of Lab 13
    if(duty3 > period || duty4 > period) return;

    // configure pins
    // assuming its secondary module function as it is above, verify in testing
    P2->DIR |= 0xC0; // set p2.6 and p2.7 to outputs
    P2->SEL0 |= 0xC0; // set sel0 to secondary module pin function for pins 6 and 7
    P2->SEL1 &= ~0xC0; // set sel1 to secondary module pin function for pins 6 and 7

    // # warning "remove this before code integration!!!! will break the reflectance sensors"
    // P7->DIR |= 0x08; //set p7.3 to output. this is tied to CCR0
    // P7->SEL0 |= 0x08; // set P7.3 to secondary module function
    // P7->SEL1 &= ~0x08; // set P7.3 to secondary module function

    //configure timer

    // one counter for the period
    TIMER_A0->CCTL[0] = 0x0080; // set output mode of the timer to toggle
    TIMER_A0->CCR[0] = period;  // Period is 2*period*4*83.33ns is 0.6667*period
                                // with period at 15000, this equals 100hz
    TIMER_A0->EX0 = 0x0000;     // divide by 1 for the input clock. this bit along with ID
                                // bits select the overall divider for the input clock.

    // counter for the duty cycle of P2.6 is on TA0.3
    // when selecting the toggle/reset mode as the outmode, the output is toggled when the timer counts
    // to the TAxCCRn value. it is reset when the timer counts to the
    // TAxCCR0 value. That is why we load the period into the module above.
    TIMER_A0->CCTL[3] = 0x040; // set the output mode of the timer to toggle/reset.
    TIMER_A0->CCR[3] = duty3; //load the duty cycle on P2.6 in.

    // counter for the duty cycle of P2.7 is on TA0.4
    TIMER_A0->CCTL[4] = 0x040; // set the output mode of the timer to toggle/reset.
    TIMER_A0->CCR[4] = duty4; //load the duty cycle on P2.6 in.

    //configure the timer.
    TIMER_A0->CTL &= ~0x0030; //stop the timer before configuration
    // bits15-10=XXXXXX, reserved
    // bits9-8=10,       TASSEL bits, set clock source to SMCLK
    // bits7-6=10,       set input clock divider /4
    // bits5-4=11,       set the timer to up/down mode. timer counts to CCR- and then down to 0000h
    // bit3=X,           reserved
    // bit2=0,           set this bit to reset TAxR, the timer divider logic, the count direction
    // bit1=0,           no interrupt on timer
    TIMER_A0->CTL = 0x02B0; //clock is divided by 4


}

//***************************PWM_Duty3*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  duty3
// Outputs: none
// period of P2.6 is 2*period*666.7ns, duty cycle is duty3/period
//void PWM_Duty3(uint16_t duty3){
//    // write this as part of Lab 13
//    if(duty3 > TIMER_A0->CCR[0]) return; //if duty3 > period, bad input
//
//    // otherwise set the new duty cycle.
//    TIMER_A0->CCR[3] = duty3;
//}

//***************************PWM_Duty4*******************************
// change duty cycle of PWM output on P2.7
// Inputs:  duty4
// Outputs: none
// period of P2.7 is 2*period*666.7ns, duty cycle is duty2/period
//void PWM_Duty4(uint16_t duty4){
//    // write this as part of Lab 13
//    if(duty4 > TIMER_A0->CCR[0]) return; //if duty3 > period, bad input
//
//    // otherwise set the new duty cycle.
//    TIMER_A0->CCR[4] = duty4;
//
//}

#endif
