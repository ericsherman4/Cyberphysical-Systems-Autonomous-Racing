// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot


#include "Bump.h"
#include <stdint.h>
#include "launchpad.h"



void BumpInt_Init(void){
    P4->IE |= 0xED; //Enabling interrupts for bump pins on port 4
    P4->IES |= 0xED;   //Enabling rising edge interrupt bump pins (I think)
    P4->IFG &= ~0xED; //Clearing interrupt flag
    NVIC_EnableIRQ(PORT4_IRQn);
    //P4->IV |= 0x0F; //Setting interrupt to low priority (not sure if this is a mistake, or what "low" implies)
}

void PORT4_IRQHandler(void){
    LaunchPad_Output(0x01); //Setting LED
    P4->IFG &= ~0xED;       //Clearing Flags
}

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    P4-> SEL0 &= ~0xED; //Initializing P4 bumper pins
    P4-> SEL1 &= ~0xED;
    P4-> DIR |= 0xED;
    P4-> REN |= 0xED; //Enabling pull-up resistors
    P4-> OUT |= 0xED;
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    uint8_t bump, bumpOut; //Creating read variables
    bump = P4->IN; //Reading from input register
    bumpOut = (~bump&0x80) >> 2 | (~bump&0x40) >> 2 | (~bump&0x20) >> 2| (~bump&0x08) >> 1 | (~bump&0x04) >> 1 | (~bump&0x01); //Inverting and shifting bumper bits to align with a 6 bit output
    return bumpOut;
}

