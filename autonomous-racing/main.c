#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "UART0.h"
#include "Odometry.h"
#include "SysTickInts.h"


#define ODO_INIT_XPOS 0 
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0

// 1/48/10^6*700000 is 15ms update
#define ODO_UPDATE_PERIOD 700000

uint8_t count = 100;
uint8_t state = 0;

void SysTick_Handler()
{
    UpdatePosition();
    if(--count == 0)
    {
        count = 100;
        state = !state;
        if(state)
            LaunchPad_LED(PINK);
        else
            LaunchPad_LED(GREEN);
    }

}

void main(void)
{
    Clock_Init48MHz();
    Motor_Init();
    LaunchPad_Init();
    LaunchPad_LED(0);
    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);
    
    SysTick_Init_Ints(7000000, 4);

    // 2.54*4/0.0001 = 101600
    if(ForwardUntilX(101600))
    {
        LaunchPad_LED(1);
    }
    else
    {
        LaunchPad_LED(0);
    }
    Motor_Stop();

    while(1)
    {
        // Systick handler is currently running

        

    }

}
