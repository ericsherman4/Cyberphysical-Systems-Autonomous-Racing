#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "UART0.h"
#include "Odometry.h"
#include "SysTickInts.h"
#include "Bump.h"
#include "Tachometer.h"
#include "I2CB1.h"
#include "Blinker.h"


#define ODO_INIT_XPOS 0 
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0

// 1/48/10^6*700000 is 15ms update
//1/48/10^6*50000000 is 1.04s so divide by 10 is 100ms
#define ODO_UPDATE_PERIOD 700000

#define COUNT_RESET 50

uint8_t count = COUNT_RESET;
uint8_t state = 0;

void SysTick_Handler()
{
    UpdatePosition();
    if(--count == 0)
    {
        count = COUNT_RESET;
        state = !state;
        if(state)
            LaunchPad_Output(PINK);
        else
            LaunchPad_Output(GREEN);
    }

}

void main(void)
{
    Clock_Init48MHz();
    Motor_Init();
    LaunchPad_Init();
    LaunchPad_LED(0);
    Bump_Init();
    Tachometer_Init();
    Blinker_Init();
    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);
    
    SysTick_Init_Ints(ODO_UPDATE_PERIOD, 4);
    EnableInterrupts();



    // 0.0001 = 10^-4
    // 12in = 30.48cm/0.0001 = 304800
    // zero for success
    if(!ForwardUntilX(304800))
    {
        LaunchPad_LED(1);
    }
    else
    {
        LaunchPad_LED(0);
    }
    Motor_Stop();
    Clock_Delay1ms(2000);
    LaunchPad_LED(0);

    //desired theta in units of 2*pi/16384 radians
    //4096 is turn right

    if(!SoftRightUntilTh(-(16384*1/4)))
    {
        LaunchPad_LED(1);
    }
    else
    {
        LaunchPad_LED(0);
    }
    Motor_Stop();
    Clock_Delay1ms(2000);
    LaunchPad_LED(0);

//    if(!SoftLeftUntilTh((16384*1/4)))
    if(!SoftLeftUntilTh(0))
    {
        LaunchPad_LED(1);
    }
    else
    {
        LaunchPad_LED(0);
    }
    Motor_Stop();
//    Clock_Delay1ms(2000);
//    LaunchPad_LED(0);

    while(1)
    {
        // Systick handler is currently running

        

    }

}
