#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "UART0.h"
#include "Odometry.h"


#define ODO_INIT_XPOS 0 
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0


void main(void)
{
    Clock_Init48MHz();
    Motor_Init();
    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);

    while(1)
    {
        

    }

}
