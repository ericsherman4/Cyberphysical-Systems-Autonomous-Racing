#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "UART0.h"
#include "Odometry.h"
#include "SysTick.h"
#include "bump.h"


#define ODO_INIT_XPOS 0
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0


void main(void)
{
    Clock_Init48MHz();
    LaunchPad_Init();
    Motor_Init();
    SysTick_Init();
    Bump_Init();
    BumpInt_Init();
    UART0_Init();

    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);
    uint32_t right_PWM = 1000;
    uint32_t left_PWM = 1000;

    float param_I = 10.0;
    float param_D = 10.0;
    float param_P = 10.0;

    while(1)
    {
        //Motor control commands
        char c = UART0_InChar();
        if(c == 12){ //C character for forward
        Motor_Forward(left_PWM, right_PWM);
        }
        if(c == 11){ //B character for backwards
            Motor_Backward(left_PWM, right_PWM);
        }
        if(c == 10){ //A character for stopping
            Motor_Stop();
        }

        //PID tuning commands (imagine using a switch case, couldnt be me)
        if(c==0x87){ //P is decreased by a factor of 10
            param_P = param_P/10;
        }
        if(c==0x89){ //P is increased by a factor of 10
            param_P = param_P*10;
        }
        if(c==0x54){ //I is decreased by a factor of 10
            param_I = param_I/10;
        }
        if(c==0x56){ //I is increased by a factor of 10
            param_I = param_I*10;
        }
        if(c==0x21){ //D is decreased by a factor of 10
            param_D = param_D/10;
        }
        if(c==0x23){ //D is increased by a factor of 10
            param_D = param_D*10;
        }
        delay(200);
    }

}
