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
#include "opt3101.h"
#include "pid.h"




#define COUNT_RESET 50

uint8_t count = COUNT_RESET;
uint8_t state = 0;

//opt3101 stuff
uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}



void SysTick_Handler()
{
    UpdatePosition();
//    if(--count == 0)
//    {
//        count = COUNT_RESET;
//        state = !state;
//        if(state)
//            LaunchPad_Output(PINK);
//        else
//            LaunchPad_Output(GREEN);
//    }

}

// https://coder-tronics.com/state-machine-tutorial-pt2/
enum states { S_HALLWAY1_STR, S_HALLWAY1_ALIGN, S_HALLWAY1TO2, S_NAME3};
uint8_t curr_state = S_HALLWAY1_STR;
int32_t target_heading = 0;

#warning "JUST GO BETWEEN STRAIGHT AND THEN DRIVING UNTIL HEADING 0"

void StateMachine_Main_Run()
{
    // if nothing happens, stay in state
    int next_state = curr_state;
    
    switch(curr_state)
    {
        case S_HALLWAY1_STR:
            // go straight for x amount
            // hug left wall
            if(Distances[1] < 400)
            {
                //run A0 machine
            }

            #warning "make sure this variable is updating correctly"
            if(MyX > HALLWAY1_X_MIN && MyX < HALLWAY1_X_MAX)
            {
                next_state = S_HALLWAY1TO2;
            }

            break;

        case S_HALLWAY1TO2:

            break;
    }

    // Some code here if entry or exit functions need to be called.
    
    curr_state = next_state;

}

void StateMachine_AO_Run()
{

}



void main(void)
{

    //next step, PID to go straight
    // thing is we cant always rely on perfectly straight heading
    // so we will need the walls to correct us sometimes,
    // in certain hallways we wanna use certain walls though


    Clock_Init48MHz();
    uint32_t channel = 1;
    Distance_Sensor_Init(1);

    Motor_Init();
    LaunchPad_Init();
    LaunchPad_LED(0);

    Bump_Init();
    Tachometer_Init();
    Blinker_Init();
    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);


    SysTick_Init_Ints(ODO_UPDATE_PERIOD, 4);
    EnableInterrupts(); //used for tach i think

    ForwardUntilXStart(DISTANCE_1FT);

    //while its still running
    uint32_t result = 0;
    while(!result)
    {
        result = ForwardUntilXStatus();
        
        if(pollDistanceSensor())
        {

            channel = (channel+1)%3;
            OPT3101_StartMeasurementChannel(channel);

            if(Distances[0] < 700)
            {
                Motor_Forward(MOTORFAST-2000, MOTORFAST +2000);
                LaunchPad_Output(GREEN);
            }
            else if(Distances[2] < 700)
            {
                Motor_Forward(MOTORFAST+2000, MOTORFAST - 2000);
                LaunchPad_Output(PINK);
            }
            else{
                Motor_Forward(MOTORFAST, MOTORFAST);
                LaunchPad_Output(BLUE);
            }
        }
    }

    if(result == 1)
    {
        LaunchPad_LED(1);
    }



    Motor_Stop();


    while(1)
    {
        // Systick handler is currently running


    }

}
