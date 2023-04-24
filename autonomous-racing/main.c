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
#include "statemachine.h"


//opt3101 stuff
uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
uint32_t channel = 1;

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

    if(pollDistanceSensor())
    {

        channel = (channel+1)%3;
        OPT3101_StartMeasurementChannel(channel);

    }
}

void main(void)
{

    Clock_Init48MHz();
    
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


    while(1)
    {
        // Systick handler is currently running
        StateMachine_Main_Run();
    }

}
