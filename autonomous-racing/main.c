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

volatile uint32_t uptime_ms;

//opt3101 stuff
uint32_t Distances[3] = {65536, 65536, 65536};
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
    #ifdef MS_15_UPDATE
        uptime_ms += 15;
    #endif
    #ifdef MS_25_UPDATE
        uptime_ms +=25;
    #endif
    UpdatePosition();

    if(pollDistanceSensor())
    {

        channel = (channel+1)%3;
        OPT3101_StartMeasurementChannel(channel);
    }
    Motor_Run();
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

    uptime_ms = 0;
    SysTick_Init_Ints(SYSTICK_UPDATE, 4);
    EnableInterrupts(); //used for tach i think

    UART0_Init();

    StateMachine_Store_Distances(Distances);

    char uart_command = 'g';

    while(1)
    {
        if((EUSCI_A0->IFG&0x01) != 0)
        {
            uart_command = ((char)(EUSCI_A0->RXBUF));
        }

        if(uart_command == 'g')
        {
            StateMachine_Main_Run();
        }
        else if(uart_command == 's')
        {
            Motor_Set_Target(M_STOP,0,0);
        }
        // Systick handler is currently running
    }

}
