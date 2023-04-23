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


#define ODO_INIT_XPOS 0 
#define ODO_INIT_YPOS 0
#define ODO_INIT_HEADING 0
#define DISTANCE_1FT 304800

// 1/48/10^6*700000 is 15ms update
// 1/48/10^6*1200000 is 25ms update
//1/48/10^6*50000000 is 1.04s so divide by 10 is 100ms
#define ODO_UPDATE_PERIOD 1200000
#warning "see if wheel counts are between 5 and 20"

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

void UartSetCur(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec

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

void Distance_Sensor_Init(uint32_t start_channel)
{
    I2CB1_Init(30);
    Init();
    Clear();
    OutString("OPT3101");
    SetCursor(0, 1);
    OutString("Left =");
    SetCursor(0, 2);
    OutString("Centr=");
    SetCursor(0, 3);
    OutString("Right=");
    SetCursor(0, 4);
    OutString("Busy-wait");
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_StartMeasurementChannel(start_channel);
}

void main(void)
{

    //next step, PID to go straight
    // thing is we cant always rely on perfectly straight heading


    Clock_Init48MHz();
    Motor_Init();
    LaunchPad_Init();
    LaunchPad_LED(0);
    Bump_Init();
    Tachometer_Init();
    Blinker_Init();
    Odometry_Init(ODO_INIT_XPOS, ODO_INIT_YPOS, ODO_INIT_HEADING);

    uint32_t channel = 1;
    Distance_Sensor_Init(channel);




    SysTick_Init_Ints(ODO_UPDATE_PERIOD, 4);
    EnableInterrupts();

    ForwardUntilXStart(DISTANCE_1FT*5);

    //while its still running
    uint32_t result = 0;
    while(!result)
    {
        result = ForwardUntilXStatus();
        
        if(pollDistanceSensor())
        {
            if(TxChannel <= 2)
            {
                SetCursor(6, TxChannel+1);
                OutUDec(Distances[TxChannel]);
            }
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
