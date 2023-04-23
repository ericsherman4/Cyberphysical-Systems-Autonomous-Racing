//*****************************************************************************
// test main for Robot with OPT3101 time of flight sensor
// MSP432 with RSLK Max and OPT3101
// Daniel and Jonathan Valvano
// July 6, 2020
//****************************************************************************
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2020, Jonathan Valvano, All rights reserved.

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
// see opt3101.h for OPT3101 hardware connections
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections

#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "I2CB1.h"
#include "CortexM.h"
#include "opt3101.h"
#include "LaunchPad.h"
#include "UART0.h"
#include "Motor.h"

void UartSetCur_2(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear_2(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear_2
#define SetCursor UartSetCur_2
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec

bool pollDistanceSensor_2(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}


void main_unused(void)
{ // busy-wait implementation
  uint32_t channel = 1;
  Clock_Init48MHz();
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
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
  OPT3101_StartMeasurementChannel(channel);
  StartTime = SysTick->VAL;
  Motor_Init();

  while(1)
  {
    if(pollDistanceSensor_2())
    {
      TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
      if(TxChannel <= 2)
      {
        SetCursor(6, TxChannel+1);
        OutUDec(Distances[TxChannel]);
      }
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      StartTime = SysTick->VAL;
    }

    if(Distances[1] <200)
    {
        Motor_Stop();
    }
    else
    {
        Motor_Forward(3000,3000);
    }

  }
}
