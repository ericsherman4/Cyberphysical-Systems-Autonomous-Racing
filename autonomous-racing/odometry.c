// odometry.c
// Inputs to the system are left and right tachometer counts
// System is called periodically, e.g., every 20 to 40 ms
// Private global variables from the system are position (x,y) and orientation theta
// This system will need calibration to your particular robot, adjust N D W C for your robot
// Runs on the MSP432
// Daniel and Jonathan Valvano
// February 10, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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
#include <stdint.h>
#include "msp.h"
#include <stdio.h>
#include "fixed.h"
#include "odometry.h"
#include "Motor.h"
#include "Tachometer.h"
#include "LaunchPad.h"
#include "bump.h"
#include "clock.h"
#include "blinker.h"

int32_t Robotx,Roboty; // position in 0.0001cm
int32_t Robottheta;    // direction units 2*pi/16384 radians (-pi to +pi)
// e.g., 90 degrees (pi/2 radians) is 4096
// private variables
int32_t Lr;     // LCount*c/n the arc distance traveled by the left wheel (0.0001cm)
int32_t Rr;     // RCount*c/n the arc distance traveled by the right wheel (0.0001cm)
int32_t L;      // distance from left wheel to pivot point
int32_t R;      // distance from right wheel to pivot point
int32_t dtheta; // change in direction 2*pi/16384 radians (-pi to +pi)
int32_t dz;     // change in distance 0.0001


#define w2 (W/2) // used for rounding

void Odometry_Init(int32_t initx, int32_t inity, int32_t initTheta){
  Robotx =initx;
  Roboty = inity;
  Robottheta = initTheta;
}
void Odometry_Update(int32_t LCount, int32_t RCount){
  int32_t L2; 
  int32_t absLr,absRr;
  Lr = (LCount*C)/N;      // 0.0001cm
  Rr = (RCount*C)/N;      // 0.0001cm
  if(Lr>=0){
    absLr = Lr;
  }else{
    absLr = -Lr;
  }
  if(Rr>=0){
    absRr = Rr;
  }else{
    absRr = -Rr;
  }
  if(Lr == Rr){
    dtheta = 0;        // straight
    dz = Lr/2;
  }else if(absLr < absRr){   // turning left?
    if(Lr == 0){       // Lr is 0
      L = 0;           // left wheel stopped
      dtheta = (2608*Rr+w2)/W;
      dz = (dtheta*W+5215)/10430;  // 0.0001cm, approximation
    }else{
      L = (W*Lr)/(Rr - Lr);        // 0.0001cm
      L2 = L/2;
      if(L>0){
        dtheta = (2608*Lr+L2)/L;   // 2608=16384/2/pi
      }else{
        dtheta = (2608*Lr-L2)/L;   // 2608=16384/2/pi
      }
      dz = (dtheta*(L+w2)+2608)/5215;  // 0.0001cm, approximation
//    dtheta = (10000*Lr)/L;        // 0.0001 radians
//    dz = (dtheta*(L+w/2))/20000;  // 0.0001cm, approximation
    }
  }else{
    if(Rr == 0){
      L = 0;          // right wheel stopped
      dtheta = (-2608*Lr+w2)/W;
      dz = (-dtheta*W+5215)/10430;       // 0.0001cm, approximation
    }else{
      R = (W*Rr)/(Lr - Rr);        // 0.001cm
      dtheta = (-2608*Rr)/R;       // 2*pi/16384 radians
      dz = (-dtheta*(R+w2)+2608)/5215; // 0.0001cm, approximation
    }
  }
  //  or  dz = (tan(dtheta/2)*(L+w/2))/10000    (0.0001cm) more accurate
  Robotx = Robotx + (dz*fixed_cos2(Robottheta))/65536;  // 0.0001cm
  Roboty = Roboty + (dz*fixed_sin2(Robottheta))/65536;  // 0.0001cm  first part of move
  Robottheta = Robottheta + dtheta;               // 0.001 radians
  if(Robottheta>=PI)Robottheta=Robottheta-TWOPI;  //-8192 to +8191
  if(Robottheta<-PI)Robottheta=Robottheta+TWOPI;
  Robotx = Robotx + (dz*fixed_cos2(Robottheta))/65536;  // 0.0001cm
  Roboty = Roboty + (dz*fixed_sin2(Robottheta))/65536;  // 0.0001cm  second part of move
}
int32_t Odometry_GetX(void){
  return Robotx;
}
int32_t Odometry_GetY(void){
  return Roboty;
}
int32_t Odometry_GetAngle(void){
  return Robottheta;
}

void Odometry_Get(int32_t *x, int32_t *y, int32_t *theta){
  *x = Robotx;
  *y = Roboty;
  *theta = Robottheta;
}

void Display()
{

}


enum RobotState Action;
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t LastLeftSteps;         // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t TotalLeftSteps;        // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t LastRightSteps;        // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t TotalRightSteps;       // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int32_t MyX,MyY;               // position in 0.0001cm
int32_t MyTheta;               // direction units 2*pi/16384 radians (-pi to +pi)
int32_t Error; // in 0.0001cm or in 2*pi/16384 radians



int32_t abs(int32_t in){ // absolute value
  if(abs>=0) return in;
  return -in;
}
void UpdatePosition(void){
  Tachometer_Get(&LeftTach, &LeftDir, &TotalLeftSteps, &RightTach, &RightDir, &TotalRightSteps);
  LeftSteps = TotalLeftSteps-LastLeftSteps;
  RightSteps = TotalRightSteps-LastRightSteps;
  Odometry_Update(LeftSteps,RightSteps);
  Odometry_Get(&MyX,&MyY,&MyTheta);
  LastLeftSteps = TotalLeftSteps;
  LastRightSteps = TotalRightSteps;
}

void WaitUntilBumperTouched(void){uint32_t data;
  uint32_t count = 10;
  do{// wait for touch
    Blinker_Output(FR_LEFT+FR_RGHT);
    Clock_Delay1ms(200); // debounce
    Blinker_Output(BK_LEFT+BK_RGHT);
    Clock_Delay1ms(200); // debounce
    data = Bump_Read(); // 8 bit switch inputs
    count--;
    if(count==0){
      Display();
      count = 10;
    }
  }while(data==0);
  do{// wait for release
    Blinker_Output(BK_LEFT+FR_RGHT);
    Clock_Delay1ms(50); // debounce
    Blinker_Output(FR_LEFT+BK_RGHT);
    Clock_Delay1ms(50); // debounce
    data = Bump_Read(); // 8 bit switch inputs
    count--;
    if(count==0){
      Display();
      count = 10;
    }
  }while(data);
}
void StopUntilBumperTouched(void){
  Motor_Stop();
  Action = ISSTOPPED;
  Display();
  WaitUntilBumperTouched();
}
void Forward(void){
  Action = GOFORWARD;
  Blinker_Output(FR_LEFT+FR_RGHT);
  Display();
  Motor_Forward(MOTORFAST,MOTORFAST);  // move
}
void HardLeft(void){
  Action = HARDLEFT;
  Blinker_Output(FR_LEFT+BK_LEFT);
  Display();
  Motor_Left(MOTORSLOW,MOTORSLOW);  // left
}
void HardRight(void){
  Action = HARDRIGHT;
  Blinker_Output(FR_RGHT+BK_RGHT);
  Display();
  Motor_Right(MOTORSLOW,MOTORSLOW);  // right
}
void SoftLeft(void){
  Action = SOFTLEFT;
  Blinker_Output(FR_LEFT+BK_LEFT);
  Display();
  Motor_Forward(MOTORSLOW,MOTORFAST);  // left
}
void SoftRight(void){
  Action = SOFTRIGHT;
  Blinker_Output(FR_RGHT+BK_RGHT);
  Display();
  Motor_Forward(MOTORFAST,MOTORSLOW);  // right
}
uint32_t ForwardUntilCrash(void){uint32_t data;
  Forward();
  do{// wait for touch
    data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
  }while(data==0);
  return data; // reason for stopping
}
uint32_t ForwardUntilX(int32_t desiredX){uint32_t data;
  int32_t goal;  // in 0.0001cm
  int32_t lastgoal=abs(desiredX-MyX);  // in 0.0001cm
  int32_t badCount = 10;
  Forward();
  do{// wait for touch or X position
    data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
    Error = desiredX-MyX;   // in 0.0001cm
    goal = abs(Error);      // in 0.0001cm
    if(goal > lastgoal){
      badCount--;
      if(badCount<=0){
        data = 0xFF; // wrong way
      }
    }
    lastgoal = goal;
  }while((data==0)&&(goal>XYTOLERANCE));
  Display();
  return data; // reason for stopping, 0 means success
}
int32_t goal;      // in 0.0001cm
int32_t lastgoal;  // in 0.0001cm
int32_t badCount;
int32_t desiredX,desiredY,desiredTh;
enum OdometryCommand Goal; // stopped
void ForwardUntilXStart(int32_t thedesiredX){
  desiredX = thedesiredX;
  lastgoal=abs(desiredX-MyX);  // in 0.0001cm
  badCount = 10;
  Goal = FORWARDTOX;
  Forward();
}
// true if done or error
// false if still running ok
uint32_t ForwardUntilXStatus(void){uint32_t data;
  data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
  if(data) return data;   // crash
  Error = desiredX-MyX;   // in 0.0001cm
  goal = abs(Error);      // in 0.0001cm
  if(goal > lastgoal){    // missed it, going wrong way??
    badCount--;
    if(badCount<=0){
      return 0xFF; // wrong way
    }
  }
  lastgoal = goal;
  return (goal<XYTOLERANCE); // true if close enough
}
uint32_t ForwardUntilY(int32_t desiredY){uint32_t data;
  int32_t goal;  // in 0.0001cm
  int32_t lastgoal=abs(desiredY-MyY);  // in 0.0001cm
  int32_t badCount = 10;
  Forward();
  do{// wait for touch or Y position
    data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
    Error = desiredY-MyY;   // in 0.0001cm
    goal = abs(Error);      // in 0.0001cm
    if(goal > lastgoal){
      badCount--;
      if(badCount<=0){
        data = 0xFF; // wrong way
      }
    }
    lastgoal = goal;
  }while((data==0)&&(goal>XYTOLERANCE));
  Display();
  return data; // reason for stopping, 0 means success
}
void ForwardUntilYStart(int32_t thedesiredY){
  desiredY = thedesiredY;
  badCount = 10;
  lastgoal=abs(desiredY-MyY);  // in 0.0001cm
  Goal = FORWARDTOY;
  Forward();
}
// true if done or error
// false if still running ok
uint32_t ForwardUntilYStatus(void){uint32_t data;
  data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
  if(data) return data;   // crash
  Error = desiredY-MyY;   // in 0.0001cm
  goal = abs(Error);      // in 0.0001cm
  if(goal > lastgoal){    // missed it, going wrong way??
    badCount--;
    if(badCount<=0){
      return 0xFF; // wrong way
    }
  }
  lastgoal = goal;
  return (goal<XYTOLERANCE);
}

uint32_t SoftLeftUntilTh(int32_t desiredTh){
  uint32_t data;
  int32_t goal;  // in 2*pi/16384 radians
  int32_t badCount = 10;
  int32_t lastgoal=abs(desiredTh-MyTheta);  // in 2*pi/16384 radians
  SoftLeft();
  do{// wait for touch or Th position
    data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
    Error = desiredTh-MyTheta;   // in 2*pi/16384 radians
    if(Error >= PI)Error = Error-TWOPI;  // -8192 to +8191
    if(Error < -PI)Error = Error+TWOPI;  // -8192 to +8191
    goal = abs(Error);           // in 2*pi/16384 radians
    if(goal > lastgoal){         // missed it, going wrong way??
      badCount--;
      if(badCount<=0){
        data = 0xFF; // wrong way
      }
    }
    lastgoal = goal;
  }while((data==0)&&(goal>THETATOLERANCE));
  Display();
  return data; // reason for stopping, 0 means success
}

uint32_t SoftRightUntilTh(int32_t desiredTh)
{
    uint32_t data;
    int32_t goal;  // in 2*pi/16384 radians
    int32_t badCount = 10;
    int32_t lastgoal=abs(desiredTh-MyTheta);  // in 2*pi/16384 radians
    SoftRight();
    do{// wait for touch or Th position
      data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
      Error = desiredTh-MyTheta;   // in 2*pi/16384 radians
      if(Error >= PI)Error = Error-TWOPI;  // -8192 to +8191
      if(Error < -PI)Error = Error+TWOPI;  // -8192 to +8191
      goal = abs(Error);           // in 2*pi/16384 radians
      if(goal > lastgoal){         // missed it, going wrong way??
        badCount--;
        if(badCount<=0){
          data = 0xFF; // wrong way
        }
      }
      lastgoal = goal;
    }while((data==0)&&(goal>THETATOLERANCE));
    Display();
    return data; // reason for stopping, 0 means success
}

void SoftLeftUntilThStart(int32_t thedesiredTh){
  desiredTh = thedesiredTh;
  badCount = 10;
  lastgoal=abs(thedesiredTh-MyTheta);  // in 2*pi/16384 radians
  Goal = LEFTTOTH;
  SoftLeft();
}

void SoftRightUntilThStart(int32_t thedesiredTh)
{
  desiredTh = thedesiredTh;
  badCount = 10;
  lastgoal=abs(thedesiredTh-MyTheta);  // in 2*pi/16384 radians
  Goal = RIGHTTOTH;
  SoftRight();
}


// true if done or error
// false if still running ok
uint32_t  ForwardUntilThStatus(void){uint32_t data;
  data = Bump_Read()+(LaunchPad_Input()<<6); // 8 bit switch inputs
  if(data) return data;   // crash
  Error = desiredTh-MyTheta;   // in 2*pi/16384 radians
  if(Error >= PI)Error = Error-TWOPI;  // -8192 to +8191
  if(Error < -PI)Error = Error+TWOPI;  // -8192 to +8191
  goal = abs(Error);           // in 2*pi/16384 radians
  if(goal > lastgoal){         // missed it, going wrong way??
    badCount--;
    if(badCount<=0){
      return 0xFF; // wrong way
    }
  }
  lastgoal = goal;
  return (goal<THETATOLERANCE);
}

uint32_t CheckGoal(void){
  if(Goal==STOP){
    Motor_Stop(); return 1;
  }
  if(Goal==FORWARDTOX){
    return ForwardUntilXStatus();
  }
  if(Goal==FORWARDTOY){
    return ForwardUntilYStatus();
  }
  if(Goal==LEFTTOTH){
    return ForwardUntilThStatus();
  }
  return 0;
}

