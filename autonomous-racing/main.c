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

// https://coder-tronics.com/state-machine-tutorial-pt2/


states_e curr_state = BEGIN;
int32_t target_heading = 0;
uint32_t command_status = 0;

#define NX_STATE(val) next_state=(val); break;
uint32_t ramp;
uint8_t consistent_right = 0;
uint8_t consistent_left = 0;

void upon_entry(states_e state)
{
    switch(state)
    {
        case S_HALLWAY1_STR:
            LaunchPad_Output(GREEN);
            target_heading = 0;
            ForwardUntilXStart(DISTANCE_1FT, S_HALLWAY1_STR);
            consistent_left = 0;
            consistent_right = 0;
            break;
        case S_HALLWAY1_ALIGN:
            LaunchPad_Output(BLUE);
            target_heading =0;
            // turning left, mytheta goes up so if mytheta > target, then we are facing too far to the left
            // so we want to turn to the right
            if(MyTheta > target_heading)
            {
                // turn right
                SoftRightUntilThStart(target_heading);
            }
            else
            {
                // turn left
                SoftLeftUntilThStart(target_heading);
            }
            break;
        case S_HALLWAY1_TO2:
            LaunchPad_Output(PINK);
            target_heading = -4096;
            HardRightUntilThStart(target_heading);
            break;
        case S_HALLWAY2_ALIGN:
            LaunchPad_Output(BLUE);
            if(MyTheta > target_heading)
            {
                SoftRightUntilThStart(target_heading);
            }
            else
            {
                SoftLeftUntilThStart(target_heading);
            }
            break;
        case S_HALLWAY2_STR:
            target_heading = -4096;
            LaunchPad_Output(GREEN);
            #warning "These are wrong!! you keep calling forwarduntilystart but its already past this distance so it probably errors right away"
            // but maybe not because it does stay in green...
            ForwardUntilYStart(DISTANCE_1FT, S_HALLWAY2_STR);
            consistent_left = 0;
            consistent_right = 0;
            break;
        case S_HALLWAY2_TO3:
            target_heading = -8192;
            LaunchPad_Output(PINK);
            HardRightUntilThStart(target_heading);
            break;
        case S_HALLWAY3_STR:
            target_heading = 0;
            LaunchPad_Output(GREEN);
            ForwardUntilXStart(DISTANCE_1FT, S_HALLWAY3_STR);
            consistent_left = 0;
            consistent_right = 0;
            break;
        case S_HALLWAY3_FIRST_ALIGN:
            LaunchPad_Output(YELLOW);
            if(MyTheta > target_heading)
            {
                SoftRightUntilThStart(target_heading);
            }
            else
            {
                SoftLeftUntilThStart(target_heading);
            }
            break;
        case S_HALLWAY3_ALIGN:
            target_heading = 0;
            LaunchPad_Output(BLUE);
            if(MyTheta > target_heading)
            {
                SoftRightUntilThStart(target_heading);
            }
            else
            {
                SoftLeftUntilThStart(target_heading);
            }
            break;
        case S_STOP:
            LaunchPad_Output(RED);
            Motor_Stop();
            break;
        
    }
}

void upon_exit(states_e state)
{
    switch(state)
    {
        case S_HALLWAY1_TO2:
            Odometry_Init(MyX, 0, MyTheta);
            break;
        case S_HALLWAY2_TO3:
            Odometry_Init(0, 0, 0);
            break;
        // case S_HALLWAY3_FIRST_ALIGN:

    }

}

// void set_target_heading(states_e next_state)
// {
//     // set target heading
//     switch(next_state)
//     {
//         case S_HALLWAY1_STR:
//         case S_HALLWAY1_ALIGN:
//             target_heading = 0;
//             break;
//         case S_HALLWAY1_TO2:
//         case S_HALLWAY2_STR:
//         case S_HALLWAY2_ALIGN:
//             target_heading = -(16384 >> 2) + 100; //equiv to divide by 4
//         //TODO: finish
//     }
// }



void StateMachine_Main_Run()
{
    // if nothing happens, stay in state
    states_e next_state = curr_state;
    
    switch(curr_state)
    {
        case BEGIN:
            // default switch from begin to hallway1 straight so you can run its entry command

            for(ramp=4000; ramp <= MOTORFAST; ramp+=100)
            {
                Motor_Forward(ramp,ramp);
                Clock_Delay1ms(50);
            }
            NX_STATE(S_HALLWAY2_STR);
        
        case S_HALLWAY1_STR:
            
            // check for obstacles

            if(Distances[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }

            consistent_left = (Distances[0] < 900) ? consistent_left+1 : 0;
            consistent_right = (Distances[2] < 900) ? consistent_right+1 : 0;

            if(consistent_right == 4)
            {
                Odometry_Init(MyX, MyY, -100);
                consistent_right = 0;
            }
            else if(consistent_left == 4)
            {
                //approaching left wall but robot thinks theta is center
                // make the robot think heading is further to the left than it is so the robot will push right
                Odometry_Init(MyX, MyY, 100);
                consistent_left = 0;
            }


            // Check on X loc
//            if(MyX > (DISTANCE_1FT*20))
             if (MyX > (DISTANCE_1FT*91))
            {
                Motor_Stop();
                NX_STATE(S_HALLWAY1_TO2);
            }

            // check on status of going straight
            command_status = ForwardUntilXStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Stop();
                // check if heading is too far off course
                if(MyTheta > (300 + target_heading) || MyTheta < (target_heading - 300))
                {
                    NX_STATE(S_HALLWAY1_ALIGN);
                }
                else
                {
                    //stay in state 
                    // call entry functionality again to restart state
                    upon_entry(curr_state);
                    NX_STATE(curr_state);
                }
            }
            else if(command_status > 1)
            {
                // error occurred 
                // Motor_Stop();
                NX_STATE(S_HALLWAY1_ALIGN);
            }
            else
            {
                // status is 0 meaning it is still running
                NX_STATE(curr_state);
            }
            

        case S_HALLWAY1_ALIGN:
            // check on turning status
            command_status = ForwardUntilThStatus();

            if(command_status == 1)
            {
                //success, we are realigned, go back to driving forward
                NX_STATE(S_HALLWAY1_STR);
            }
            else if(command_status > 1)
            {
                // failed. rerun the state in case we turned to far.
                // heading angles will redetermine what direction to turn in
                upon_entry(curr_state);
                NX_STATE(curr_state);
            }
            else
            {
                //status is 0, still turning
                NX_STATE(curr_state);
            }

        case S_HALLWAY1_TO2:
            command_status = ForwardUntilThStatus();

            if(command_status == 1)
            {
                //sucess
                Motor_Stop();
                NX_STATE(S_HALLWAY2_STR);
            }
            else if(command_status > 1)
            {
                // failed might have turned too far
                // go to align
                NX_STATE(S_HALLWAY2_ALIGN);
            }
            else
            {
                //still running, stay in state
                NX_STATE(curr_state);
            }

        case S_HALLWAY2_ALIGN:
            command_status = ForwardUntilThStatus();
            if(command_status == 1)
            {
                //sucess, we are aligned
                NX_STATE(S_HALLWAY2_STR);
            }
            else if(command_status > 1)
            {
                // failed, align again
                upon_entry(curr_state);
                NX_STATE(curr_state);
            }
            else
            {
                //still running
                NX_STATE(curr_state);
            }

        case S_HALLWAY2_STR:

            // check for obstacles
            if(Distances[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }


            // Check on Y loc
            if(MyY < (-(DISTANCE_1FT*18))) //placed robot facing mechanical closet, with robot aligned with back of the bench
            // if (MyY < (-(DISTANCE_1FT*122))) // THIS DISTANCE IS PERF, aligned with left side of the bathroom door? 
            {
                Motor_Stop();
                NX_STATE(S_HALLWAY2_TO3);
            }


            // if(MyY < (-(DISTANCE_1FT*95)))
            if(MyY < -(DISTANCE_1FT*5))
            {
                consistent_left = (Distances[0] < 300) ? (consistent_left+1) : 0;
                consistent_right = (Distances[2] < 300) ? (consistent_right+1) : 0;
            }
            else
            {
                consistent_left = (Distances[0] < 900) ? (consistent_left+1) : 0;
                consistent_right = (Distances[2] < 900) ? (consistent_right+1) : 0;
            }

            if(consistent_right == 4)
            {
                // make the robot think its more turned right than it is
                Odometry_Init(MyX, MyY, -4096 -100);
                consistent_right = 0;
            }

            if(consistent_left == 4)
            {
                // turn right a bit
                Odometry_Init(MyX, MyY, -4096 +100);
                consistent_left = 0;
            }

            
            // check on status of going straight
            command_status = ForwardUntilYStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Stop();
                // check if heading is too far off course
                if(MyTheta > (300 + target_heading) || MyTheta < (target_heading - 300))
                {
                    NX_STATE(S_HALLWAY2_ALIGN);
                }
                else
                {
                    //stay in state 
                    // call entry functionality again to restart state
                    upon_entry(curr_state);
                    NX_STATE(curr_state);
                }
            }
            else if(command_status > 1)
            {
                // error occurred 
                // Motor_Stop();
                NX_STATE(S_HALLWAY2_ALIGN);
            }
            else
            {
                // status is 0 meaning it is still running
                NX_STATE(curr_state);
            }

        case S_HALLWAY2_TO3:
            command_status = ForwardUntilThStatus();

            if(MyTheta < -7900)
            {
                Odometry_Init(MyX, MyY, 0); //if you reset theta to zero then x will increase in a positive direction
                NX_STATE(S_HALLWAY3_ALIGN);
            }

            if(command_status == 1)
            {
                //sucess
                Motor_Stop();
                NX_STATE(S_HALLWAY3_STR);
            }
            else if(command_status > 1)
            {
                // failed might have turned too far
                // go to align
                NX_STATE(S_HALLWAY3_ALIGN);
            }
            else
            {
                //still running, stay in state
                NX_STATE(curr_state);
            }

        case S_HALLWAY3_ALIGN:
            command_status = ForwardUntilThStatus();
            if(command_status == 1)
            {
                //sucess, we are aligned
                NX_STATE(S_HALLWAY3_STR);
            }
            else if(command_status > 1)
            {
                // failed, align again
                upon_entry(curr_state);
                NX_STATE(curr_state);
            }
            else
            {
                //still running
                NX_STATE(curr_state);
            }

        case S_HALLWAY3_STR:
            // check for obstacles
            if(Distances[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }


            // Check on X loc
            if (MyX > ((DISTANCE_1FT*93)))
            {
                Motor_Stop();
                NX_STATE(S_STOP);
            }

            consistent_left = (Distances[0] < 200) ? (consistent_left+1) : 0;
            consistent_right = (Distances[2] < 200) ? (consistent_right+1) : 0;

            if(consistent_right == 4)
            {
                // make the robot think its more turned right than it is
                Odometry_Init(MyX, MyY, 200);
                consistent_right = 0;
            }

            #warning "YOU NEED TO ADD A CHECK ONCE U REACH END OF HALLWAY 2 BECAUS THE THINGS ON THE RIGHT PUSH U INTO THE SERVICE ELEVATOR"

            if(consistent_left == 4)
            {
                // turn right a bit
                Odometry_Init(MyX, MyY, 200);
                consistent_left = 0;
            }

            
            // check on status of going straight
            command_status = ForwardUntilXStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Stop();
                // check if heading is too far off course
                if(MyTheta > (300 + target_heading) || MyTheta < (target_heading - 300))
                {
                    NX_STATE(S_HALLWAY3_ALIGN);
                }
                else
                {
                    //stay in state 
                    // call entry functionality again to restart state
                    upon_entry(curr_state);
                    NX_STATE(curr_state);
                }
            }
            else if(command_status > 1)
            {
                // error occurred 
                // Motor_Stop();
                NX_STATE(S_HALLWAY3_ALIGN);
            }
            else
            {
                // status is 0 meaning it is still running
                NX_STATE(curr_state);
            }



        case S_STOP:
            // dead hang
            NX_STATE(curr_state);
    }

    
    if(next_state !=  curr_state)
    {
        // set_target_heading(next_state);
        upon_exit(curr_state);
        upon_entry(next_state);
    }
    
    curr_state = next_state;
}

void StateMachine_AO_Run()
{

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
