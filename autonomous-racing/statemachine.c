#include "statemachine.h"

#include "odometry.h"
#include "motor.h"
#include "Launchpad.h"
#include "Clock.h"


// https://coder-tronics.com/state-machine-tutorial-pt2/

extern uint32_t uptime_ms;

uint32_t start_time;

uint32_t * Distances_local;


states_e curr_state = BEGIN;
states_e pre_crash_state = BEGIN;
int32_t target_heading = 0;
uint32_t command_status = 0;

uint32_t ramp;
uint8_t consistent_right = 0;
uint8_t consistent_left = 0;
uint8_t consistent_front = 0;

bool reached_end_hw2 = false;

void StateMachine_Get_State(states_e *state)
{
    *state = curr_state;
}

void StateMachine_Store_Distances(uint32_t * distances)
{
    Distances_local = distances;
}


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
            consistent_front = 0;
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
        case S_HALLWAY2_STR_END:
            target_heading = -4096;
            LaunchPad_Output(YELLOW);
            ForwardUntilYStart(DISTANCE_1FT, S_HALLWAY2_STR_END);
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
        case S_CRASH:
            LaunchPad_Output(0);
            start_time = uptime_ms;
            Motor_Set_Target(M_STOP, 0, 0);
            break;
        case S_STOP:
            LaunchPad_Output(RED);
            Motor_Set_Target(M_STOP, 0, 0);
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



void StateMachine_Main_Run()
{
    // if nothing happens, stay in state
    states_e next_state = curr_state;
    
    switch(curr_state)
    {
        case BEGIN:
            // default switch from begin to hallway1 straight so you can run its entry command

            NX_STATE(STATE_AFTER_BEGIN);
        
        case S_HALLWAY1_STR:
            
            // check for obstacles

            if(Distances_local[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }

            consistent_left = (Distances_local[0] < 900) ? consistent_left+1 : 0;
            consistent_right = (Distances_local[2] < 900) ? consistent_right+1 : 0;

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

            consistent_front = (Distances_local[1] < 950) ? (consistent_front+1) : 0;

            // Check on X loc
        //    if(MyX > (DISTANCE_1FT*10) && consistent_front == 3)
             if (HALLWAY1_STR_CHK && (consistent_front == 1))
            {
                Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY1_TO2);
            }

            // check on status of going straight
            command_status = ForwardUntilXStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Set_Target(M_STOP, 0, 0);
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
            else if(command_status > 1 && command_status < 255)
            {
                // we crashed
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
            {
                //going wrong directon
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
            else if(command_status > 1 && command_status < 255)
            {
                // we crashed
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
                
            }
            else if(command_status == 255)
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
                Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY2_STR);
            }
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
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
                if(reached_end_hw2)
                {
                    NX_STATE(S_HALLWAY2_STR_END);
                }
                else
                {
                    NX_STATE(S_HALLWAY2_STR);
                }
            }
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
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
            if(Distances_local[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }

            if(HALLWAY2_END_CHK)
            // if(MyY < -(DISTANCE_1FT*5))
            {
                reached_end_hw2 = true;
            }

            consistent_left = (Distances_local[0] < 900) ? (consistent_left+1) : 0;
            consistent_right = (Distances_local[2] < 900) ? (consistent_right+1) : 0;


            if(consistent_right == 4)
            {
                // make the robot think its more turned right than it is
                Odometry_Init(MyX, MyY, -4096 -200);
                // consistent_right = 0;
            }

            if(consistent_left == 4)
            {
                // turn right a bit
                Odometry_Init(MyX, MyY, -4096 +200);
                // consistent_left = 0;
            }

            
            // check on status of going straight
            command_status = ForwardUntilYStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Set_Target(M_STOP, 0, 0);
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
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
            {
                // error occurred 
                // Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY2_ALIGN);
            }
            else
            {
                // status is 0 meaning it is still running
                NX_STATE(curr_state);
            }

        case S_HALLWAY2_STR_END:

            // check for obstacles
            if(Distances_local[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }


            // Check on Y loc
            // if(MyY < (-(DISTANCE_1FT*18))) //placed robot facing mechanical closet, with robot aligned with back of the bench
            if (HALLWAY2_STR_END_CHK) // THIS DISTANCE IS PERF, aligned with left side of the bathroom door? 
            {
                Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY2_TO3);
            }


            consistent_left = (Distances_local[0] < 500) ? (consistent_left+1) : 0;
            consistent_right = (Distances_local[2] < 500) ? (consistent_right+1) : 0;


            if(consistent_right == 4)
            {
                // make the robot think its more turned right than it is
                Odometry_Init(MyX, MyY, -4096 -200);
                consistent_right = 0;
            }

            if(consistent_left == 4)
            {
                // turn right a bit
                Odometry_Init(MyX, MyY, -4096 +200);
                consistent_left = 0;
            }

            
            // check on status of going straight
            command_status = ForwardUntilYStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Set_Target(M_STOP, 0, 0);
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
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
            {
                // error occurred 
                // Motor_Set_Target(M_STOP, 0, 0);
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
                Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY3_STR);

            }
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
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
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
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
            if(Distances_local[1] < 300)
            {
                LaunchPad_LED(1);
            }
            else
            {
                LaunchPad_LED(0);
            }


            // Check on X loc
            if (HALLWAY3_END_CHK)
            {
                Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_STOP);
            }

            consistent_left = (Distances_local[0] < 700) ? (consistent_left+1) : 0;
            consistent_right = (Distances_local[2] < 700) ? (consistent_right+1) : 0;

            if(consistent_right == 3)
            {
                // make the robot think its more turned right than it is
                Odometry_Init(MyX, MyY, -200);
                // consistent_right = 0;
            }

            if(consistent_left == 3)
            {
                // turn right a bit
                Odometry_Init(MyX, MyY, 200);
                // consistent_left = 0;
            }

            
            // check on status of going straight
            command_status = ForwardUntilXStatus();
            
            if(command_status == 1)
            {
                // success
                // Motor_Set_Target(M_STOP, 0, 0);
                // check if heading is too far off course
                if(MyTheta > (100 + target_heading) || MyTheta < (target_heading - 100))
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
            else if(command_status > 1 && command_status < 255)
            {
                pre_crash_state = curr_state;
                NX_STATE(S_CRASH);
            }
            else if(command_status == 255)
            {
                // error occurred 
                // Motor_Set_Target(M_STOP, 0, 0);
                NX_STATE(S_HALLWAY3_ALIGN);
            }
            else
            {
                // status is 0 meaning it is still running
                NX_STATE(curr_state);
            }


        case S_CRASH:
        
            if((uptime_ms - start_time) >= 1000)
            {
                NX_STATE(pre_crash_state);
            }
            else
            {
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
