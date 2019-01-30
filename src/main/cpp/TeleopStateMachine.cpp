#include "TeleopStateMachine.h"

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_HATCH_STATION_STATE = 2;
const int GET_HATCH_GROUND_STATE = 3;
const int POST_INTAKE_HATCH_STATE = 4;
const int GET_BALL_STATE = 5;
const int POST_INTAKE_BALL_STATE = 6;
const int PLACE_HATCH_STATE = 7
const int PLACE_BALL_STATE = 8
const int POST_OUTTAKE_STATE = 9;
int state = INIT_STATE

Elevator *elevator;
Intake *intake;
Arm *arm;
Suction *suction;
Solenoids *solenoids;

bool state_intake = false; //set to true to override the states set in the state machine
bool state_arm = false;
bool state_elevator = false;
bool state_suction = false;
bool state_solenoids = false;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_,
    Arm *arm_, Suction *suction_, Solenoids *solenoids_){

    elevator = elevator_;
    intake = intake_;
    arm = arm_;
    suction = suction_;
    solenoids = solenoids_;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool bottom_intake_in,
    bool bottom_intake_out, bool top_intake_in, bool top_intake_out, bool suction_on,
    bool suction_off, bool hatch_out, bool arm_up, bool arm_down,
    bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low,
    bool elevator_ball_up, bool elevator_ball_mid, bool elevator_ball_low,
    bool get_ball, bool get_hatch_ground, bool get_hatch_station, bool post_intake_ball,
    bool post_intake_hatch, bool place_hatch, bool place_ball, bool post_outtake){

  switch (state) {

    case INIT_STATE:
      frc::SmartDashboard::PutString("State", "INIT");
      elevator->elevator_state = elevator->INIT_STATE_H;
      intake->intake_state = intake->INIT_STATE_H;
      arm->arm_state = arm->INIT_STATE_H;
      suction->suction_state = suction->INIT_STATE_H;
      solenoids->solenoids_state = solenoids->INIT_STATE_H;
      break;

    case WAIT_FOR_BUTTON_STATE:
      frc::SmartDashboard::PutString("State", "WAIT FOR BUTTON");
      if (get_hatch_station) {
        state = GET_HATCH_STATION_STATE;
      } else if (get_hatch_ground) {
        state = GET_HATCH_GROUND_STATE;
      } else if (post_intake_hatch) {
        state = POST_INTAKE_HATCH_STATE
      } else if (get_ball) {
        state = GET_BALL_STATE;
      } else if (post_intake_ball) {
        state = POST_INTAKE_BALL_STATE;
      } else if (place_hatch) {
        state = PLACE_HATCH_STATE;
      } else if (place_ball) {
        state = PLACE_BALL_STATE;
      } else if (post_outtake) {
        state = POST_OUTTAKE_STATE;
      }
      break;

    case GET_HATCH_STATION_STATE:
      frc::SmartDashboard::PutString("State", "GET HATCH STATION");
      if (state_elevator){
        elevator->elevator_state = elevator->DOWN_STATE_H;
      }
      if (state_arm) {
        arm->arm_state = arm->UP_STATE_H;
      }
      if (state_suction) {
        suction->suction_state = suction->ON_STATE_H;
      }
      if (state_solenoids) {
        solenoids->solenoids_state = solenoids->OUT_STATE_H;
      }
      if (intake->HaveHatch() || post_intake) {
        state = POST_INTAKE_HATCH_STATE;
      }
      break;

    case GET_HATCH_GROUND_STATE:
      frc::SmartDashboard::PutString("State", "GET HATCH GROUND");
      if (state_elevator) {
        elevator->elevator_state = elevator->DOWN_STATE_H;
      }
      if (state_arm) {
        arm->arm_state = arm->DOWN_STATE_H;
      }
      if (state_intake) {
        intake->intake_state = intake->BOTTOM_INTAKE_IN_STATE_H;
      }
      if (state_suction) {
        suction->suction_state = suction->ON_STATE_H;
      }
      if (state_solenoids) {
        solenoids->solenoids_state = solenoids->OUT_STATE_H;
      }
      if (intake->HaveHatch() || post_intake) {
        state = POST_INTAKE_HATCH_STATE;
      }
      break;

    case POST_INTAKE_HATCH_STATE:
      frc::SmartDashboard::PutString("State", "POST INTAKE HATCH");
      if (state_elevator) {
        elevator->elevator_state = elevator->DOWN_STATE_H;
      }
      if (state_arm) {
        arm->arm_state = arm->UP_STATE_H;
      }
      if (state_suction) {
        suction->suction_state = suction->ON_STATE_H;
      }
      if (state_solenoids) {
        solenoids->solenoids_state = solenoids->IN_STATE_H;
      }
      if (place_hatch) {
        state = PLACE_HATCH_STATE;
      }
      break;

    case GET_BALL_STATE:
      frc::SmartDashboard::PutString("State", "GET BALL");
      if (state_elevator) {
        elevator->elevator_state = elevator->DOWN_STATE_H;
      }
      if (state_arm) {
        arm->arm_state = arm->DOWN_STATE_H;
      }
      if (state_intake) {
        intake->intake_state = intake->BOTTOM_INTAKE_IN_STATE_H;
        intake->intake_state = intake->TOP_INTAKE_IN_STATE_H;
      }
      if (intake->HaveBall() || post_intake) {
        state = POST_INTAKE_BALL_STATE;
      }
      break;

    case POST_INTAKE_BALL_STATE:
      frc::SmartDashboard::PutString("State", "POST INTAKE BALL");
      if (state_elevator) {
        elevator->elevator_state = elevator->DOWN_STATE_H;
      }
      if (state_arm) {
        arm->arm_state = arm->UP_STATE_H;
      }
      if (state_intake) {
        intake->intake_arm = intake->BOTTOM_INTAKE_IN_STATE_H;
        intake->intake_arm = intake->TOP_INTAKE_IN_STATE_H;
      }
      if (place_ball) {
        state = PLACE_BALL_STATE;
      }
      break;

    case PLACE_HATCH_STATE:
      frc::SmartDashboard::PutString("State", "PLACE HATCH");
      if (state_arm) {
        arm->arm_state = arm->UP_STATE_H;
      }
      if (elevator_hatch_up) {
        elevator->elevator_state = elevator->TOP_HATCH_H;
      } else if (elevator_hatch_mid) {
        elevator->elevator_state = elevator->MID_HATCH_H;
      } else if (elevator_hatch_low) {
        elevator->elevator_state = elevator->BOTTOM_HATCH_H;
      }
      if (state_solenoids) {
        solenoids->solenoids_state = solenoids->OUT_STATE_H
      }
      break;

    case PLACE_BALL_STATE:
      frc::SmartDashboard::PutString("State", "PLACE BALL");
      break;

    case POST_OUTTAKE_STATE:
      frc::SmartDashboard::PutString("State", "POST OUTTAKE");
      break;

  }

}
