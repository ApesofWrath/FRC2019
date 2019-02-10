#include "TeleopStateMachine.h"

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_HATCH_STATION_STATE = 2;
const int GET_HATCH_GROUND_STATE = 3;
const int POST_INTAKE_HATCH_STATE = 4;
const int GET_CARGO_STATE = 5;
const int POST_INTAKE_CARGO_STATE = 6;
const int PLACE_HATCH_STATE = 7;
const int PLACE_CARGO_STATE = 8;
const int POST_OUTTAKE_HATCH_STATE = 9;
const int POST_OUTTAKE_CARGO_STATE = 10;
int state = INIT_STATE;

int last_state = 0;

Elevator *elevator;
Intake *intake;
Arm *arm;
HatchPickup *hatch_pickup;

bool state_top_intake = false; //set to true to override the states set in the state machine
bool state_bottom_intake = false;
bool state_arm = false;
bool state_elevator = false;
bool state_suction = false;
bool state_solenoids = false;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_,
    Arm *arm_, HatchPickup *hatch_pickup_){

    elevator = elevator_;
    intake = intake_;
    arm = arm_;
    hatch_pickup = hatch_pickup_;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool bottom_intake_in,
    bool bottom_intake_out, bool bottom_intake_stop, bool top_intake_in, bool top_intake_out,
    bool top_intake_stop, bool suction_on, bool suction_off, bool hatch_out, bool hatch_in, bool arm_up,
    bool arm_down, bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low,
    bool elevator_cargo_up, bool elevator_cargo_mid, bool elevator_cargo_low, bool get_cargo,
    bool get_hatch_ground, bool get_hatch_station, bool post_intake_cargo, bool post_intake_hatch,
    bool place_hatch, bool place_cargo, bool post_outtake_hatch, bool post_outtake_cargo) {

    arm->IsElevatorHigh(elevator->IsElevatorHigh()); //diff arm upper soft limit
    elevator->IsArmBack(arm->GetAngularPosition());

    if (wait_for_button) {
      state = WAIT_FOR_BUTTON_STATE;
    }

    //top intake
    if (top_intake_in) {
  		state_top_intake = false;
  		intake->top_intake_state = intake->IN_STATE_H;
  	} else if (top_intake_out) {
  		state_top_intake = false;
  		intake->top_intake_state = intake->OUT_STATE_H;
  	} else if (top_intake_stop) {
      state_top_intake = false;
      intake->top_intake_state = intake->STOP_STATE_H;
    } else {
      state_top_intake = false;
    }

    //bottom intake
    if (bottom_intake_in) {
      state_bottom_intake = false;
  		intake->bottom_intake_state = intake->IN_STATE_H;
    } else if (bottom_intake_out) {
      state_bottom_intake = false;
  		intake->bottom_intake_state = intake->OUT_STATE_H;
    } else if (bottom_intake_stop) {
  		state_bottom_intake = false;
  		intake->bottom_intake_state = intake->STOP_STATE_H;
  	} else {
  		state_bottom_intake = true;
  	}

    //arm
  	if (arm_up) {
  		state_arm = false;
  		arm->arm_state = arm->UP_STATE_H;
  	} else if (arm_down) {
  		state_arm = false;
  		arm->arm_state = arm->DOWN_STATE_H;
  	} else {
  		state_arm = true;
  	}

    //elevator
  	if (elevator_hatch_up) {
  		state_elevator = false;
  		elevator->elevator_state = elevator->TOP_HATCH_STATE_H;
  	} else if (elevator_hatch_mid) {
  		state_elevator = false;
  		elevator->elevator_state = elevator->MID_HATCH_STATE_H;
  	} else if (elevator_hatch_low) {
  		state_elevator = false;
  		elevator->elevator_state = elevator->BOTTOM_CARGO_STATE_H;
  	} else if (elevator_cargo_up) {
  		state_elevator = false;
      elevator->elevator_state = elevator->TOP_CARGO_STATE_H;
  	} else if (elevator_cargo_mid) {
      state_elevator = false;
      elevator->elevator_state = elevator->MID_CARGO_STATE_H;
    } else if (elevator_cargo_low) {
      state_elevator = false;
      elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
    } else {
      state_elevator = true;
    }

    //suction
    if (suction_on) {
      state_suction = false;
      hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
    } else if (suction_off) {
      state_suction = false;
      hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
    } else {
      state_suction = true;
    }

    //solenoids
    if (hatch_out) {
      state_solenoids = false;
      hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
    } else if (hatch_in) {
      state_solenoids = false;
      hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
    } else {
      state_solenoids = true;
    }

  switch (state) {

    case INIT_STATE:

      frc::SmartDashboard::PutString("State", "INIT");

      elevator->elevator_state = elevator->INIT_STATE_H;
      intake->top_intake_state = intake->STOP_STATE_H;
      intake->bottom_intake_state = intake->STOP_STATE_H;
      arm->arm_state = arm->INIT_STATE_H;
      hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
      hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
      state = WAIT_FOR_BUTTON_STATE;
      last_state = INIT_STATE;
      break;

    case WAIT_FOR_BUTTON_STATE:

      frc::SmartDashboard::PutString("State", "WAIT FOR BUTTON");

      if (get_hatch_station) {
        state = GET_HATCH_STATION_STATE;
      } else if (get_hatch_ground) {
        state = GET_HATCH_GROUND_STATE;
      } else if (post_intake_hatch) {
        state = POST_INTAKE_HATCH_STATE;
      } else if (get_cargo) {
        state = GET_CARGO_STATE;
      } else if (post_intake_cargo) {
        state = POST_INTAKE_CARGO_STATE;
      } else if (place_hatch) {
        state = PLACE_HATCH_STATE;
      } else if (place_cargo) {
        state = PLACE_CARGO_STATE;
      } else if (post_outtake_hatch) {
        state = POST_OUTTAKE_HATCH_STATE;
      } else if (post_outtake_cargo) {
        state = POST_OUTTAKE_CARGO_STATE;
      }
      last_state = WAIT_FOR_BUTTON_STATE;
      break;

    case GET_HATCH_STATION_STATE:

      frc::SmartDashboard::PutString("State", "GET HATCH STATION");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->UP_STATE_H;
        }
      }
      if (state_suction) {
        hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
      }
      if (state_solenoids) {
        hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
      }
      if (hatch_pickup->HaveHatch() || post_intake_hatch) {
        state = POST_INTAKE_HATCH_STATE;
      }
      last_state = GET_HATCH_GROUND_STATE;
      break;

    case GET_HATCH_GROUND_STATE:

      frc::SmartDashboard::PutString("State", "GET HATCH GROUND");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->DOWN_STATE_H;
        }
      }
      if (state_bottom_intake) {
        intake->bottom_intake_state = intake->IN_STATE_H;
      }
      if (state_suction) {
        hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
      }
      if (state_solenoids) {
        hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
      }
      if (hatch_pickup->HaveHatch() || post_intake_hatch) {
        state = POST_INTAKE_HATCH_STATE;
      }
      last_state = GET_HATCH_GROUND_STATE;
      break;

    case POST_INTAKE_HATCH_STATE:

      frc::SmartDashboard::PutString("State", "POST INTAKE HATCH");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->UP_STATE_H;
        }
      }
      if (state_suction) {
        hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
      }
      if (state_solenoids) {
        hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
      }
      if (place_hatch) {
        state = PLACE_HATCH_STATE;
      }
      last_state = POST_INTAKE_HATCH_STATE;
      break;

    case GET_CARGO_STATE:

      frc::SmartDashboard::PutString("State", "GET CARGO");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->DOWN_STATE_H;
        }
      }
      if (state_bottom_intake) {
        intake->bottom_intake_state = intake->IN_STATE_H;
      }
      if (state_top_intake) {
        intake->top_intake_state = intake->IN_STATE_H;
      }
      if (intake->HaveBall() || post_intake_cargo) {
        state = POST_INTAKE_CARGO_STATE;
      }
      last_state = GET_CARGO_STATE;
      break;

    case POST_INTAKE_CARGO_STATE:

      frc::SmartDashboard::PutString("State", "POST INTAKE CARGO");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->UP_STATE_H;
        }
      }
      if (state_bottom_intake) {
        intake->bottom_intake_state = intake->IN_STATE_H;
      }
      if (state_top_intake) {
        intake->top_intake_state = intake->IN_STATE_H;
      }
      if (place_cargo) {
        state = PLACE_CARGO_STATE;
      }
      last_state = POST_INTAKE_CARGO_STATE;
      break;

    case PLACE_HATCH_STATE:

      frc::SmartDashboard::PutString("State", "PLACE HATCH");

      if (state_arm) {
        arm->arm_state = arm->UP_STATE_H;
      }
      if (state_solenoids) {
        hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
      }
      if (elevator_hatch_up) {
        elevator->elevator_state = elevator->TOP_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= 0.60 && !place_hatch) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (hatch_pickup->ReleasedHatch()){
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
      } else if (elevator_hatch_mid) {
        elevator->elevator_state = elevator->MID_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= 0.40 && !place_hatch) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (hatch_pickup->ReleasedHatch()){
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
      } else if (elevator_hatch_low) {
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= 0.20 && !place_hatch) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (hatch_pickup->ReleasedHatch()){
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
      }
      last_state = PLACE_HATCH_STATE;
      break;

    case PLACE_CARGO_STATE:

      frc::SmartDashboard::PutString("State", "PLACE CARGO");

      if (state_arm) {
        arm->arm_state = arm->DOWN_STATE_H;
      }
      if (elevator_cargo_up) {
        elevator->elevator_state = elevator->TOP_CARGO_STATE_H;
        if (elevator->GetElevatorPosition() >= 0.60 && !place_cargo) { //placeholder
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (intake->ReleasedBall()) {
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }
      } else if (elevator_cargo_mid) {
        elevator->elevator_state = elevator->MID_CARGO_STATE_H;
        if (elevator->GetElevatorPosition() >= 0.40 && !place_cargo) { //placeholder
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (intake->ReleasedBall()) {
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }
      } else if (elevator_cargo_low) {
         elevator->elevator_state = elevator->BOTTOM_CARGO_STATE_H;
         if (elevator->GetElevatorPosition() >= 0.20 && !place_cargo) { //placeholder
           intake->top_intake_state = intake->OUT_STATE_H;
           intake->bottom_intake_state = intake->OUT_STATE_H;
           if (intake->ReleasedBall()) {
             state = POST_OUTTAKE_CARGO_STATE;
           }
         }
      }
      last_state = PLACE_CARGO_STATE;
      break;

    case POST_OUTTAKE_HATCH_STATE:

      frc::SmartDashboard::PutString("State", "POST OUTTAKE HATCH");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->UP_STATE_H;
        }
      }
      if (state_solenoids) {
        hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
      }
      if (state_suction) {
        hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
      }
      state = WAIT_FOR_BUTTON_STATE;
      last_state = POST_OUTTAKE_HATCH_STATE;
      break;

    case POST_OUTTAKE_CARGO_STATE:

      frc::SmartDashboard::PutString("State", "POST OUTTAKE CARGO");

      if (state_elevator){
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (elevator->GetElevatorPosition() >= .20) {
          arm->arm_state = arm->UP_STATE_H;
        }
      }
      if (state_top_intake) {
        intake->top_intake_state = intake->STOP_STATE_H;
      }
      if (state_bottom_intake) {
        intake->bottom_intake_state = intake->STOP_STATE_H;
      }
      state = WAIT_FOR_BUTTON_STATE;
      last_state = POST_OUTTAKE_CARGO_STATE;
      break;

  }

}
