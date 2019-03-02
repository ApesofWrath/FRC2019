#include "TeleopStateMachine.h"

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;

const int GET_HATCH_STATION_INIT_STATE = 2;
const int GET_HATCH_STATION_SUCTION_STATE = 3;
const int GET_HATCH_GROUND_INIT_STATE = 4;
const int GET_HATCH_GROUND_SUCTION_STATE = 5;
const int POST_INTAKE_HATCH_STATE = 6;

const int GET_CARGO_GROUND_STATE = 7;
const int GET_CARGO_STATION_STATE = 8;
const int POST_INTAKE_CARGO_STATE = 9;

const int PLACE_HATCH_LOW_STATE = 10;
const int PLACE_HATCH_MID_STATE = 11;
const int PLACE_HATCH_HIGH_STATE = 12;

const int PLACE_CARGO_LOW_STATE = 13;
const int PLACE_CARGO_MID_STATE = 14;
const int PLACE_CARGO_HIGH_STATE = 15;
const int PLACE_CARGO_BAY_STATE = 16;

const int POST_OUTTAKE_HATCH_STATE = 17;
const int POST_OUTTAKE_CARGO_STATE = 18;
int state = INIT_STATE;

int last_state = 0;

int counter_suction = 0;

bool state_top_intake = false; //set to false to override the states set in the state machine
bool state_bottom_intake = false;
bool state_arm = false;
bool state_elevator = false;
bool state_suction = false;
bool state_solenoids = false;

bool auto_balanced = false;

TeleopStateMachine::TeleopStateMachine(DriveController *drive_, Elevator *elevator_, Intake *intake_,
  Arm *arm_, HatchPickup *hatch_pickup_){

    state = INIT_STATE;

    drive = drive_;
    elevator = elevator_;
    intake = intake_;
    arm = arm_;
    hatch_pickup = hatch_pickup_;

  }

  void TeleopStateMachine::Initialize(){
    state = INIT_STATE;
  }

//only used in place high states
  void TeleopStateMachine::AutoBalance() {

    if (std::abs(drive->GetRoll()) > 0.2 && elevator->GetElevatorPosition() > elevator->LIFTING_ARM_POS) { //0.2 rad ~11 deg
        state_elevator = false; //override manual control
        state_arm = false;
        arm->arm_state = arm->REST_STATE_H;
        if (arm->GetAngularPosition() > 2.0) {
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        }
        auto_balanced = true;
    } else if (auto_balanced) {
        state_elevator = true;
        state_arm = true;
        auto_balanced = false; //allow manual control again
    }

  }

  void TeleopStateMachine::StateMachine(bool wait_for_button, bool bottom_intake_in, bool bottom_intake_out,
    bool bottom_intake_stop, bool top_intake_in, bool top_intake_out, bool top_intake_stop,
    bool suction_on, bool suction_off, bool hatch_out, bool hatch_in, bool arm_up, bool arm_mid,
    bool arm_high_cargo, bool arm_down, bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low,
    bool elevator_cargo_up, bool elevator_cargo_mid, bool elevator_cargo_low, bool get_cargo_ground,
    bool get_cargo_station, bool get_hatch_ground, bool get_hatch_station, bool post_intake_cargo,
    bool post_intake_hatch, bool place_hatch_high, bool place_hatch_mid, bool place_hatch_low, bool place_cargo_high,
    bool place_cargo_mid, bool place_cargo_low, bool place_cargo_bay, bool post_outtake_hatch, bool post_outtake_cargo, bool extra_button) {

      if (wait_for_button) {
        state = WAIT_FOR_BUTTON_STATE;
      }

      //top intake
      if (top_intake_in) {
        // state_top_intake = false;
        // intake->top_intake_state = intake->IN_STATE_H;
      } else if (top_intake_out) {
        state_top_intake = false;
        intake->top_intake_state = intake->OUT_STATE_H;
      } else if (top_intake_stop) {
        state_top_intake = false;
        intake->top_intake_state = intake->STOP_STATE_H;
      } else {
        state_top_intake = true;
      }

      //bottom intake
      if (bottom_intake_in) {
       state_bottom_intake = false;
       intake->bottom_intake_state = intake->IN_SLOW_STATE_H; //slow, not full speed
      } else if (bottom_intake_out) {
        state_bottom_intake = false;
        intake->bottom_intake_state = intake->OUT_SLOW_STATE_H;
      } else if (bottom_intake_stop) {
        state_bottom_intake = false;
        intake->bottom_intake_state = intake->STOP_STATE_H;
      } else {
        state_bottom_intake = true;
      }

      //arm
      if (arm_up) {
        state_arm = false;
        arm->arm_state = arm->REST_STATE_H;
      } else if (arm_mid) {
        state_arm = false;
        arm->arm_state = arm->HATCH_STATE_H;
      } else if (arm_down) {
        state_arm = false;
        arm->arm_state = arm->CARGO_STATE_H;
      } else if (arm_high_cargo) {
        state_arm = false;
        arm->arm_state = arm->HIGH_CARGO_STATE_H;
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
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
      } else if (elevator_cargo_up) {
        state_elevator = false;
        elevator->elevator_state = elevator->TOP_CARGO_STATE_H;
      } else if (elevator_cargo_mid) {
        state_elevator = false;
        elevator->elevator_state = elevator->MID_CARGO_STATE_H;
      } else if (elevator_cargo_low) {
        state_elevator = false;
        elevator->elevator_state = elevator->BOTTOM_CARGO_STATE_H;
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
        arm->arm_state = arm->INIT_STATE_H;

        intake->top_intake_state = intake->STOP_STATE_H;
        intake->bottom_intake_state = intake->STOP_STATE_H;
        hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
        hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;

        state = WAIT_FOR_BUTTON_STATE;
        last_state = INIT_STATE;
        break;

        case WAIT_FOR_BUTTON_STATE:
        frc::SmartDashboard::PutString("State", "WAIT FOR BUTTON");

        if (get_hatch_station) {
          state = GET_HATCH_STATION_INIT_STATE;
        } else if (get_hatch_ground) {
          state = GET_HATCH_GROUND_INIT_STATE;
        } else if (post_intake_hatch) {
          state = POST_INTAKE_HATCH_STATE;
        } else if (get_cargo_ground) {
          state = GET_CARGO_GROUND_STATE;
        } else if (get_cargo_station) {
          state = GET_CARGO_STATION_STATE;
        } else if (post_intake_cargo) {
          state = POST_INTAKE_CARGO_STATE;
        } else if (place_hatch_low) {
          state = PLACE_HATCH_LOW_STATE;
        } else if (place_hatch_mid) {
          state = PLACE_HATCH_MID_STATE;
        } else if (place_hatch_high) {
          state = PLACE_HATCH_HIGH_STATE;
        } else if (place_cargo_low) {
          state = PLACE_CARGO_LOW_STATE;
        } else if (place_cargo_mid) {
          state = PLACE_CARGO_MID_STATE;
        } else if (place_cargo_high) {
          state = PLACE_CARGO_HIGH_STATE;
        } else if (place_cargo_bay) {
          state = PLACE_CARGO_BAY_STATE;
        } else if (post_outtake_hatch) {
          state = POST_OUTTAKE_HATCH_STATE;
        } else if (post_outtake_cargo) {
          state = POST_OUTTAKE_CARGO_STATE;
        }

        last_state = WAIT_FOR_BUTTON_STATE;
        break;

        case GET_HATCH_STATION_INIT_STATE:
        frc::SmartDashboard::PutString("State", "GET HATCH STATION INIT");
        if (state_elevator) {
          elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        }
        if (state_arm) {
            arm->arm_state = arm->HATCH_STATE_H; //place hatch
        }
        if (std::abs(arm->GetAngularPosition() - arm->HATCH_ANGLE) <= 0.2) {
          counter_suction = 0;
          state = GET_HATCH_STATION_SUCTION_STATE;
        }
        last_state = GET_HATCH_STATION_INIT_STATE;
        break;

        case GET_HATCH_STATION_SUCTION_STATE:
        frc::SmartDashboard::PutString("State", "GET HATCH STATION SUCTION");

        counter_suction++;
        frc::SmartDashboard::PutNumber("counter", counter_suction);

        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
        }
        if (state_solenoids) {
          hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
        }

        if (hatch_pickup->HaveHatch() && counter_suction > 10) { //havehatch() needs suction current to have ramped up already; wait 10 counts
          arm->arm_state = arm->REST_STATE_H;
          if (std::abs(arm->GetAngularPosition() - arm->REST_ANGLE) <= 0.2) {
            state = POST_INTAKE_HATCH_STATE;
          }
        }

        last_state = GET_HATCH_STATION_SUCTION_STATE;
        break;

        case GET_HATCH_GROUND_INIT_STATE:
        frc::SmartDashboard::PutString("State", "GET HATCH GROUND");

        if (state_elevator && arm->arm_state != arm->GET_HATCH_GROUND_STATE_H) {
          elevator->elevator_state = elevator->LIFTING_ARM_STATE_H;
          if (elevator->GetElevatorPosition() >= .30) {
            arm->arm_state = arm->GET_HATCH_GROUND_STATE_H;
          }
        } else if (arm->GetAngularPosition() < 0.7) {
          state = GET_HATCH_GROUND_SUCTION_STATE;
        }

        last_state = GET_HATCH_GROUND_INIT_STATE;
        break;

        case GET_HATCH_GROUND_SUCTION_STATE:
        frc::SmartDashboard::PutString("State", "GET HATCH GROUND SUCTION");

        intake->bottom_intake_state = intake->IN_STATE_H; //start spin once arm is in position
        elevator->elevator_state = elevator->HOLD_HATCH_STATE_H;
        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
        }
        if (state_solenoids) { //seeHatch() TODO: last check to be made
          //intake->bottom_intake_state = intake->STOP_STATE_H;       //stop spin once see hatch
      //    hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
        /*  if (intake->HaveHatch()) { //may see hatch, but need to wait until we suction it
            hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
            hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
            intake->bottom_intake_state = intake->STOP_STATE_H;
            state = POST_INTAKE_HATCH_STATE;
          } */
        }

        if (false || post_intake_hatch) { //hatch_pickup->HaveHatch() - may need to add counter like get hatch station
          hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
          hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
          intake->bottom_intake_state = intake->STOP_STATE_H;
          state = POST_INTAKE_HATCH_STATE;
        }

        last_state = GET_HATCH_GROUND_SUCTION_STATE;
        break;

        case POST_INTAKE_HATCH_STATE:
        frc::SmartDashboard::PutString("State", "POST INTAKE HATCH");

        if (state_bottom_intake) {
        intake->bottom_intake_state = intake->STOP_STATE_H; // from GET_HATCH_GROUND_STATE state
        }
        if (state_top_intake) {
        intake->top_intake_state = intake->STOP_STATE_H; // from GET_HATCH_GROUND_STATE state
        }
        if (state_arm) {
          arm->arm_state = arm->REST_STATE_H;
        }
        if ((arm->GetAngularPosition() > 0.7) && state_elevator) {
          elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        }
        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->ON_STATE_H;
        }
        if (state_solenoids) {
          hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
        }
        if (place_hatch_high) {
          state = PLACE_HATCH_HIGH_STATE;
        } else if (place_hatch_mid) {
          state = PLACE_HATCH_MID_STATE;
        } else if (place_hatch_low) {
          state = PLACE_HATCH_LOW_STATE;
        }

        last_state = POST_INTAKE_HATCH_STATE;
        break;

        case GET_CARGO_GROUND_STATE:
        frc::SmartDashboard::PutString("State", "GET CARGO GROUND");

        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
        }
        if (state_solenoids) {
          hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
        }
        if (state_bottom_intake) {
          intake->bottom_intake_state = intake->IN_STATE_H;
        }
        if (state_top_intake) {
          intake->top_intake_state = intake->IN_STATE_H;
        }

        if (state_elevator && arm->arm_state != arm->GET_HATCH_GROUND_STATE_H) {
          elevator->elevator_state = elevator->LIFTING_ARM_STATE_H;
          if (elevator->GetElevatorPosition() >= .30) {
            arm->arm_state = arm->GET_HATCH_GROUND_STATE_H;
          }
        } else if (state_elevator && arm->GetAngularPosition() < 0.7) {
          elevator->elevator_state = elevator->HOLD_HATCH_STATE_H; //same as hold cargo height
        }

        if (intake->HaveBall() || post_intake_cargo) {
          state = POST_INTAKE_CARGO_STATE;
        }
        last_state = GET_CARGO_GROUND_STATE;
        break;

        case GET_CARGO_STATION_STATE:
        frc::SmartDashboard::PutString("State", "GET CARGO STATION");

        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
        }
        if (state_solenoids) {
          hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
        }
        if (state_bottom_intake) {
          intake->bottom_intake_state = intake->IN_STATE_H;
        }
        if (state_top_intake) {
          intake->top_intake_state = intake->IN_STATE_H;
        }

        if (state_elevator && arm->arm_state != arm->GET_HATCH_GROUND_STATE_H) { //change arm states
          elevator->elevator_state = elevator->LIFTING_ARM_STATE_H;
          if (elevator->GetElevatorPosition() >= .30) { //TODO: Change elevator height
            arm->arm_state = arm->GET_HATCH_GROUND_STATE_H;
          }
        } else if (state_elevator && arm->GetAngularPosition() < 0.7) {
          elevator->elevator_state = elevator->HOLD_HATCH_STATE_H; //same as hold cargo height
        }

        if (intake->HaveBall() || post_intake_cargo) {
          state = POST_INTAKE_CARGO_STATE;
        }
        last_state = GET_CARGO_STATION_STATE;
        break;

        case POST_INTAKE_CARGO_STATE:

        frc::SmartDashboard::PutString("State", "POST INTAKE CARGO");

        if (state_elevator) {
          elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        }
        if (state_arm) {
          arm->arm_state = arm->REST_STATE_H;
        }
        if (state_bottom_intake) {
          intake->bottom_intake_state = intake->HOLD_STATE_H;
        }
        if (state_top_intake) {
          intake->top_intake_state = intake->HOLD_STATE_H;
        }

        if (place_cargo_low) { //as opposed to having to go through wfb to get to a PLACE STATE
          state = PLACE_CARGO_LOW_STATE;
        } else if (place_cargo_mid) {
          state = PLACE_CARGO_MID_STATE;
        } else if (place_cargo_high) {
          state = PLACE_CARGO_HIGH_STATE;
        } else if (place_cargo_bay) {
          state = PLACE_CARGO_BAY_STATE;
        }
        last_state = POST_INTAKE_CARGO_STATE;
        break;

        case PLACE_HATCH_LOW_STATE:
        frc::SmartDashboard::PutString("State", "LOW HATCH");
        elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        if (state_arm) {
          arm->arm_state = arm->HATCH_STATE_H;
        }

        if (std::abs(elevator->GetElevatorPosition() - elevator->BOTTOM_HATCH_POS) < 0.2 && !place_hatch_low) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (state_solenoids) {
            hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
          }
          if (hatch_pickup->ReleasedHatch()) { //extra_button
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
        last_state = PLACE_HATCH_LOW_STATE;
        break;

        case PLACE_HATCH_MID_STATE:
        frc::SmartDashboard::PutString("State", "MID HATCH");
        //AutoBalance();
        elevator->elevator_state = elevator->MID_HATCH_STATE_H;
        if (state_arm) {
          arm->arm_state = arm->HATCH_STATE_H;
        }

        if (std::abs(elevator->GetElevatorPosition() - elevator->MID_HATCH_POS) < 0.2 && !place_hatch_mid) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (state_solenoids) {
            hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
          }
          if (hatch_pickup->ReleasedHatch()) { //extra_button
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
        last_state = PLACE_HATCH_MID_STATE;
        break;

        case PLACE_HATCH_HIGH_STATE:
        //AutoBalance();
        frc::SmartDashboard::PutString("State", "HIGH HATCH");
        elevator->elevator_state = elevator->TOP_HATCH_STATE_H;
        if (state_arm) {
          arm->arm_state = arm->HATCH_STATE_H;
        }

        if (std::abs(elevator->GetElevatorPosition() - elevator->TOP_HATCH_POS) < 0.2 && !place_hatch_high) { //placeholder
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
          if (state_solenoids) {
            hatch_pickup->solenoid_state = hatch_pickup->OUT_STATE_H;
          }
          if (hatch_pickup->ReleasedHatch()) { //extra_button
            state = POST_OUTTAKE_HATCH_STATE;
          }
        }
        last_state = PLACE_HATCH_HIGH_STATE;
        break;

        case PLACE_CARGO_LOW_STATE:
        frc::SmartDashboard::PutString("State", "LOW CARGO");

        if (state_arm) {
          arm->arm_state = arm->CARGO_STATE_H;
        }

        elevator->elevator_state = elevator->BOTTOM_CARGO_STATE_H;
        if (!place_cargo_low) { //placeholder //and arm height at pos
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (intake->ReleasedBall()) { //top_intake_in
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }
        last_state = PLACE_CARGO_LOW_STATE;
        break;

        case PLACE_CARGO_MID_STATE:
        frc::SmartDashboard::PutString("State", "MID CARGO");
        //AutoBalance();
        if (state_arm) {
          arm->arm_state = arm->CARGO_STATE_H;
        }

        elevator->elevator_state = elevator->MID_CARGO_STATE_H;
        if (!place_cargo_mid) { //placeholder //and arm height at pos
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (intake->ReleasedBall()) { //top_intake_in
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }
        last_state = PLACE_CARGO_MID_STATE;
        break;

        case PLACE_CARGO_HIGH_STATE:
        frc::SmartDashboard::PutString("State", "HIGH CARGO");
        //AutoBalance();
        if (state_arm) {
          arm->arm_state = arm->HIGH_CARGO_STATE_H;//arm->HIGH_CARGO_STATE_H;
        }

        elevator->elevator_state = elevator->TOP_CARGO_STATE_H;
        if (!place_cargo_high) { //placeholder //and arm height at pos
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (intake->ReleasedBall()) { //top_intake_in
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }

        last_state = PLACE_CARGO_HIGH_STATE;
        break;

        case PLACE_CARGO_BAY_STATE:
        frc::SmartDashboard::PutString("State", "BAY CARGO");
        //AutoBalance();
        if (state_arm) {
          arm->arm_state = arm->CARGO_STATE_H;
        }

        elevator->elevator_state = elevator->MID_CARGO_STATE_H;
        if (!place_cargo_bay) { //placeholder //and arm height at pos
          intake->top_intake_state = intake->OUT_STATE_H;
          intake->bottom_intake_state = intake->OUT_STATE_H;
          if (bottom_intake_in) { //intake->ReleasedBall(
            state = POST_OUTTAKE_CARGO_STATE;
          }
        }
        last_state = PLACE_CARGO_BAY_STATE;
        break;

        case POST_OUTTAKE_HATCH_STATE:
        frc::SmartDashboard::PutString("State", "POST OUTTAKE HATCH");

        if (state_arm) {
          arm->arm_state = arm->REST_STATE_H;
        }

        if (state_elevator && arm->GetAngularPosition() > 2.0) {
          elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
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

        if (state_elevator) {
          elevator->elevator_state = elevator->BOTTOM_HATCH_STATE_H;
        }
        if (state_arm) {
          arm->arm_state = arm->REST_STATE_H;
        }
        if (state_top_intake) {
          intake->top_intake_state = intake->STOP_STATE_H;
        }
        if (state_bottom_intake) {
          intake->bottom_intake_state = intake->STOP_STATE_H;
        }
        if (state_solenoids) {
          hatch_pickup->solenoid_state = hatch_pickup->IN_STATE_H;
        }
        if (state_suction) {
          hatch_pickup->suction_state = hatch_pickup->OFF_STATE_H;
        }

        state = WAIT_FOR_BUTTON_STATE;
        last_state = POST_OUTTAKE_CARGO_STATE;
        break;
      }

    }
