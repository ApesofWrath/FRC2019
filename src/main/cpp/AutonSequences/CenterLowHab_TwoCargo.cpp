#include "../../include/AutonSequences/CenterLowHab_TwoCargo.h"

const int PLACE_ELEMENT = 0;
const int GET_ELEMENT_TWO = 1;
const int PLACE_ELEMENT_TWO = 2;
const int STOP_STATE = 3;

CenterLowHabTwoCargo::CenterLowHabTwoCargo(Waypoint start) {
  start_pos = start;
  MAX_VELOCITY = 11.0;
  MAX_ACCELERATION = 6.0;
  MAX_JERK = 100000.0;
  dt = 0.02; // in seconds
  WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)
}

void CenterLowHabTwoCargo::BuildTotalTrajectory() {
  Waypoint end;
  end = LeftFrontCargoBay(start_pos, false);
  end = BottomCargoRefill(end, true);
  RightFrontCargoBay(end, false);
  FillRemainingTrajectory();
  PrintTrajectory();
}

void CenterLowHabTwoCargo::CenterLowHabTwoCargoStateMachine(bool *place_cargo, bool *get_cargo) {

  switch (auton_sequence_state) {

    case PLACE_ELEMENT:
      if (auton_state_machine->shoot_counter == 0) {
        *place_cargo = true;
        if (auton_state_machine->GetLeftVel() < 0.5) {
          auton_state_machine->place_cargo = true;
        } else {
          auton_state_machine->place_cargo = false;
        }
      }
      state = GET_ELEMENT_TWO
      break;

    case GET_ELEMENT_TWO:
      if (auton_state_machine->shoot_counter == 1) {
        *get_cargo = true;
        if (auton_state_machine->GetLeftVel() < 0.5) {
          auton_state_machine->get_cargo = true;
        } else {
          auton_state_machine->get_cargo = false;
        }
      }
      break;

    case PLACE_ELEMENT_TWO:
      if (auton_state_machine->shoot_counter == 1) {
        *place_cargo = true;
        if (auton_state_machine->GetLeftVel() < 0.5) {
          auton_state_machine->place_cargo = true;
        } else {
          auton_state_machine->place_cargo = false;
        }
      }
      state = STOP_STATE
      break;

    case STOP_STATE:
      auton_state_machine=?stop = true;
      break;
  }
}
