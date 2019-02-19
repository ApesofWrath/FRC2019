#include "../../include/AutonSequences/CenterLowHab_RocketLeftCenter_OneCargo.h"

const int PLACE_ELEMENT = 0;
const int STOP_STATE = 1;

CenterLowHabRocketLeftCenterOneCargo::CenterLowHabRocketLeftCenterOneCargo(Waypoint start) {
  start_pos = start;
  MAX_VELOCITY = 11.0;
  MAX_ACCELERATION = 6.0;
  MAX_JERK = 100000.0;
  dt = 0.02; // in seconds
  WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)
}

void CenterLowHabRocketLeftCenterOneCargo::BuildTotalTrajectory() {
  LeftRocketFront(start_pos, false);
  FillRemainingTrajectory();
  PrintTrajectory();
}

void CenterLowHabRocketLeftCenterOneCargo::CenterLowHabRocketLeftCenterOneCargoStateMachine(bool *place_cargo) {

  switch (rocket_left_center_one_cargo_state) {

    case PLACE_ELEMENT:
      if (auton_state_machine->shoot_counter == 0) {
        *place_cargo = true;
        if (auton_state_machine->GetLeftVel() < 0.5) {
          auton_state_machine->place_rocket_cargo = true;
        } else {
          auton_state_machine->place_rocket_cargo = false;
        }
      }
      state = STOP_STATE
      break;

    case STOP_STATE:
      auton_state_machine->stop = true;
      break;
  }
}
