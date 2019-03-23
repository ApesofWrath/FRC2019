#include "../../include/AutonSequences/CenterLowHab_OneCargo.h"

const int IDLE = 0;
const int PLACE_ELEMENT = 1;
const int STOP_STATE = 2;

CenterLowHabOneCargo::CenterLowHabOneCargo(Waypoint start) {
  start_pos = start;
  MAX_VELOCITY = 11.0;
  MAX_ACCELERATION = 6.0;
  MAX_JERK = 100000.0;
  dt = 0.02; // in seconds
  WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)
}

void CenterLowHabOneCargo::BuildTotalTrajectory() {
  LeftFrontCargoBay(start_pos, false);
  FillRemainingTrajectory();
  PrintTrajectory();
}

void CenterLowHabOneCargo::UpdateState() {
  if (drive_controller->GetDriveIndex() == zeroing_indeces.at(0)) {

  }
}

void CenterLowHabOneCargo::CenterLowHabOneCargoStateMachine(bool *place_cargo) {

  switch (auton_sequence_state) {

    case IDLE:
    break;

    case PLACE_ELEMENT:
      if (auton_state_machine->shoot_counter == 0) {
        *place_cargo = true;
        if (auton_state_machine->GetLeftVel() < 0.5) {
          auton_state_machine->place_cargo = true;
        } else {
          auton_state_machine->place_cargo = false;
        }
      }
      state = STOP_STATE;
      break;

    case STOP_STATE:
      auton_state_machine->stop = true;
      break;
  }
}
