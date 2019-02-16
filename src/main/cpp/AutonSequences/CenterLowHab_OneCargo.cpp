#include "../../include/AutonSequences/CenterLowHab_OneCargo.h"

CenterLowHabOneCargo::CenterLowHabOneCargo(Waypoint start) {
  start_pos = start;
  MAX_VELOCITY = 11.0;
  MAX_ACCELERATION = 6.0;
  MAX_JERK = 100000.0;
  dt = 0.02; // in seconds
  WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)

  const int PLACE_ELEMENT_STATE = 0;
  const int STOP_STATE = 1;
  int cargo_ship_one_cargo_state = 0;
}

void CenterLowHabOneCargo::BuildTotalTrajectory() {
  LeftFrontCargoBay(start_pos, false);
  FillRemainingTrajectory();
  PrintTrajectory();
}

void CenterLowHabOneCargo::CenterLowHabOneCargoStateMachine() {

  switch (cargo_ship_one_cargo_state) {

    case PLACE_ELEMENT_STATE:
      
      break;

    case STOP_STATE:
      break;
  }
}
