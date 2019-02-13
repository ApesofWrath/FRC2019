#include "../../include/AutonSequences/CenterLowHab_RocketLeftCenter_OneCargo.h"

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
