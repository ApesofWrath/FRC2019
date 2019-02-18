#include "../../include/AutonSequences/MoveForward.h"

MoveForward::MoveForward(Waypoint start) {
  start_pos = start;
  MAX_VELOCITY = 11.0;
  MAX_ACCELERATION = 6.0;
  MAX_JERK = 100000.0;
  dt = 0.02; // in seconds
  WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)
}

void MoveForward::BuildTotalTrajectory() {
  Waypoint start = {0, 0, 0};
  Forward(start, false); // To make it forward/backward change this to true and false respectively
  FillRemainingTrajectory();
  PrintTrajectory();
}
