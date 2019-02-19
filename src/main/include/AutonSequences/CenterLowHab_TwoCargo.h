#ifndef SRC_CENTERLOWHAB_TWOCARGO_H_

#include "AutonDrive.h"


class CenterLowHabTwoCargo : public AutonDrive {
public:
  CenterLowHabTwoCargo(Waypoint start);
  void BuildTotalTrajectory();
  void CenterLowHabTwoCargoStateMachine();

  const int PLACE_ELEMENT_H = 0;
  const int STOP_STATE_H = 1;
//
// protected:
//   double MAX_VELOCITY;1
//   double MAX_ACCELERATION;
//   double MAX_JERK;
//   double dt;
//   double WHEELBASE_WIDTH;
};
#endif
