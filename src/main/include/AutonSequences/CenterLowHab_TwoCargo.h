#ifndef SRC_CENTERLOWHAB_TWOCARGO_H_

#include "AutonDrive.h"


class CenterLowHabTwoCargo : public AutonDrive {
public:
  CenterLowHabTwoCargo(Waypoint start);
  void BuildTotalTrajectory();
//
// protected:
//   double MAX_VELOCITY;
//   double MAX_ACCELERATION;
//   double MAX_JERK;
//   double dt;
//   double WHEELBASE_WIDTH;
};
#endif
