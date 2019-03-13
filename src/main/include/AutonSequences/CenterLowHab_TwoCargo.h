#include "AutonDrive.h"


class CenterLowHabTwoCargo : public AutonDrive {
public:
  CenterLowHabTwoCargo(Waypoint start);
  void BuildTotalTrajectory();
};
