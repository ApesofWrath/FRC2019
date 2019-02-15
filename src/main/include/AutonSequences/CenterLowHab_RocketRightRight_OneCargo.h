#ifndef SRC_CENTERLOWHAB_ROCKETRIGHTRIGHT_ONECARGO_H_

#include "AutonDrive.h"

// // // Code Name: Bartholomew_Fragrance@gmail_escapevesselright_uno.cpp

class CenterLowHabRocketRightRightOneCargo : public AutonDrive {

public:
  CenterLowHabRocketRightRightOneCargo(Waypoint start);
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
