#ifndef SRC_CENTERLOWHAB_ROCKETRIGHTCENTER_ONECARGO_H_


#include "AutonDrive.h"

// // // Code Name: Bartholomew_Fragrance@gmail_escapevesselright_uno.cpp

class CenterLowHabRocketRightCenterOneCargo : public AutonDrive {

public:
  CenterLowHabRocketRightCenterOneCargo(Waypoint start);
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
