#ifndef SRC_CENTERLOWHAB_ROCKETLEFTCENTER_ONECARGO_H_

#include "AutonDrive.h"

// // // Code Name: Bartholomew_Fragrance@gmail_escapevesselright_uno.cpp

class CenterLowHabRocketLeftCenterOneCargo : public AutonDrive {

public:
  CenterLowHabRocketLeftCenterOneCargo(Waypoint start);
  void BuildTotalTrajectory();
  void CenterLowHabRocketLeftCenterOneCargoStateMachine();

  const int PLACE_ELEMENT_H = 0;
  const int STOP_STATE_H = 1;
//
// protected:
//   double MAX_VELOCITY;
//   double MAX_ACCELERATION;
//   double MAX_JERK;
//   double dt;
//   double WHEELBASE_WIDTH;
};
#endif
