// IMPORTANT
  // CODE NAME: Bartholomew_Fragrance@gmail_leftcargo_lame_uno.cpp
    // breakdown for future code names:
        // Bartholomew_Fragrance = center of low hab
        // Leftcargo = the left most cargo
        // Lame = the bays right in front of the hatch
        // uno = number of balls (1)
        // reloadblasters = player station
        // escapevesselleft = leftmost rocket
        // escapevesselright = rightmost rocket
#ifndef SRC_CENTERLOWHAB_ONECARGO_H_

#include "AutonDrive.h"

class CenterLowHabOneCargo : public AutonDrive {

public:
  CenterLowHabOneCargo(Waypoint start);
  void BuildTotalTrajectory();
  void CenterLowHabOneCargoStateMachine();

const int PLACE_ELEMENT_H = 0;
const int STOP_STATE_H = 1;
// protected:
//   double MAX_VELOCITY;
//   double MAX_ACCELERATION;
//   double MAX_JERK;
//   double dt;
//   double WHEELBASE_WIDTH;
};
#endif
