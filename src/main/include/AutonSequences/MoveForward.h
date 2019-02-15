// IMPORTANT
  // CODE NAME: Bartholomew_Fragrance@gmail_leftcargo_lame_uno.cpp
    // breakdown for future code names:
        // Bartholomew_Fragrance = center of low hab
        // Leftcargo = the left most cargo
        // Lame = the bays right in front of the hatch
        // uno = number of balls (1)
#ifndef SRC_CENTERLOWHAB_ROCKETLEFTLEFT_ONECARGO_H_

#include "AutonDrive.h"

class MoveForward : public AutonDrive {

public:
  MoveForward(Waypoint start);
  void BuildTotalTrajectory();
};
#endif
