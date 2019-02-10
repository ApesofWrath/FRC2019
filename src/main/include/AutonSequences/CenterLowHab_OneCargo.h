// IMPORTANT
  // CODE NAME: Bartholomew_Fragrance@gmail_leftcargo_lame_uno.cpp
    // breakdown for future code names:
        // Bartholomew_Fragrance = center of low hab
        // Leftcargo = the left most cargo
        // Lame = the bays right in front of the hatch
        // uno = number of balls (1)

#include "AutonDrive.h"

class CenterLowHabOneCargo : public AutonDrive {

public:
  CenterLowHabOneCargo(Waypoint start);
  void BuildTotalTrajectory();
};
