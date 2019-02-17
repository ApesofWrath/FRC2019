#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	//l1, l2, l3, l4, r1, r2, r3, r4, pcm, f_channel, r_channel, two_speed
	//left right from robot perspective
	DriveController() : DriveBase(18, 23, 30, 36, 24, 21, 22, 29, 0, 0, 0, false) { // TODO: change back for new robot


	}

};

#endif
