#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	//l1, l2, l3, l4, r1, r2, r3, r4, pcm, f_channel, r_channel, two_speed
	DriveController() : DriveBase(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true) {


	}

};

#endif
