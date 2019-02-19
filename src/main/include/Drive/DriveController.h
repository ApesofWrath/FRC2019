#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	//24, 6, 7, 0, 21, 5, 8, 0, 0, 0, 0, false - comp bot //TODO: ifdef for comp/practice bot

	//l1, l2, l3, l4, r1, r2, r3, r4, pcm, f_channel, r_channel, two_speed
	//left right from robot perspective
	DriveController() : DriveBase(16, 4, 7, 0, 49, 1, 2, 0, 0, 0, 0, false) {


	}

};

#endif
