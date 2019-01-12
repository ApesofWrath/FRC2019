#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	const int l1 = 0;
	const int l2 = 0;
	const int l3 = 0;
	const int l4 = 0;
	const int r1 = 0;
	const int r2 = 0;
	const int r3 = 0;
	const int r4 = 0;

	const int pcm = -1;
	const int f_channel = -1;
	const int r_channel = -1;

	bool two_speed = false;

	DriveController() : DriveBase(l1, l2, l3, l4, r1, r2, r3, r4, pcm, f_channel, r_channel, two_speed) {


	}

};

#endif
