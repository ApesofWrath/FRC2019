#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	DriveController() : DriveBase(18, 23, 30, 36, 24, 21, 22, 29, true) {


	}

};

#endif /* SRC_DRIVE_H_ */
