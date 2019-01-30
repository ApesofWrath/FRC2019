#ifndef SRC_VISION_H_
#define SRC_VISION_H_

#include <frc/WPILib.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class Vision {
public:

	Vision();
	auto GetYawToTarget();
	auto GetDepthToTarget();

};

#endif
