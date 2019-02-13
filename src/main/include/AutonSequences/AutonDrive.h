/* TODO:
*    -Make BuildTotalTrajectory abstract/virtual function for all subclasses to *     implement (will keep the naming consistent and eliminate hassle)
*
*/

#ifndef SRC_AUTONDRIVE_H_
#define SRC_AUTONDRIVE_H_

#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fstream>
#include <vector>
#include <list>
#include <pathfinder.h>
// #include "../AutonStateMachine.h"
// #include "../DriverController.h"

class AutonDrive {

public:
	// DriveController *drive_controller;
	// AutonStateMachine *auton_state_machine;

  // Other variables that sequences need to have access to like Elevaotr, intake, etc.

  // AutonDrive(DriveController *dc, AutonStateMachine *ausm);
  AutonDrive();
  void PrintTrajectory();
  std::vector<std::vector<double> > GetFullProfile();

protected:
    // SI units
    double MAX_VELOCITY = 11.0;
    double MAX_ACCELERATION = 6.0;
    double MAX_JERK = 100000.0;
    double dt = 0.02; // in seconds
    double WHEELBASE_WIDTH = 2.1; // TODO: determine (in meters)

    Waypoint start_pos; // All start points are relative to the center of the low hab platform as (0, 0, 0)
		std::vector<int> zeroing_indeces; // can also add the profile refs here

		void GeneratePartialTrajectory(int num_points, Waypoint points[], bool isReversed);

    // return waypoint as start position for next segment
    Waypoint LeftFrontCargoBay(Waypoint start_point, bool isReversed);
    Waypoint BottomCargoRefill(Waypoint start_point, bool isReversed);
    Waypoint RightFrontCargoBay(Waypoint start_point, bool isReversed);
    Waypoint Forward(Waypoint start_point, bool isReversed);
    Waypoint RightRocketFront(Waypoint start_point, bool isReversed);
    Waypoint LeftRocketFront(Waypoint start_point, bool isReversed);
    Waypoint LeftRocketLeft(Waypoint start_point, bool isReversed);
    Waypoint RightRocketRight(Waypoint start_point, bool isReversed);

    Waypoint TestModule(Waypoint start_point);
    void FillRemainingTrajectory();
};


#endif /* SRC_AUTONDRIVE_H_ */
