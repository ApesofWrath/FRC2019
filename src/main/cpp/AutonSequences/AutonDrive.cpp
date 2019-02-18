#include "../../include/AutonSequences/AutonDrive.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> // for timing how long it takes to generate a profile
#include <chrono> // ditto

// Fix errors and add to header file so subclasses have access to it
std::vector<std::vector<double> > full_refs(1500, std::vector<double>(5)); // initalizes each index value to 0;

int current_index = 0;

AutonDrive::AutonDrive() {}

void AutonDrive::GeneratePartialTrajectory(int num_points, Waypoint points[], bool isReversed) {
  // TODO: current_index or current_index + 1 makes more sense?
  zeroing_indeces.push_back(current_index);

  TrajectoryCandidate candidate;

  // Samples high returns 1000 points with a dt timestep
  pathfinder_prepare(points, num_points, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, dt, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK, &candidate);

  int length = candidate.length;
  // myfile << "LENGTH: " << std::to_string(length);
  Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

 int result = pathfinder_generate(&candidate, trajectory);

  Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

  pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, WHEELBASE_WIDTH);

  for (int i = 0; i < length; i++) {
    Segment sl = rightTrajectory[i]; //THE LEFT AND RIGHT TRAJECTORIES MUST BE SWITCHED WHEN GOING BACKWARDS. OTHERWISE, ANGLE WILL NOT CORRESPOND TO EACH SIDE'S MAGNITUDES. PATHFINDER ALWAYS ASSUMES FORWRAD TRAJECTORIES; IN ADDITION TO REVERSING (*-1.0 and -PI for yaw), WE MUST SWITCH THE LEFT AND RIGHT TRAJECTORIES
		Segment sr = leftTrajectory[i]; //magnitudes AND directions must reverse, to get mirror image
    if (isReversed) {
      full_refs.at(current_index).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards
      full_refs.at(current_index).at(1) = -1.0 * ((double) sl.position); //pathfinder does not give negative references
      full_refs.at(current_index).at(2) = -1.0 * ((double) sr.position);
      full_refs.at(current_index).at(3) = -1.0 * ((double) sl.velocity);
      full_refs.at(current_index).at(4) = -1.0 * ((double) sr.velocity);
    } else {
      full_refs.at(current_index).at(0) = (double) sl.heading;
  		full_refs.at(current_index).at(1) = (double) sl.position;
  		full_refs.at(current_index).at(2) = (double) sr.position;
  		full_refs.at(current_index).at(3) = (double) sl.velocity;
  		full_refs.at(current_index).at(4) = (double) sr.velocity;
    }

    current_index++;;
  }

  free(leftTrajectory);
  free(rightTrajectory);
  free(trajectory);
}

Waypoint AutonDrive::BottomCargoRefill(Waypoint start_point, bool isReversed) {
  Waypoint points[3];
  Waypoint mid = {9.5, 3, d2r(180)};
  Waypoint end = {8.5, -4, d2r(-90)};
  points[0] = start_point;
  points[1] = mid;
  points[2] = end;
  GeneratePartialTrajectory(3, points, isReversed);
  return end;
}

// May need to add option to go through further point to make sure it doesn't hit the hab platform
Waypoint AutonDrive::LeftFrontCargoBay(Waypoint start_point, bool isReversed) {
  Waypoint points[2];
  Waypoint end = {-1, 12, d2r(0)};
  points[0] = start_point;
  points[1] = end;
  GeneratePartialTrajectory(2, points, isReversed);
  return end;
}

Waypoint AutonDrive::RightFrontCargoBay(Waypoint start_point, bool isReversed) {
  Waypoint points[2];
  Waypoint end = {1, 12, d2r(0)};
  points[0] = start_point;
  points[1] = end;
  GeneratePartialTrajectory(2, points, isReversed);
  return end;
}

Waypoint AutonDrive::TestModule(Waypoint start_point) {
  int num_points = 3;
  Waypoint points[num_points]; // Assuming start and end point only
  Waypoint p2 = { -16, 0, d2r(0) };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
  Waypoint p3 = {  -21.5, 1.1, d2r(-10) };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
  points[0] = start_point;
  points[1] = p2;
  points[2] = p3;
  GeneratePartialTrajectory(num_points, points, false);
  return p3;
}

Waypoint AutonDrive::Forward(Waypoint start_point, bool isReversed) {
  Waypoint points[2];
  Waypoint end = {4, 0, d2r(0)};
  points[0] = start_point;
  points[1] = end;
  GeneratePartialTrajectory(2, points, isReversed);
  return end;
}

Waypoint AutonDrive::RightRocketFront(Waypoint start_point, bool isReversed) {
  Waypoint points[2];
  Waypoint end = { 10.5 , 13 , d2r(90) };
  points[0] = start_point;
  points[1] = end;
  GeneratePartialTrajectory(2,points, isReversed);
}

void AutonDrive::FillRemainingTrajectory() {
  // Out of bounds
  if (current_index >= 0 && current_index <= 1500) {
    frc::SmartDashboard::PutString("fill", "y");
      for (int i = current_index; i < 1500; i++) {
        full_refs.at(i).at(0) = full_refs.at(i - 1).at(0);
        full_refs.at(i).at(1) = full_refs.at(i - 1).at(1);
        full_refs.at(i).at(2) = full_refs.at(i - 1).at(2);
        full_refs.at(i).at(3) = full_refs.at(i - 1).at(3);
        full_refs.at(i).at(4) = full_refs.at(i - 1).at(4);
    }
  }
}

void AutonDrive::PrintTrajectory() {
    std::ofstream myfile;
    myfile.open("/home/lvuser/test.txt");
    myfile << "current_index: " << std::to_string(current_index);
  for (int i = 0; i < current_index; i++) {
    myfile << std::to_string(full_refs.at(i).at(0)) << "\n";
    myfile << std::to_string(full_refs.at(i).at(1)) << "\n";
    myfile << std::to_string(full_refs.at(i).at(2)) << "\n";
    myfile << std::to_string(full_refs.at(i).at(3)) << "\n";
    myfile << std::to_string(full_refs.at(i).at(4)) << "\n";
  }
  myfile.close();
}

std::vector<std::vector<double> > AutonDrive::GetFullProfile() {

  return full_refs;

}
