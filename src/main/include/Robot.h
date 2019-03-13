/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <string>
#include <frc/Joystick.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <pathfinder.h>

#include "AutonSequences/MoveForward.h"
#include "AutonSequences/CenterLowHab_TwoCargo.h"
#include "DriveController.h"
#include "Vision.h"

class Robot : public frc::TimedRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:

  nt::NetworkTableEntry xEntry;

  //auton sequence
  frc::SendableChooser<std::string> auton_chooser;
  const std::string kDoNothing = "DoNothing";
  const std::string kDriveForward = "DriveForward";
  const std::string kCenterLowHabOneCargo = "CenterLowHabOneCargo";
  std::string m_autoSelected;

  //where robot starts on the field
  frc::SendableChooser<std::string> start_chooser;
  const std::string kLeft = "Left";
  const std::string kCenter = "Center";
  const std::string kRight = "Right";
  std::string m_startSelected;
  Waypoint start;

  const int JOY_THROTTLE = 0;
  const int JOY_WHEEL = 1;
  const int JOY_OP = 2;

  Joystick *joyThrottle, *joyWheel, *joyOp;
  DriveController *drive_controller;
  Vision *vision;
  CenterLowHabTwoCargo *sequence;

  bool is_rotation, is_vision, is_regular;

};

#endif
