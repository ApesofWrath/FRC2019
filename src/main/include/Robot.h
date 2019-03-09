/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "TeleopStateMachine.h"
#include "MotionProfiling/ArmMotionProfiler.h"
#include "Arm.h"
#include "Intake.h"
#include "Elevator.h"
#include "MotionProfiling/ElevatorMotionProfiler.h"
#include "HatchPickup.h"
#include <frc/Joystick.h>
#include <pathfinder.h>
#include "AutonSequences/MoveForward.h"
#include "Drive/DriveController.h"
#include "Drive/Vision.h"

class Robot : public frc::TimedRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  TeleopStateMachine *tsm;
  frc::Joystick *joyOp1, *joyOp2;


  bool  wait_for_button,  bottom_intake_in,  bottom_intake_out,
       bottom_intake_stop,  top_intake_in,  top_intake_out,  top_intake_stop,
       suction_on,  suction_off,  hatch_out,  hatch_in,  arm_up, arm_mid, arm_high_cargo, arm_down,
       elevator_hatch_up,  elevator_hatch_mid,  elevator_hatch_low,  elevator_cargo_up,  elevator_cargo_mid,  elevator_cargo_low,  get_cargo_ground, get_cargo_station, get_hatch_ground, get_hatch_station,  post_intake_cargo,  post_intake_hatch,
       place_hatch_high, place_hatch_mid, place_hatch_low, place_cargo_high, place_cargo_mid, place_cargo_low, post_outtake_hatch,  post_outtake_cargo, extra_button, place_cargo_bay;

  ElevatorMotionProfiler *elevator_profiler;
  ArmMotionProfiler *arm_profiler;

  Arm *arm;
  Elevator *elevator;
  Intake *intake;
  HatchPickup *hatch_pickup;



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
//  Waypoint start;

  const int JOY_THROTTLE = 0;
  const int JOY_WHEEL = 1;
  const int JOY_OP = 2;

  PowerDistributionPanel *pdp;
  frc::Joystick *joyThrottle, *joyWheel;
  DriveController *drive_controller;
  Vision *vision;
  frc::Solenoid *led_solenoid;
  //MoveForward *move_forward;

  bool is_rotation, is_vision, is_regular;

};
#endif
