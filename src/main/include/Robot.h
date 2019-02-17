/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "TeleopStateMachine.h"
#include "ArmMotionProfiler.h"
#include "Arm.h"
#include "Intake.h"
#include "Elevator.h"
#include "ElevatorMotionProfiler.h"
#include "HatchPickup.h"
#include <frc/Joystick.h>

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
  frc::Joystick *joyOp, *joyWheel;


  bool  wait_for_button,  bottom_intake_in,  bottom_intake_out,
       bottom_intake_stop,  top_intake_in,  top_intake_out,  top_intake_stop,
       suction_on,  suction_off,  hatch_out,  hatch_in,  arm_up,  arm_down,
       elevator_hatch_up,  elevator_hatch_mid,  elevator_hatch_low,  elevator_cargo_up,  elevator_cargo_mid,  elevator_cargo_low,  get_cargo,  get_hatch_ground, get_hatch_station,  post_intake_cargo,  post_intake_hatch,  place_hatch, place_cargo,  post_outtake_hatch,  post_outtake_cargo;

  ElevatorMotionProfiler *elevator_profiler;
  ArmMotionProfiler *arm_profiler;

  Arm *arm;
  Elevator *elevator;
  Intake *intake;
  HatchPickup *hatch_pickup;



 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
#endif
