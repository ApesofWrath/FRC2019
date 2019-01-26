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
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <frc/IterativeRobot.h>
#include <frc/DigitalOutput.h>
#include <frc/DoubleSolenoid.h>
//#include <frc/Compressor.h>


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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  const int JOY_THROTTLE = 0;

  const int BOTTOM_INTAKE_IN = 8;
  const int BOTTOM_INTAKE_OUT = 9;
  const int TOP_INTAKE_IN = 4;
  const int TOP_INTAKE_OUT = 3;
  const int BOTH_INTAKES_IN = 11;
  const int BOTH_INTAKES_OUT = 10;
  const int SUCTION_IN = 6;
  const int SUCTION_RELEASE = 7;
  const int PUSH_HATCH = 1;

  frc::Joystick *joyThrottle;

  TalonSRX *canTalonTopIntake, *canTalonBottomIntake, *suction1, *suction2;
  frc::DoubleSolenoid *doubleSolenoid;
  //frc::Compressor *compressor;
};

#endif
