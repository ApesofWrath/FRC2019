/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  const int JOY_THROTTLE = 0;

  const int BOTTOM_INTAKE_IN = 1;
  const int BOTTOM_INTAKE_OUT = 2;
  const int TOP_INTAKE_IN = 3;
  const int TOP_INTAKE_OUT = 4;
  const int SUCTION_IN = 5;
  const int SUCTION_RELEASE = 6;

  Joystick *joyThrottle;

  TalonSRX *canTalonTopIntake, *canTalonBottomIntake;

  DigitalOutput *suctionIn, *suctionOut;

  DoubleSolenoid *doubleSolenoid1, *doubleSolenoid2;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
