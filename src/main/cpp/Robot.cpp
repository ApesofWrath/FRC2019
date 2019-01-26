/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  joyThrottle = new Joystick(JOY_THROTTLE);
	joyWheel = new Joystick(JOY_WHEEL);
	joyOp = new Joystick(JOY_OP);

  drive_controller = new DriveController();

}

void Robot::RobotPeriodic() {

  auto inst = nt::NetworkTableInstance::GetDefault(); //TODO: figure out how to init once only
  auto table = inst.GetTable("SmartDashboard");

  xEntry = table->GetEntry("X");

}

void Robot::AutonomousInit() {

  drive_controller->ZeroAll(true);

  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = SmartDashboard::GetString("Auto Selector",
       kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

}

void Robot::AutonomousPeriodic() {

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  if (drive_controller->set_profile) {
  //  drive_controller->RunAutonDrive();
  }

}

void Robot::TeleopInit() {

  drive_controller->ZeroAll(true);

}

void Robot::TeleopPeriodic() {

    is_heading = joyThrottle->GetRawButton(0);
		is_vision = false;
		is_fc = false;

   drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_heading);

   //frc::SmartDashboard::PutNumber("L1", )
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
