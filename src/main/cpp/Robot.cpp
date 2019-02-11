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
  vision = new Vision();

}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() { //not go to periodic until prof sent to dc

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

  //  drive_controller->RunAutonDrive();

}

void Robot::TeleopInit() {

  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);

}

double previous_yaw = 0.0;

void Robot::TeleopPeriodic() {

    is_rotation = joyThrottle->GetRawButton(-1);
		is_vision = joyThrottle->GetRawButton(-1);
		is_regular = joyThrottle->GetRawButton(-1);
    is_regular = true;

    drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);

    double y = joyThrottle->GetY();
    // double x = joyWheel->GetX();

    //  drive_controller->canTalonLeft1->Set(ControlMode::PercentOutput, x);
    //  drive_controller->canTalonRight1->Set(ControlMode::PercentOutput, x);

     // double yaw_rate = drive_controller->ahrs->GetRate();//abs(previous_yaw - drive_controller->GetYawPos()) / 0.020;

     // frc::SmartDashboard::PutNumber("YAW RATE", yaw_rate);
     // frc::SmartDashboard::PutNumber("X", x);

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
