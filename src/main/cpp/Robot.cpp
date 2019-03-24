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

  led_solenoid = new Solenoid(3, 3);

  drive_controller = new DriveController();
  vision = new Vision();
  led_solenoid->Set(true);
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

   drive_controller->RunAutonDrive();

}

void Robot::TeleopInit() {

  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);

}

void Robot::TeleopPeriodic() {

    is_rotation = joyThrottle->GetRawButton(3);
		is_vision = joyThrottle->GetRawButton(1);
		is_regular = joyThrottle->GetRawButton(2);

    is_vision = true;
  //   double y = joyThrottle->GetY();
  //
  //    drive_controller->canTalonLeft1->Set(ControlMode::PercentOutput, -y);
  // 	   drive_controller->canTalonLeft2->Set(ControlMode::PercentOutput, -y);
  // 	   drive_controller->canTalonLeft3->Set(ControlMode::PercentOutput, -y);
  //   //
  // 	   drive_controller->canTalonRight1->Set(ControlMode::PercentOutput, y);
  // 	   drive_controller->canTalonRight2->Set(ControlMode::PercentOutput, y);
  // 	   drive_controller->canTalonRight3->Set(ControlMode::PercentOutput, y);


  drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);
  // drive_controller->VisionDriveStateMachine();
  // drive_controller->GenerateVisionProfile(3.28, d2r(0), d2r(45));

  // frc::SmartDashboard::PutNumber("L1", drive_controller->GetLeftVel());
  //  // frc::SmartDashboard::PutNumber("L2", drive_controller->canTalonLeft2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("R1", drive_controller->canTalonRight1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("R2", drive_controller->canTalonRight2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("R3", drive_controller->canTalonRight3->GetOutputCurrent());
  //
  //  frc::SmartDashboard::PutNumber("L1", drive_controller->canTalonLeft1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("L2", drive_controller->canTalonLeft2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("L3", drive_controller->canTalonLeft3->GetOutputCurrent());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
