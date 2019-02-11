/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {

  auton_chooser.SetDefaultOption(kDoNothing, kDoNothing);
  auton_chooser.AddOption(kDriveForward, kDriveForward);
  auton_chooser.AddOption(kCenterLowHabOneCargo, kCenterLowHabOneCargo);
  frc::SmartDashboard::PutData("Auto Modes", &auton_chooser);

  start_chooser.SetDefaultOption(kCenter, kCenter);
  start_chooser.AddOption(kLeft, kLeft);
  start_chooser.AddOption(kRight, kRight);
  frc::SmartDashboard::PutData("Start Placement", &start_chooser);

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

  m_autoSelected = auton_chooser.GetSelected();
  m_autoSelected = SmartDashboard::GetString("Auto Selector",
       kDriveForward);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kCenterLowHabOneCargo) {
    // Custom Auto goes here
  } else if (m_autoSelected == kDoNothing) {
    // Default Auto goes here
  } else if (m_autoSelected == kMoveForward) {
    move_forward = new MoveForward();
    move_forward->BuildTotalTrajectory();
    drive_controller->SetAutonRefs(move_forward->GetFullProfile());
  }

}

void Robot::AutonomousPeriodic() {

  //  drive_controller->RunAutonDrive();

}

void Robot::TeleopInit() {

  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);

}

void Robot::TeleopPeriodic() {

    is_rotation = joyThrottle->GetRawButton(-1);
		is_vision = joyThrottle->GetRawButton(-1);
		is_regular = joyThrottle->GetRawButton(-1);

  //   double y = joyThrottle->GetY();
  //
  //    drive_controller->canTalonLeft1->Set(ControlMode::PercentOutput, -y);
  // 	   drive_controller->canTalonLeft2->Set(ControlMode::PercentOutput, -y);
  // 	   drive_controller->canTalonLeft3->Set(ControlMode::PercentOutput, -y);
  //   //
  // 	   drive_controller->canTalonRight1->Set(ControlMode::PercentOutput, y);
  // 	   drive_controller->canTalonRight2->Set(ControlMode::PercentOutput, y);
  // 	   drive_controller->canTalonRight3->Set(ControlMode::PercentOutput, y);
  //
  drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);
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
