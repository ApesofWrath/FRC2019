/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
std::vector<std::vector<double>> profile;
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

  move_forward = new MoveForward();
  move_forward->BuildTotalTrajectory();
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() { //not go to periodic until prof sent to dc
drive_controller->ZeroAll(true);

  m_autoSelected = auton_chooser.GetSelected();
  m_autoSelected = SmartDashboard::GetString("Auto Selector",
       kDriveForward);//default

  m_startSelected = start_chooser.GetSelected();
  m_startSelected = SmartDashboard::GetString("Center Selector",
       kCenter); //default

       if (m_startSelected == kCenter) {
         start = { 0.0 , 0.0, 0.0 };
           frc::SmartDashboard::PutNumber("waypoint", 0);
       } else if (m_startSelected == kLeft) {
         start = { 0.0 , 0.0, 0.0 }; //TODO:
       } else if (m_startSelected == kRight) {
         start = { 0.0 , 0.0, 0.0 };
       }

  if (m_autoSelected == kCenterLowHabOneCargo) {
    // Custom Auto goes here
  } else if (m_autoSelected == kDoNothing) {
    // Default Auto goes here
     frc::SmartDashboard::PutNumber("nothing", 0);
  } else if (m_autoSelected == kDriveForward) {


  }
  drive_controller->RestartVision();
  // profile = move_forward->GetFullProfile();
  // frc::SmartDashboard::PutNumber("first", move_forward->GetFullProfile().at(1499).at(2));
  // drive_controller->SetAutonRefs(profile);
}

void Robot::AutonomousPeriodic() {
  // drive_controller->RunAutonDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);
  drive_controller->VisionDriveStateMachine();
}

void Robot::TeleopInit() {

  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);

}

double previous_yaw = 0.0;

void Robot::TeleopPeriodic() {

    is_rotation = joyThrottle->GetRawButton(3);
    is_vision = joyThrottle->GetRawButton(1);
    is_regular = joyThrottle->GetRawButton(2);

    drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);

    // double y = joyThrottle->GetY();
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
