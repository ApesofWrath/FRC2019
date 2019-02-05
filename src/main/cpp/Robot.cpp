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

  arm_mp = new ArmMotionProfiler(2.0, 10.0, 0.02);
//  elev_mp = new ElevatorMotionProfiler(1.15, 5.0, 0.02);

  arm = new Arm(arm_mp); //take out pdp
//  elev = new Elevator(elevator_mp);

  joyOp = new frc::Joystick(0);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {


}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
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
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    // arm->ManualArm(joyOp);


    bool arm_low = joyOp->GetRawButton(3);
    bool arm_down = joyOp->GetRawButton(4);
    bool arm_high = joyOp->GetRawButton(5);
    bool arm_mid = joyOp->GetRawButton(6);

    frc::SmartDashboard::PutBoolean("arm_up",arm_low);
    frc::SmartDashboard::PutBoolean("arm_down", arm_down);

    if (arm_low) {
      arm->arm_state = arm->LOW_CARGO_STATE_H;
    } else if (arm_down) {
      arm->arm_state = arm->DOWN_STATE_H;
    } else if (arm_high) {
      arm->arm_state = arm->HIGH_CARGO_STATE_H;
    } else if (arm_mid) {
      arm->arm_state = arm->MID_CARGO_STATE_H;
    }

    arm->ArmStateMachine();

    arm->Rotate();

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
