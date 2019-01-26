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

  joyThrottle = new frc::Joystick(JOY_THROTTLE);

  canTalonTopIntake = new TalonSRX(0);
  canTalonBottomIntake = new TalonSRX(1);

  suctionIn = new frc::DigitalOutput(2);
  suctionOut = new frc::DigitalOutput(3);

  //doubleSolenoid1 = new DoubleSolenoid(2, 4);
//  doubleSolenoid2 = new DoubleSolenoid(3, 6);

  frc::DoubleSolenoid doubleSolenoid1 {2, 4};
  frc::DoubleSolenoid doubleSolenoid2 {3, 6};


}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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

  bool bottom_intake_in = joyThrottle->GetRawButton(BOTTOM_INTAKE_IN);
  bool bottom_intake_out = joyThrottle->GetRawButton(BOTTOM_INTAKE_OUT);
  bool top_intake_in = joyThrottle->GetRawButton(TOP_INTAKE_IN);
  bool top_intake_out = joyThrottle->GetRawButton(TOP_INTAKE_OUT);
  bool suction_in = joyThrottle->GetRawButton(SUCTION_IN);
  bool suction_release = joyThrottle->GetRawButton(SUCTION_RELEASE);

  if (bottom_intake_in) {
    canTalonBottomIntake->Set(ControlMode::PercentOutput, 0.3);
  } else if (bottom_intake_out) {
    canTalonBottomIntake->Set(ControlMode::PercentOutput, -0.3);
  } else {
    canTalonBottomIntake->Set(ControlMode::PercentOutput, 0.0);
  }

  if (top_intake_in) {
    canTalonTopIntake->Set(ControlMode::PercentOutput, 0.3);
  } else if (top_intake_out) {
    canTalonTopIntake->Set(ControlMode::PercentOutput, -0.3);
  } else {
    canTalonTopIntake->Set(ControlMode::PercentOutput, 0.0);
  }

  if (suction_in) {
    suctionIn->Set(true);
    doubleSolenoid1->Set(frc::DoubleSolenoid::kForward);
    doubleSolenoid2->Set(frc::DoubleSolenoid::kForward);

  } else if (suction_release) {
    suctionOut->Set(true);
    doubleSolenoid1->Set(frc::DoubleSolenoid::kReverse);
    doubleSolenoid2->Set(frc::DoubleSolenoid::kReverse);

  } else {
    doubleSolenoid1->Set(frc::DoubleSolenoid::kOff);
    doubleSolenoid2->Set(frc::DoubleSolenoid::kOff);

  }




}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
