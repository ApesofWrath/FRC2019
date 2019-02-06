/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

const int ELEVATOR_UP = 3; //left lower
const int ELEVATOR_MID = 5; //left upper
const int ELEVATOR_DOWN = 4; //right lower

bool elevator_up, elevator_mid, elevator_down;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  elevator_mp = new ElevatorMotionProfiler(1.15, 5.0, 0.02);
  elevator = new Elevator(elevator_mp);
  joyElev = new frc::Joystick(0);
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
  elevator->PrintElevatorInfo();
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

  elevator->ElevatorStateMachine();
  // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.
  elevator_up = joyElev->GetRawButton(ELEVATOR_UP);
	elevator_mid = joyElev->GetRawButton(ELEVATOR_MID);
	elevator_down = joyElev->GetRawButton(ELEVATOR_DOWN);

  frc::SmartDashboard::PutBoolean("elevator_up", elevator_up);
  frc::SmartDashboard::PutBoolean("elevator_mid", elevator_mid);
  frc::SmartDashboard::PutBoolean("elevator_down", elevator_down);

  if (elevator_up) {
    elevator->elevator_state = elevator->ROCKET_TOP_CARGO;
  }
  if (elevator_mid) {
    elevator->elevator_state = elevator->ROCKET_MID_CARGO;
  }
  if (elevator_down) {
    elevator->elevator_state = elevator->ROCKET_BOTTOM_CARGO;
  }

  elevator->Move();


  // elevator->ManualElevator(joyElev);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
