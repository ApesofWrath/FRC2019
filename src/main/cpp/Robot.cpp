/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

// bool wait_for_button, get_hatch_ground, get_hatch_station, post_intake_hatch,
//   get_cargo, post_intake_cargo, place_hatch, place_cargo, post_outtake_cargo,
//   post_outtake_hatch;

bool  wait_for_button,  bottom_intake_in,  bottom_intake_out,
     bottom_intake_stop,  top_intake_in,  top_intake_out,  top_intake_stop,
     suction_on,  suction_off,  hatch_out,  hatch_in,  arm_up,  arm_down,
     elevator_hatch_up,  elevator_hatch_mid,  elevator_hatch_low,  elevator_cargo_up,  elevator_cargo_mid,  elevator_cargo_low,  get_cargo,  get_hatch_ground, get_hatch_station,  post_intake_cargo,  post_intake_hatch,  place_hatch, place_cargo,  post_outtake_hatch,  post_outtake_cargo;

ElevatorMotionProfiler *elevator_profiler;
ArmMotionProfiler *arm_profiler;

Arm *arm;
Elevator *elevator;
Intake *intake;
HatchPickup *hatch_pickup;


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  elevator_profiler = new ElevatorMotionProfiler(-1, -1, -1);
  elevator = new Elevator(elevator_profiler);
  arm_profiler = new ArmMotionProfiler(-1, -1, -1);
  arm = new Arm(arm_profiler);
  intake = new Intake();
  hatch_pickup = new HatchPickup();

  tsm = new TeleopStateMachine(elevator, intake, arm, hatch_pickup);
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
  wait_for_button = joyOp->GetRawButton(1);
  get_hatch_ground = joyOp->GetRawButton(2);
  get_hatch_station = joyOp->GetRawButton(3);
  post_intake_hatch = joyOp->GetRawButton(4);
  get_cargo = joyOp->GetRawButton(5);
  post_intake_cargo = joyOp->GetRawButton(6);
  place_hatch = joyOp->GetRawButton(7);
  place_cargo = joyOp->GetRawButton(8);
  post_outtake_cargo = joyOp->GetRawButton(9);
  post_outtake_hatch = joyOp->GetRawButton(10);


  tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop, suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low, elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch, place_hatch, place_cargo, post_outtake_hatch, post_outtake_cargo);
  // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.


  // if (wait_for_button) {
  //   tsm->state = state->WAIT_FOR_BUTTON_STATE_H;
  // } else if (get_hatch_ground) {
  //   tsm->state = state->GET_HATCH_GROUND_STATE_H;
  // } else if (get_hatch_station) {
  //   tsm->state = state->GET_HATCH_STATION_STATE_H;
  // } else if (post_intake_hatch) {
  //   tsm->state = state->POST_INTAKE_HATCH_STATE_H;
  // } else if (get_cargo) {
  //   tsm->state = state->GET_CARGO_STATE_H;
  // } else if (post_intake_cargo) {
  //   tsm->state = state->POST_INTAKE_CARGO_STATE_H;
  // } else if (place_hatch) {
  //   tsm->state = state->PLACE_HATCH_STATE_H;
  // } else if (place_cargo) {
  //   tsm->state = state->PLACE_CARGO_STATE_H;
  // } else if (post_outtake_cargo) {
  //   tsm->state = state->POST_OUTTAKE_CARGO_STATE_H;
  // } else if (post_outtake_hatch) {
  //   tsm->state = state->POST_OUTTAKE_HATCH_STATE_H;
  // }


  // elevator->ManualElevator(joyElev);
}

void Robot::TestPeriodic() {}

//#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
