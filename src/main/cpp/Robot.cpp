/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {

  auton_chooser.SetDefaultOption(kDoNothing, kDoNothing);
  auton_chooser.AddOption(kDriveForward, kDriveForward);
  auton_chooser.AddOption(kCenterLowHabOneCargo, kCenterLowHabOneCargo);
  frc::SmartDashboard::PutData("Auto Modes", &auton_chooser);

  start_chooser.SetDefaultOption(kCenter, kCenter);
  start_chooser.AddOption(kLeft, kLeft);
  start_chooser.AddOption(kRight, kRight);
  frc::SmartDashboard::PutData("Start Placement", &start_chooser);

  pdp = new PowerDistributionPanel(3); // 0 is the vcm

  joyThrottle = new frc::Joystick(0);
  joyWheel = new frc::Joystick(1);
  joyOp1 = new frc::Joystick(2);
  joyOp2 = new frc::Joystick(3);

  drive_controller = new DriveController();
  vision = new Vision();
  start = {0,0,0};
  move_forward = new MoveForward(start);

  //  elevator_profiler = new ElevatorMotionProfiler(0.5, 2.0, 0.02);
  elevator = new Elevator(elevator_profiler);
  //  arm_profiler = new ArmMotionProfiler(1.0, 5.0, 0.02);
  arm = new Arm(arm_profiler);
  intake = new Intake();
  hatch_pickup = new HatchPickup();

  tsm = new TeleopStateMachine(elevator, intake, arm, hatch_pickup);

}

void Robot::RobotPeriodic() {

  frc::SmartDashboard::PutNumber("el vel", elevator->talonElevator1->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("el pos", elevator->talonElevator1->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("el targ pos", elevator->talonElevator1->GetActiveTrajectoryPosition());
  frc::SmartDashboard::PutNumber("el targ vel", elevator->talonElevator1->GetActiveTrajectoryVelocity());

  frc::SmartDashboard::PutNumber("arm vel", arm->talonArm->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("arm pos", arm->talonArm->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("arm targ pos", arm->talonArm->GetActiveTrajectoryPosition());
  frc::SmartDashboard::PutNumber("arm targ vel", arm->talonArm->GetActiveTrajectoryVelocity());
}

void Robot::AutonomousInit() {

  //  drive_controller->ZeroAll(true);

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

    //  drive_controller->SetAutonRefs();
  }

}

void Robot::AutonomousPeriodic() {

  drive_controller->RunAutonDrive(joyThrottle, joyOp2, is_regular, is_vision, is_rotation);

}

void Robot::TeleopInit() {
  tsm->Initialize();
  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);
}

void Robot::TeleopPeriodic() {

  is_rotation = false;
  is_vision = false;
  is_regular = true;

  drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);

  //frc::SmartDashboard::PutNumber("CUR", hatch_pickup->suction1->GetOutputCurrent());
  elevator->ElevatorStateMachine();
  arm->ArmStateMachine();
  intake->IntakeTopStateMachine();
  intake->IntakeBottomStateMachine();
  hatch_pickup->SuctionStateMachine();
  hatch_pickup->SolenoidStateMachine();

  //elevator
  //  elevator_hatch_up = joyOp1->GetRawButton(1);
  //  elevator_hatch_mid = joyOp1->GetRawButton(2); //doesnt work
  //  elevator_hatch_low = joyOp1->GetRawButton(3);
  // elevator_cargo_up = joyOp1->GetRawButton(4);
  // elevator_cargo_mid  = joyOp1->GetRawButton(5);
  // elevator_cargo_low = joyOp1->GetRawButton(6);



  // suction

  //  suction_on =  joyOp1->GetRawButton(7);
  //  suction_off =  joyOp1->GetRawButton(8);


  // hatch -solenoid
  //  hatch_in = joyOp2->GetRawButton(1);
  hatch_out = joyOp2->GetRawButton(2);


  // arm
  //arm_up = joyOp2->GetRawButton(3); //hatch
  arm_mid
  = joyOp2->GetRawButton(4);//e

  //arm_driving = joyOp2->GetRawButton(12);
  //  arm_down = joyOp2->GetRawButton(5);//cargo

  // intakes //top and bottom switch, bottom switch in/out


  //  bottom_intake_out = joyOp2->GetRawButton(7);

  //  top_intake_in  = joyOp2->GetRawButton(9);
  top_intake_out  = joyOp2->GetRawButton(10);
  top_intake_stop = joyOp2->GetRawButton(11);

  place_cargo_bay = joyOp2->GetRawButton(9);

  get_hatch_ground = joyOp1->GetRawButton(12);
  post_intake_hatch = joyOp1->GetRawButton(4);

  place_hatch_low = joyOp1->GetRawButton(5);
  place_hatch_mid = joyOp1->GetRawButton(6);
  place_hatch_high = joyOp1->GetRawButton(1);

  place_cargo_low =  joyOp1->GetRawButton(7);
  place_cargo_mid =  joyOp1->GetRawButton(8);
  place_cargo_high = joyOp2->GetRawButton(1);

  //have suctioned hatch from station
  bottom_intake_stop = joyOp2->GetRawButton(8);
  //have shot ball
  bottom_intake_in = joyOp2->GetRawButton(6);

  get_hatch_station =  joyOp2->GetRawButton(7);
  post_outtake_hatch = joyOp2->GetRawButton(5);
  extra_button = joyOp1->GetRawButton(3);
  post_intake_cargo = joyOp2->GetRawButton(3);
  get_cargo = joyOp2->GetRawButton(12);
  tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop,
    suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_mid, arm_driving, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low,
    elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch,
    place_hatch_high, place_hatch_mid, place_hatch_low, place_cargo_high, place_cargo_mid, place_cargo_low, place_cargo_bay, post_outtake_hatch, post_outtake_cargo, extra_button);
    // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.

  }

  void Robot::TestPeriodic() {}

  //#ifndef RUNNING_FRC_TESTS
  int main() { return frc::StartRobot<Robot>(); }
