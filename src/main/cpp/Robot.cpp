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
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

//  elevator_profiler = new ElevatorMotionProfiler(0.5, 2.0, 0.02);
  elevator = new Elevator(elevator_profiler);
//  arm_profiler = new ArmMotionProfiler(1.0, 5.0, 0.02);
  arm = new Arm(arm_profiler);
  intake = new Intake();
  hatch_pickup = new HatchPickup();

  tsm = new TeleopStateMachine(elevator, intake, arm, hatch_pickup);
  joyOp = new frc::Joystick(0);
  joyWheel = new frc::Joystick(1);

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

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  tsm->Initialize();
}

void Robot::TeleopPeriodic() {


  elevator->ElevatorStateMachine();
  arm->ArmStateMachine();
  intake->IntakeTopStateMachine();
  intake->IntakeBottomStateMachine();
  hatch_pickup->SuctionStateMachine();
  hatch_pickup->SolenoidStateMachine();

  //elevator
  elevator_hatch_up = joyOp->GetRawButton(1);
  elevator_hatch_mid = joyOp->GetRawButton(2); //doesnt work
  elevator_hatch_low = joyOp->GetRawButton(3);
  // elevator_cargo_up = joyOp->GetRawButton(4);
  // elevator_cargo_mid  = joyOp->GetRawButton(5);
  // elevator_cargo_low = joyOp->GetRawButton(6);



  // suction

  suction_on =  joyOp->GetRawButton(7);
  suction_off =  joyOp->GetRawButton(8);


  // hatch -solenoid
  hatch_in = joyWheel->GetRawButton(1);
  hatch_out = joyWheel->GetRawButton(2);


  // arm
  arm_up = joyWheel->GetRawButton(3); //hatch
  arm_mid
   = joyWheel->GetRawButton(4);//e

arm_driving = joyWheel->GetRawButton(12);
//  arm_down = joyWheel->GetRawButton(5);//cargo

  // intakes //top and bottom switch, bottom switch in/out
  bottom_intake_in = joyWheel->GetRawButton(6);//switch
  bottom_intake_out = joyWheel->GetRawButton(7);
  bottom_intake_stop = joyWheel->GetRawButton(8);
  top_intake_in  = joyWheel->GetRawButton(9);
  top_intake_out  = joyWheel->GetRawButton(10);
  top_intake_stop = joyWheel->GetRawButton(11);
get_hatch_ground = joyOp->GetRawButton(12);
post_intake_hatch = joyOp->GetRawButton(4);
place_hatch_low = joyOp->GetRawButton(5);
place_hatch_mid = joyOp->GetRawButton(6);
post_outtake_hatch = joyWheel->GetRawButton(5);

  tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop,
    suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_mid, arm_driving, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low,
    elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch,
    place_hatch_high, place_hatch_mid, place_hatch_low, place_cargo_high, place_cargo_mid, place_cargo_low, post_outtake_hatch, post_outtake_cargo);
  // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.
}

void Robot::TestPeriodic() {}

//#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
