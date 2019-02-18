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

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  elevator_profiler = new ElevatorMotionProfiler(0.5, 2.0, 0.02);
  elevator = new Elevator(elevator_profiler);
  arm_profiler = new ArmMotionProfiler(1.0, 5.0, 0.02);
  arm = new Arm(arm_profiler);
  intake = new Intake();
  hatch_pickup = new HatchPickup();

  tsm = new TeleopStateMachine(elevator, intake, arm, hatch_pickup);
  joyOp = new frc::Joystick(0);
	joyWheel = new frc::Joystick(1);

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

  frc::SmartDashboard::PutNumber("el vel", elevator->talonElevator1->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("el pos", elevator->talonElevator1->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("el targ pos", elevator->talonElevator1->GetActiveTrajectoryPosition());
  frc::SmartDashboard::PutNumber("el targ vel", elevator->talonElevator1->GetActiveTrajectoryVelocity());
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

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  //elevator->talonElevator1->Set(ControlMode::MotionMagic, 4096);
  arm->talonArm->SetSelectedSensorPosition(13800, 0, 10);

}

void Robot::TeleopPeriodic() {

// double arb = cos(3.14 - arm->GetAngularPosition()) * 0.158 * -1.0 * 1.7;
//
// frc::SmartDashboard::PutNumber("arb", arb);
// frc::SmartDashboard::PutNumber("arm ang", arm->GetAngularPosition());
//
// if (joyOp->GetRawButton(1)) {
// intake->talonIntake1->Set(ControlMode::PercentOutput, 0.3); //outttake
// intake->talonIntake2->Set(ControlMode::PercentOutput, 0.3);
// } else if (joyOp->GetRawButton(2)) {
//   intake->talonIntake1->Set(ControlMode::PercentOutput, -0.3); //intake
//   intake->talonIntake2->Set(ControlMode::PercentOutput, -0.3);
// } else {
//   intake->talonIntake1->Set(ControlMode::PercentOutput, -0);
//   intake->talonIntake2->Set(ControlMode::PercentOutput, -0);
// }



// //arm->talonArm->Set(ControlMode::PercentOutput, joyOp->GetY());
// //  arm->talonArm->Set(ControlMode::PercentOutput, joyOp->GetY());
// frc::SmartDashboard::PutNumber("arm vel", arm->talonArm->GetSelectedSensorVelocity(0));
// frc::SmartDashboard::PutNumber("arm pos", arm->talonArm->GetSelectedSensorPosition(0));
//
// frc::SmartDashboard::PutNumber("arm targ pos", arm->talonArm->GetActiveTrajectoryPosition());
// frc::SmartDashboard::PutNumber("arm targ vel", arm->talonArm->GetActiveTrajectoryVelocity());
//	elevator->ElevatorStateMachine();
//	arm->ArmStateMachine();
	// intake->IntakeTopStateMachine();
	// intake->IntakeBottomStateMachine();
	// hatch_pickup->SuctionStateMachine();
	// hatch_pickup->SolenoidStateMachine();
  //
  // //elevator
  // elevator_hatch_up = joyOp->GetRawButton(1);
  // elevator_hatch_mid = joyOp->GetRawButton(2); //doesnt work
  // elevator_hatch_low = joyOp->GetRawButton(3);
  // elevator_cargo_up = joyOp->GetRawButton(4);
  // elevator_cargo_mid  = joyOp->GetRawButton(5);
  // elevator_cargo_low = joyOp->GetRawButton(6);
  //
  //
	//   // suction
	//   suction_on =  joyOp->GetRawButton(7);
	//   suction_off =  joyOp->GetRawButton(8);
  //
  //
	// // hatch -solenoid
	// hatch_in = joyWheel->GetRawButton(1);
	// hatch_out = joyWheel->GetRawButton(2);
  //
  // // arm
  // arm_up = joyWheel->GetRawButton(3);
  // arm_down = joyWheel->GetRawButton(4);
  //
  // // intakes
  // bottom_intake_in = joyWheel->GetRawButton(5);
  // bottom_intake_out = joyWheel->GetRawButton(6);
  // bottom_intake_stop = joyWheel->GetRawButton(7);
  // top_intake_in  = joyWheel->GetRawButton(8);
  // top_intake_out  = joyWheel->GetRawButton(9);
  // top_intake_stop = joyWheel->GetRawButton(10);
  //

  //tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop, suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low, elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch, place_hatch, place_cargo, post_outtake_hatch, post_outtake_cargo);
  // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.
}

void Robot::TestPeriodic() {}

//#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
