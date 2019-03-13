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

    frc::SmartDashboard::PutNumber("arb ff", 0);

  auton_chooser.SetDefaultOption(kDoNothing, kDoNothing);
  auton_chooser.AddOption(kDriveForward, kDriveForward);
  auton_chooser.AddOption(kCenterLowHabOneCargo, kCenterLowHabOneCargo);
  frc::SmartDashboard::PutData("Auto Modes", &auton_chooser);

  start_chooser.SetDefaultOption(kCenter, kCenter);
  start_chooser.AddOption(kLeft, kLeft);
  start_chooser.AddOption(kRight, kRight);
  frc::SmartDashboard::PutData("Start Placement", &start_chooser);

  pdp = new PowerDistributionPanel(0); // 0 is the vcm

  led_solenoid = new Solenoid(9, 2);

  joyThrottle = new frc::Joystick(0);
  joyWheel = new frc::Joystick(1);
  joyOp1 = new frc::Joystick(2);
  joyOp2 = new frc::Joystick(3);
  joyOpButtons = new frc::Joystick(4);

  drive_controller = new DriveController();
  vision = new Vision();
//  start = {0,0,0};
//  move_forward = new MoveForward(start);

  //  elevator_profiler = new ElevatorMotionProfiler(0.5, 2.0, 0.02);
  elevator = new Elevator(elevator_profiler);
  //  arm_profiler = new ArmMotionProfiler(1.0, 5.0, 0.02);
  arm = new Arm(arm_profiler);
  intake = new Intake();
  hatch_pickup = new HatchPickup();

  tsm = new TeleopStateMachine(drive_controller, elevator, intake, arm, hatch_pickup);

}

void Robot::RobotPeriodic() {
//1.63 hatch scorings
//1.05 ball scoring
//0.05 hatch ground pickup, ball ground pickup
//   frc::SmartDashboard::PutNumber("top!", intake->talonIntake1->GetOutputCurrent());
// frc::SmartDashboard::PutNumber("bot!", intake->talonIntake2->GetOutputCurrent());
// frc::SmartDashboard::PutNumber("arm slider", joyThrottle->GetRawAxis(3));
// frc::SmartDashboard::PutNumber("suction2",hatch_pickup->suction2->GetOutputCurrent());
// frc::SmartDashboard::PutNumber("suction1",hatch_pickup->suction1->GetOutputCurrent());
// frc::SmartDashboard::PutBoolean("have hatch?", hatch_pickup->HaveHatch());
//
//   frc::SmartDashboard::PutString("have hatch", "n");
  frc::SmartDashboard::PutNumber("el vel", elevator->talonElevator1->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("el pos", elevator->talonElevator1->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("el targ pos", elevator->talonElevator1->GetActiveTrajectoryPosition());
  frc::SmartDashboard::PutNumber("el targ vel", elevator->talonElevator1->GetActiveTrajectoryVelocity());

  frc::SmartDashboard::PutNumber("arm vel", arm->talonArm->GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("arm pos", arm->talonArm->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("arm targ pos", arm->talonArm->GetActiveTrajectoryPosition());
  frc::SmartDashboard::PutNumber("arm targ vel", arm->talonArm->GetActiveTrajectoryVelocity());

  frc::SmartDashboard::PutNumber("arm ang", arm->GetAngularPosition());
  frc::SmartDashboard::PutNumber("elev height", elevator->GetElevatorPosition());

    frc::SmartDashboard::PutNumber("arm error", arm->talonArm->GetActiveTrajectoryPosition() - arm->talonArm->GetSelectedSensorPosition(0));
}

void Robot::AutonomousInit() {

  //  drive_controller->ZeroAll(true);
  tsm->Initialize();
  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);

}

void Robot::AutonomousPeriodic() {


  //  elevator->talonElevator1->Set(ControlMode::PercentOutput, joyThrottle->GetY());

   //arm->talonArm->Set(ControlMode::PercentOutput, joyThrottle->GetRawAxis(3));
    led_solenoid->Set(true);
    is_rotation = true;
    is_vision = false;
    is_regular = false;

    drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);

    //frc::SmartDashboard::PutNumber("CUR", hatch_pickup->suction1->GetOutputCurrent());
    elevator->ElevatorStateMachine();
    arm->ArmStateMachine();
    intake->IntakeTopStateMachine();
    intake->IntakeBottomStateMachine();
    hatch_pickup->SuctionStateMachine();
    hatch_pickup->SolenoidStateMachine();

    hatch_out = joyOp1->GetRawButton(1);
    bottom_intake_in  = joyOp1->GetRawButton(3);
    bottom_intake_out  =  joyOp1->GetRawButton(2);
    bottom_intake_stop = joyOp1->GetRawButton(4);
    get_hatch_station =  joyOpButtons->GetRawButton(12);
//    post_intake_hatch = joyOp1->GetRawButton(1);

    place_hatch_low = joyOpButtons->GetRawButton(15);
    place_hatch_mid = joyOpButtons->GetRawButton(2);
    place_hatch_high = joyOpButtons->GetRawButton(8);
    extra_button = joyOp1->GetRawButton(11);
    post_outtake_hatch = joyOp1->GetRawButton(2);

    wait_for_button = joyOpButtons->GetRawButton(13);
  //bigger outtake cargo speed
    // top_intake_in =  joyOp1->GetRawButton(5);
    // top_intake_out = joyOp1->GetRawButton(6);
    // top_intake_stop = joyOp1->GetRawButton(7);
    // suction_on = joyOp1->GetRawButton(8);
    // suction_off = joyOp1->GetRawButton(9);
     arm_up = joyOpButtons->GetRawButton(6);
     post_intake_hatch = joyOpButtons->GetRawButton(5);

    elevator_hatch_mid = joyOp1->GetRawButton(12);

    get_cargo_ground = joyOpButtons->GetRawButton(14);
    //get_cargo_station = joyOpButtons->GetRawButton(8);
    post_intake_hatch = joyOpButtons->GetRawButton(1);
    place_cargo_bay = joyOpButtons->GetRawButton(16); //same as mid
    place_cargo_low =  joyOpButtons->GetRawButton(9);
    place_cargo_mid =  joyOpButtons->GetRawButton(16);
    place_cargo_high = joyOpButtons->GetRawButton(3);
    //have shot ball

    top_intake_in = joyOpButtons->GetRawButton(5);
  post_intake_cargo = joyOpButtons->GetRawButton(4);



    tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop,
      suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_mid, arm_high_cargo, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low,
      elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo_ground, get_cargo_station, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch,
      place_hatch_high, place_hatch_mid, place_hatch_low, place_cargo_high, place_cargo_mid, place_cargo_low, place_cargo_bay, place_cargo_bay_fast,
      post_outtake_hatch, post_outtake_cargo, extra_button);
      // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.


}

void Robot::TeleopInit() {
  //tsm->Initialize();
  drive_controller->set_profile = false; //prep for visiondrive
  drive_controller->ZeroAll(true);
}

void Robot::TeleopPeriodic() {

//  elevator->talonElevator1->Set(ControlMode::PercentOutput, joyThrottle->GetY());

 //arm->talonArm->Set(ControlMode::PercentOutput, joyThrottle->GetRawAxis(3));
  led_solenoid->Set(true);
  is_rotation = joyWheel->GetRawButton(5);
  is_vision = false;
  //is_regular = true;

  drive_controller->RunTeleopDrive(joyThrottle, joyWheel, is_regular, is_vision, is_rotation);

  //frc::SmartDashboard::PutNumber("CUR", hatch_pickup->suction1->GetOutputCurrent());
  elevator->ElevatorStateMachine();
  arm->ArmStateMachine();
  intake->IntakeTopStateMachine();
  intake->IntakeBottomStateMachine();
  hatch_pickup->SuctionStateMachine();
  hatch_pickup->SolenoidStateMachine();

  // hatch_out = joyOp1->GetRawButton(1);
  // bottom_intake_in  = joyOp1->GetRawButton(3);
  // bottom_intake_out  =  joyOp1->GetRawButton(2);
  // bottom_intake_stop = joyOp1->GetRawButton(4);
  get_hatch_station =  joyOpButtons->GetRawButton(12);
  post_intake_hatch = joyOp1->GetRawButton(1);

  place_hatch_low = joyOpButtons->GetRawButton(15);
  place_hatch_mid = joyOpButtons->GetRawButton(2);
  place_hatch_high = joyOpButtons->GetRawButton(8);
  extra_button = joyOp1->GetRawButton(11);
  post_outtake_hatch = joyOp1->GetRawButton(2);

  wait_for_button = joyOpButtons->GetRawButton(13);
//bigger outtake cargo speed
  // top_intake_in =  joyOp1->GetRawButton(5);
  // top_intake_out = joyOp1->GetRawButton(6);
  // top_intake_stop = joyOp1->GetRawButton(7);
  // suction_on = joyOp1->GetRawButton(8);
  // suction_off = joyOp1->GetRawButton(9);
   arm_up = joyOpButtons->GetRawButton(6);
   arm_mid = joyOpButtons->GetRawButton(5);

//  elevator_hatch_mid = joyOp1->GetRawButton(12);

  get_cargo_ground = joyOpButtons->GetRawButton(14);
  //get_cargo_station = joyOpButtons->GetRawButton(8);
  post_intake_cargo = joyOpButtons->GetRawButton(4);
  place_cargo_bay = joyOpButtons->GetRawButton(16); //same as mid
  place_cargo_low =  joyOpButtons->GetRawButton(9);
  place_cargo_mid =  joyOpButtons->GetRawButton(16);
  place_cargo_high = joyOpButtons->GetRawButton(3);
  //have shot ball

  top_intake_in = joyOpButtons->GetRawButton(5);


  tsm->StateMachine(wait_for_button, bottom_intake_in, bottom_intake_out, bottom_intake_stop, top_intake_in, top_intake_out, top_intake_stop,
    suction_on, suction_off, hatch_out, hatch_in, arm_up, arm_mid, arm_high_cargo, arm_down, elevator_hatch_up, elevator_hatch_mid, elevator_hatch_low,
    elevator_cargo_up, elevator_cargo_mid, elevator_cargo_low, get_cargo_ground, get_cargo_station, get_hatch_ground, get_hatch_station, post_intake_cargo, post_intake_hatch,
    place_hatch_high, place_hatch_mid, place_hatch_low, place_cargo_high, place_cargo_mid, place_cargo_low, place_cargo_bay, place_cargo_bay_fast,
    post_outtake_hatch, post_outtake_cargo, extra_button);
    // set those buttons to change the states in ElevatorStateMachine. Use if/else statements. Ask me if you don't understand what to do.

  }

  void Robot::DisabledInit() {
   tsm->Disabled();
   }

  void Robot::TestPeriodic() {}

  //#ifndef RUNNING_FRC_TESTS
  int main() { return frc::StartRobot<Robot>(); }
