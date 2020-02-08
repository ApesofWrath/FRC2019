/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Arm.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

const int REST_STATE = 0;
const int UP_STATE = 1;
const int DOWN_STATE = 2;

Arm::Arm() {

 talonArm = new TalonSRX(1);

}

void Arm::Up() {

  talonArm->Set(ControlMode::PercentOutput, 0.3);

}

void Arm::Down() {

  talonArm->Set(ControlMode::PercentOutput, -0.3);

}

void Arm::Rest() {

  talonArm->Set(ControlMode::PercentOutput, 0.0);

}

void Arm::IntakeArmStateMachine() {

  frc::SmartDashboard::PutString("INTAKE ARM STATE", "yes");

  switch(intake_arm_state){

    case REST_STATE:
    Rest();
    frc::SmartDashboard::PutString("INTAKE ARM", "rest");
    break;

    case UP_STATE:
    Up();
    frc::SmartDashboard::PutString("INTAKE ARM", "up");
    break;

    case DOWN_STATE:
    Down();
    frc::SmartDashboard::PutString("INTAKE ARM", "down");
    break;

  }

}




#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
