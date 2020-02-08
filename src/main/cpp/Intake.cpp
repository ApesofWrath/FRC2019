/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Intake.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

const int STOP_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;

Intake::Intake() {

 talonIntake = new TalonSRX(0);

}

void Intake::Stop() {

  talonIntake->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::In() {

  talonIntake->Set(ControlMode::PercentOutput, 0.3);

}

void Intake::Out() {

  talonIntake->Set(ControlMode::PercentOutput, -0.3);

}

void Intake::IntakeStateMachine() {

  frc::SmartDashboard::PutString("INTAKE STATE", "yes");

  switch(intake_state) {

    case STOP_STATE:
    Stop();
    frc::SmartDashboard::PutString("INTAKE", "stop");
    break;

    case IN_STATE:
    In();
    frc::SmartDashboard::PutString("INTAKE", "in");
    break;

    case OUT_STATE:
    Out();
    frc::SmartDashboard::PutString("INTAKE", "out");
    break;

  }

}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
