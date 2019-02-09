#include "Intake.h"

const int STOP_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;

Intake::Intake() {

  talonIntake1 = new TalonSRX(TALON_ID_TOP);
  talonIntake2 = new TalonSRX(TALON_ID_BOT);

}

void Intake::StopTop() {

  talonIntake1->Set(ControlMode::PercentOutput, 0.0 - STOP_TOP_SPEED);

}

void Intake::InTop() {

  talonIntake1->Set(ControlMode::PercentOutput, -IN_TOP_SPEED);

}

void Intake::OutTop() {

  talonIntake1->Set(ControlMode::PercentOutput, OUT_TOP_SPEED);

}

void Intake::StopBottom() {

  talonIntake2->Set(ControlMode::PercentOutput, 0.0 - STOP_BOTTOM_SPEED);

}

void Intake::InBottom() {

  talonIntake2->Set(ControlMode::PercentOutput, -IN_BOTTOM_SPEED);

}

void Intake::OutBottom() {

  talonIntake2->Set(ControlMode::PercentOutput, OUT_BOTTOM_SPEED);

}

void Intake::IntakeTopStateMachine() { //TODO: add current limit?

  switch (top_intake_state) {

    case STOP_STATE:
    StopTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "stop");
    break;

    case IN_STATE:
    InTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "in");
    break;

    case OUT_STATE:
    OutTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "out");
    break;

  }

}

void Intake::IntakeBottomStateMachine() { //TODO: add current limit?

  switch (bottom_intake_state) {

    case STOP_STATE:
    StopBottom();
    frc::SmartDashboard::PutString("BOT INTAKE", "stop");
    break;

    case IN_STATE:
    InBottom();
    frc::SmartDashboard::PutString("BOT INTAKE", "in");
    break;

    case OUT_STATE:
    OutBottom();
    frc::SmartDashboard::PutString("BOT INTAKE", "out");
    break;

  }

}

bool Intake::HaveBall() {

}

bool Intake::ReleasedBall() {

}
