#include "Intake.h"

const int STOP_STATE = 0;
const int HOLD_STATE = 1;
const int IN_STATE = 2;
const int OUT_STATE = 3;
const int OUT_SLOW_STATE = 4;
const int IN_SLOW_STATE = 5;

Intake::Intake() {

  talonIntake1 = new TalonSRX(TALON_ID_TOP);
  talonIntake2 = new TalonSRX(TALON_ID_BOT);

}

void Intake::StopTop() {

  talonIntake1->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::HoldTop() {

  talonIntake1->Set(ControlMode::PercentOutput, 0.0 - 0.2);

}

void Intake::InTop() {

  talonIntake1->Set(ControlMode::PercentOutput, -1.0);

}

void Intake::OutTop(bool slow) {

  if (!slow) {
  talonIntake1->Set(ControlMode::PercentOutput, 1.0);
} else {
  talonIntake1->Set(ControlMode::PercentOutput, 0.72);
}

}

void Intake::StopBottom() {

  talonIntake2->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::HoldBottom() {

  talonIntake2->Set(ControlMode::PercentOutput, 0.0 - 0.2);

}

void Intake::InBottom(bool slow) {

  if (!slow) {
  talonIntake2->Set(ControlMode::PercentOutput, -1.0);
} else {
  talonIntake2->Set(ControlMode::PercentOutput, -0.6);
}

}

void Intake::OutBottom(bool slow) {
if (!slow) {
  talonIntake2->Set(ControlMode::PercentOutput, 1.0);
} else {
    talonIntake2->Set(ControlMode::PercentOutput, 0.72);
}

}

void Intake::IntakeTopStateMachine() { //TODO: add current limit?

  switch (top_intake_state) {

    case STOP_STATE:
    StopTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "stop");
    break;

    case HOLD_STATE:
    HoldTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "hold");
    break;

    case IN_STATE:
    InTop();
    frc::SmartDashboard::PutString("TOP INTAKE", "in");
    break;

    case OUT_STATE:
    OutTop(false);
    frc::SmartDashboard::PutString("TOP INTAKE", "out");
    break;

    case OUT_SLOW_STATE:
    OutTop(true);
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

    case HOLD_STATE:
    HoldBottom();
    frc::SmartDashboard::PutString("BOT INTAKE", "hold");
    break;

    case IN_STATE:
    InBottom(false);
    frc::SmartDashboard::PutString("BOT INTAKE", "in");
    break;

    case OUT_STATE:
    OutBottom(false);
    frc::SmartDashboard::PutString("BOT INTAKE", "out");
    break;

    case OUT_SLOW_STATE:
    OutBottom(true);
    frc::SmartDashboard::PutString("BOT INTAKE", "out");
    break;

    case IN_SLOW_STATE:
    InBottom(true);
    break;

  }

}

bool Intake::HaveBall() {

  frc::SmartDashboard::PutNumber("top cur",talonIntake1->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("bot cur",talonIntake2->GetOutputCurrent());

  if ((talonIntake1->GetOutputCurrent() > 3.5)) {
    return true;
  } else {
    return false;
  }

}

bool Intake::ReleasedBall() {

  frc::SmartDashboard::PutNumber("top cur",talonIntake1->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("bot cur",talonIntake2->GetOutputCurrent());

  if ((talonIntake1->GetOutputCurrent() < 1.5)) {
    return true;
  } else {
    return false;
  }

  return false;
}
