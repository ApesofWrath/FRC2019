#include "Intake.h"

const int STOP_STATE = 0;
const int HOLD_STATE = 1;
const int IN_STATE = 2;
const int OUT_STATE = 3;
const int OUT_SLOW_STATE = 4;
const int IN_SLOW_STATE = 5;
const int OUT_FAST_STATE = 6;

int out_counter = 0;

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

  talonIntake1->Set(ControlMode::PercentOutput, -1.0); //-.6

}

void Intake::OutTop(bool slow) {

  if (!slow) {
  talonIntake1->Set(ControlMode::PercentOutput, 0.65);
} else {
  talonIntake1->Set(ControlMode::PercentOutput, 0.72);
}

}

void Intake::OutFastTop(bool fast) {
  if (fast) {
  talonIntake1->Set(ControlMode::PercentOutput, 0.80);
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
  out_counter++;
if (!slow) {
  talonIntake2->Set(ControlMode::PercentOutput, 0.4);
} else {
  talonIntake2->Set(ControlMode::PercentOutput, 0.72);
}

}

void Intake::OutFastBottom(bool fast) {
  if (fast) {
  talonIntake2->Set(ControlMode::PercentOutput, 0.80);
} else {
  talonIntake2->Set(ControlMode::PercentOutput, 0.72);
}

}

void Intake::IntakeTopStateMachine() { //TODO: add current limit?

    frc::SmartDashboard::PutString("TOP INTAKE state", "yes");

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

    case OUT_FAST_STATE:
    OutFastTop(true);
    frc::SmartDashboard::PutString("TOP INTAKE", "fast out");
    break;

  }

}

void Intake::IntakeBottomStateMachine() { //TODO: add current limit?

  frc::SmartDashboard::PutString("BOT INTAKE state", "yes");

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
    frc::SmartDashboard::PutString("BOT INTAKE", "in");
    break;

    case OUT_FAST_STATE:
    OutFastBottom(true);
    frc::SmartDashboard::PutString("BOT INTAKE", "fast out");
    break;

  }

}

bool Intake::HaveBall() {

  frc::SmartDashboard::PutNumber("top cur",talonIntake1->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("bot cur",talonIntake2->GetOutputCurrent());

  if ((talonIntake1->GetOutputCurrent() > 13.0) && (talonIntake2->GetOutputCurrent() > 15.0)) { // has to be top because bottom spikes if it hits the ground
    return true;
  } else {
    return false;
  }

}

bool Intake::ReleasedBall() {
  frc::SmartDashboard::PutNumber("top cur",talonIntake1->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("bot cur",talonIntake2->GetOutputCurrent());

  if (out_counter > 20) {
    out_counter = 0;
    return true;
  } else {
    return false;
  }

  return false;
}
