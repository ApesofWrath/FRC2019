#include "HatchPickup.h"

const int ON_STATE = 0;
const int OFF_STATE = 1;

const int OUT_STATE = 0;
const int IN_STATE = 1;

HatchPickup::HatchPickup() {

  suction1 = new TalonSRX(12);
  suction2 = new TalonSRX(40);
  suction2->Follow(*suction1);

  solenoid = new frc::DoubleSolenoid(9, SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL);
}

void HatchPickup::On() {

  suction1->Set(ControlMode::PercentOutput, 0.6); //.3
///  suction2->Set(ControlMode::PercentOutput, 0.3);



}

void HatchPickup::Off() {

  suction1->Set(ControlMode::PercentOutput, 0.0);
//  suction2->Set(ControlMode::PercentOutput, 0.0);

}

void HatchPickup::In() {

  solenoid->Set(frc::DoubleSolenoid::kReverse); //reversed

}

void HatchPickup::Out() {

  solenoid->Set(frc::DoubleSolenoid::kForward); //reversed

}

void HatchPickup::SuctionStateMachine() {

  switch (suction_state) {
      case ON_STATE:
          On();
          frc::SmartDashboard::PutString("SUCTION", "On");
          break;
      case OFF_STATE:
          Off();
          frc::SmartDashboard::PutString("SUCTION", "Off");
          break;
  }
}


void HatchPickup::SolenoidStateMachine() {

  switch (solenoid_state) {
    case OUT_STATE:
      Out();
      frc::SmartDashboard::PutString("SOLENOID", "Out");
      break;

    case IN_STATE:
      In();
      frc::SmartDashboard::PutString("SOLENOID", "In");
      break;
  }

}

bool HatchPickup::HaveHatch() {
  return false;
}

bool HatchPickup::ReleasedHatch() {
  return false;
}
