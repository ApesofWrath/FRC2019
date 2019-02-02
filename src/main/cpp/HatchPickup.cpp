#include "HatchPickup.h"

const int PULL_STATE = 0;
const int PUSH_STATE = 1;

const int DOWN_STATE = 0;
const int UP_STATE = 1;

HatchPickup::HatchPickup() {

  suction = new frc::DigitalOutput(SUCTION_CHANNEL);
  solenoid = new frc::DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL);
}

void HatchPickup::Pull() {

  suction->Set(true);

}

void HatchPickup::Push() {

  suction->Set(false);

}

void HatchPickup::Up() {

  solenoid->Set(frc::DoubleSolenoid::kForward);

}

void HatchPickup::Down() {

  solenoid->Set(frc::DoubleSolenoid::kReverse);

}

void HatchPickup::SuctionStateMachine() {

    switch (suction_state) {
        case PULL_STATE:
            Pull();
            frc::SmartDashboard::PutString("SUCTION", "pull");
            break;
        case PUSH_STATE:
            Push();
            frc::SmartDashboard::PutString("SUCTION", "push");
            break;
    }
}


void HatchPickup::SolenoidStateMachine() {

  switch (solenoid_state) {
    case DOWN_STATE:
      Down();
      frc::SmartDashboard::PutString("SOLENOID", "down");
      break;

    case UP_STATE:
      Up();
      frc::SmartDashboard::PutString("SOLENOID", "up");
      break;
  }
}
