#include "HatchPickup.h"

const int ON_STATE = 0;
const int OFF_STATE = 1;

const int OUT_STATE = 0;
const int IN_STATE = 1;

HatchPickup::HatchPickup() {

  suction = new frc::DigitalOutput(SUCTION_CHANNEL);
  solenoid = new frc::DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL);
}

void HatchPickup::On() {

  suction->Set(true);

}

void HatchPickup::Off() {

  suction->Set(false);

}

void HatchPickup::In() {

  solenoid->Set(frc::DoubleSolenoid::kForward);

}

void HatchPickup::Out() {

  solenoid->Set(frc::DoubleSolenoid::kReverse);

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

}

bool HatchPickup::ReleasedHatch() {

}
