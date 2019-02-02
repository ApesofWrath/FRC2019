#include "HatchPickup.h"

const int PULL_STATE = 0;
const int PUSH_STATE = 1;

HatchPickup::HatchPickup() {

  suction = new frc::DigitalOutput(SUCTION_CHANNEL);

}

void HatchPickup::Pull() {

  suction->Set(true);

}

void HatchPickup::Push() {

  suction->Set(false);

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
