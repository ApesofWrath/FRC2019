#include "Suction.h"

const int PULL_STATE = 0;
const int PUSH_STATE = 1;

Suction::Suction() {

  suction = new frc::DigitalOutput(SUCTION_CHANNEL);

}

void Suction::Pull() {

  suction->Set(true);

}

void Suction::Push() {

  suction->Set(false);

}

void Suction::SuctionStateMachine() {

    switch (suction_state) {
        case PULL_STATE:
            Pull();
            break;
        case PUSH_STATE:
            Push();
            break;
    }
}
