/*
    suction.cpp
    Created on: Jan 7th 2019
*/

#include <WPILib.h>
#include <frc/DigitalOutput.h>

/*Variables + constants*/

const int INIT_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;


Suction::Suction(int channel) {
  suction_out = DigitalOutput(channel);
}

void Suction::Pull() {
  suction_out->Set(true);
}

void Suction:Push() {
  suction_out->Set(true);
}

void Suction:SuctionStateMachine() {
    switch(suction_state) {
        case INIT_STATE:
            break;
        case PULL_STATE:
            break;
        case PUSH_STATE:
            break;
        case SWITCH_STATE:
            break;
    }
}
