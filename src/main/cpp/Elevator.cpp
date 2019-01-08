#include "Elevator.h"

int currentState;
// top level of state machine
  // top, bottom, mid
// sub states
  //top
    // cargo, hatch
  // mid
    //cargo, hatch
  // bottom
    // cargo_rocket, cargo_hatch, hatch (same height and action on cargo and cargo ship)
const int TOP_STATE = 0;
const int MID_STATE = 1
const int BOTTOM_STATE = 2;

// Elevator vs. Arm vs. intake
  // Where does it go?

Elevator::Elevator() {

}

void Elevator::ElevatorStateMachine() {
  frc::SmartDashboard::PutString("ElevatorState: ", currentState);
  switch (currentState) {
    case BOTTOM_STATE:

    break;

    case MID_STATE:

    break;

    case TOP_STATE:

    break;
  }
}
