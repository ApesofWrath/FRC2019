#include "../include/Elevator.h"
// RESEARCH HIERARCHICAL STATE MACHINE
    // const int TOP_STATE = 0;
    // const int MID_STATE = 1
    // const int BOTTOM_STATE = 2;
    // top level of state machine
      // top, bottom, mid
    // sub states
      //top
        // cargo, hatch
      // mid
        //cargo, hatch
      // bottom
        // cargo_rocket, cargo_hatch, hatch (same height and action on cargo and cargo ship)
// Elevator vs. Arm vs. intake
  // Where does it go?

#define PI 3.14159265

int current_state;
const int ROCKET_TOP_CARGO = 0;
const int ROCKET_MID_CARGO = 1;
const int ROCKET_BOTTOM_CARGO = 2;
const int ROCKET_TOP_HATCH = 3;
const int ROCKET_MID_HATCH = 4;
const int BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
const int BAY_CARGO = 6;
// ¿¿DIFFERNET STATES FOR LOADING STATION?? (HPS_STATE)

const int TALON_ID1 = -1;
const int TALON_ID2 = -1;

Elevator::Elevator(ElevatorMotionProfiler *elevator_profiler_) {
  current_state = BOTTOM_HATCH;

  elevator_profiler = elevator_profiler_;
}

void Elevator::ElevatorStateMachine() {
  switch (current_state) {
    case ROCKET_TOP_CARGO:

    break;

    case ROCKET_MID_CARGO:

    break;

    case ROCKET_BOTTOM_CARGO:

    break;

    case ROCKET_TOP_HATCH:

    break;

    case ROCKET_MID_HATCH:

    break;

    case BOTTOM_HATCH:

    break;

    case BAY_CARGO:

    break;
  }
}

void Elevator::ManualElevator() {

}

// Change the height based on the
void Elevator::Move() {

}

// Calculate and output to talons
void Elevator::SetVoltage() {

}

std::string Elevator::GetState() {
  return current_state;
}

void Elevator::PrintElevatorInfo() {
  frc::SmartDashboard::PutString("ElevatorState: ", current_state);
}
