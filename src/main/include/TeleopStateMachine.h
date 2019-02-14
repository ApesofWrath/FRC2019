#ifndef SRC_TELEOPSTATE_MACHINE_H_
#define SRC_TELEOPSTATE_MACHINE_H_
//frc and 3rd party library includes
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/WPILib.h>

// Includes of our code
#include "Arm.h"
#include "Elevator.h"
#include "Intake.h"
#include "HatchPickup.h"
#include "Elevator.h"

class TeleopStateMachine {
public:

  TeleopStateMachine(Elevator *elevator_, Intake *intake_,
      Arm *arm_, HatchPickup *hatch_pickup_);
  void StateMachine(bool wait_for_button, bool bottom_intake_in, bool bottom_intake_out,
      bool bottom_intake_stop, bool top_intake_in, bool top_intake_out, bool top_intake_stop,
      bool suction_on, bool suction_off, bool hatch_out, bool hatch_in, bool arm_up, bool arm_down,
      bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low, bool elevator_cargo_up,
      bool elevator_cargo_mid, bool elevator_cargo_low, bool get_cargo, bool get_hatch_ground,
      bool get_hatch_station, bool post_intake_cargo, bool post_intake_hatch, bool place_hatch,
      bool place_cargo, bool post_outtake_hatch, bool post_outtake_cargo);

  void Initialize();

  Elevator *elevator;
  Intake *intake;
  Arm *arm;
  HatchPickup *hatch_pickup;

  const int INIT_STATE_H = 0;
  const int WAIT_FOR_BUTTON_STATE_H = 1;
  const int GET_HATCH_STATION_STATE_H = 2;
  const int GET_HATCH_GROUND_STATE_H = 3;
  const int POST_INTAKE_HATCH_STATE_H = 4;
  const int GET_CARGO_STATE_H = 5;
  const int POST_INTAKE_CARGO_STATE_H = 6;
  const int PLACE_HATCH_STATE_H = 7;
  const int PLACE_CARGO_STATE_H = 8;
  const int POST_OUTTAKE_HATCH_STATE_H = 9;
  const int POST_OUTTAKE_CARGO_STATE_H = 10;

  int state = INIT_STATE

};
#endif
