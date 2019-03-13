#ifndef SRC_TELEOPSTATE_MACHINE_H_
#define SRC_TELEOPSTATE_MACHINE_H_
//frc and 3rd party library includes
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/WPILib.h>

// Includes of our code
#include "Drive/DriveController.h"
#include "Arm.h"
#include "Elevator.h"
#include "Intake.h"
#include "HatchPickup.h"
#include "Elevator.h"

class TeleopStateMachine {
public:

  TeleopStateMachine(DriveController *drive_, Elevator *elevator_, Intake *intake_,
      Arm *arm_, HatchPickup *hatch_pickup_);

  void StateMachine(bool wait_for_button, bool bottom_intake_in, bool bottom_intake_out,
    bool bottom_intake_stop, bool top_intake_in, bool top_intake_out, bool top_intake_stop,
    bool suction_on, bool suction_off, bool hatch_out, bool hatch_in, bool arm_up, bool arm_mid,
    bool arm_high_cargo, bool arm_down, bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low,
    bool elevator_cargo_up, bool elevator_cargo_mid, bool elevator_cargo_low, bool get_cargo_ground,
    bool get_cargo_station, bool get_hatch_ground, bool get_hatch_station, bool post_intake_cargo,
    bool post_intake_hatch, bool place_hatch_high, bool place_hatch_mid, bool place_hatch_low,
    bool place_cargo_high, bool place_cargo_mid, bool place_cargo_low, bool place_cargo_bay,
    bool place_cargo_bay_fast, bool post_outtake_hatch, bool post_outtake_cargo, bool extra_button);

  void Initialize();
  void Disabled();
  void AutoBalance();

  DriveController *drive;
  Elevator *elevator;
  Intake *intake;
  Arm *arm;
  HatchPickup *hatch_pickup;

};
#endif
