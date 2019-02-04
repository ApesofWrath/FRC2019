//frc and 3rd party library includes
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/WPILib.h>

// Includes of our code
#include "TeleopStateMachine.h"
#include "Arm.h"
#include "Elevator.h"
#include "Intake.h"
#include "HatchPickup.h"

class TeleopStateMachine {
public:

  TeleopStateMachine(Elevator *elevator_, Intake *intake_,
      Arm *arm_, Suction *suction_);
  void StateMachine(bool wait_for_button, bool bottom_intake_in,
      bool bottom_intake_out, bool bottom_intake_stop, bool top_intake_in, bool top_intake_out,
      bool top_intake_stop, bool suction_on, bool suction_off, bool hatch_out, bool hatch_in, bool arm_up,
      bool arm_down, bool elevator_hatch_up, bool elevator_hatch_mid, bool elevator_hatch_low,
      bool elevator_ball_up, bool elevator_ball_mid, bool elevator_ball_low, bool elevator_down,
      bool get_ball, bool get_hatch_ground, bool get_hatch_station, bool post_intake_ball,
      bool post_intake_hatch, bool place_hatch, bool place_ball, bool post_outtake_hatch, bool post_outtake_ball);

  Elevator *elevator;
  Intake *intake;
  Arm *arm;
  Suction *suction;
  Solenoids *solenoids;

}
