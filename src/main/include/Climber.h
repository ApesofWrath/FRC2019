#include "Arm.h"
#include "Elevator.h"
#include "MotionProfiling/ElevatorMotionProfiler.h"

#include "AHRS.h"
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"

#include <frc/SmartDashboard/SmartDashboard.h>

class Climber {

public:

  TalonSRX *talonClimb1, *talonClimb2;
  AHRS *ahrs_climber;

  const double ENC_CRUISE_VEL_C = 1000;
  const double ENC_CRUISE_ACC_C = 500;

  const double Kf_c = 0.0;
  const double Kp_c = 0.0;
  const double Ki_c = 0.0;
  const double Kd_c = 0.0;

  const int INIT_STATE_H = 0;
  const int STOP_STATE_H = 1;
  const int UP_STATE_H = 2;
  const int DOWN_STATE_H = 3;
  int climber_state;

  Climber(AHRS *ahrs_);
  void ClimberStateMachine();
  void Up();
  void Down();
  void Stop();

};
