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
  Elevator *elevator_climber;

  const double Kf_r_c = 1.0 / 2.0;
  const double Kp_r_c = 0.05;
  const double Ki_r_c = 0.0;
  const double Kd_r_c = 0.0;

  const double Kf_l_c = 1.0 / 2.0;
  const double Kp_l_c = 0.05;
  const double Ki_l_c = 0.0;
  const double Kd_l_c = 0.0;

  const double robot_width = 0.56; //m - dist from one leg to the other

  const int INIT_STATE_H = 0;
  const int STOP_STATE_H = 1;
  const int UP_STATE_H = 2;
  const int DOWN_STATE_H = 3;
  int climber_state = INIT_STATE_H;

  Climber(Elevator *elevator_);
  void ClimberStateMachine();
  void Up();
  void Down();
  void Stop();
  void GetPitch(double pitch);

  double GetClimberPosition();

private:

  double ahrs_pitch, target, left_error, right_error, ang_error, height_error, right_output, left_output;


};
