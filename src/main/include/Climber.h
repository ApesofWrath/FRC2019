#include "Arm.h"
#include "Elevator.h"
#include "MotionProfiling/ElevatorMotionProfiler.h"

#include "AHRS.h"
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"

#include <frc/SmartDashboard/SmartDashboard.h>

class Climber{
public:
  TalonSRX* talonClimb;
  ElevatorMotionProfiler* elevator_motion_profiler;
  Elevator* elevator;
  AHRS* ahrs;

  const int INIT_STATE_H = 0;
  const int STOP_STATE_H = 1;
  const int UP_STATE_H = 2;
  const int DOWN_STATE_H = 3;


  int climber_state;

  Climber();
  void ClimberStateMachine();
  void Up();
  void Down();
  void Stop();

};
