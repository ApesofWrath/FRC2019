#include <WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>


class Elevator {

public:
  TalonSRX *talonElevator1, *talonElevator2;
  ElevatorMotionProfiler *elevator_profiler;

  Elevator();
  void Start();
  void Stop();
  std::string GetState();
  void ManualElevator();

private:
  void ElevatorStateMachine(ElevatorMotionProfiler *elevator_profiler_);
  void SetVotltage(double votlage_);
  void Move();
  void PrintElevatorInfo();
};
