#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <map>
#include <vector>
#include "../cpp/ElevatorMotionProfiler.cpp"

// TODO:

  // Some of SetVoltage()
  // states variables in .cpp and .h file?

  // DONE Move (almost all)
  // DONE Motion profiler carry over
  //DONE Stuff in State machines
    // DONE talons initialization

class Elevator {

public:
  TalonSRX *talonElevator1, *talonElevator2;
  ElevatorMotionProfiler *elevator_profiler;

  Elevator(ElevatorMotionProfiler *elevator_profiler_, double rocket_top_cargo_pos_, double rocket_mid_cargo_pos_, double rocket_bottom_cargo_pos_, double rocket_top_hatch_pos_, double rocket_mid_hatch_pos_, double bottom_hatch_pos_, double bay_cargo_pos_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_, double ff_percent_e_, double PULLEY_DIAMETER_);

  void Start();
  void Stop();
  std::string GetState();
  void ManualElevator();

private:
  double rocket_top_cargo_pos, rocket_mid_cargo_pos, rocket_bottom_cargo_pos, rocket_top_hatch_pos, rocket_mid_hatch_pos, bottom_hatch_pos, bay_cargo_pos;

  double PULLEY_DIAMETER;
  const double TICKS_PER_ROT_E = 4096.0; //possibly not

  int last_elevator_state;
  int current_state;
  const int ROCKET_TOP_CARGO_H = 0;
  const int ROCKET_MID_CARGO_H = 1;
  const int ROCKET_BOTTOM_CARGO_H = 2;
  const int ROCKET_TOP_HATCH_H = 3;
  const int ROCKET_MID_HATCH_H = 4;
  const int BOTTOM_HATCH_H = 5; // Same for rocket and cargo bay, only need one
  const int BAY_CARGO_H = 6;
  // DIFFERNET STATES FOR LOADING STATION?? (HPS_STATE)

  std::map <int, std::string> elev_state = {
    {ROCKET_TOP_CARGO_H, "ROCKET TOP CARGO"},
    {ROCKET_MID_CARGO_H, "ROCKET MID CARGO"},
    {ROCKET_BOTTOM_CARGO_H, "ROCKET BOTTOM CARGO"},
    {ROCKET_TOP_HATCH_H, "ROCKET TOP HATCH"},
    {ROCKET_MID_HATCH_H, "ROCKET TOP HATCH"},
    {BOTTOM_HATCH_H, "HATCH BOTTOM"},
    {BAY_CARGO_H, "BAY CARGO"}
  };

  double goal_vel_e = 0.0;
  double goal_pos_e = 0.0;
  double ff_percent_e;

  std::vector<std::vector<double>> K_down_e, K_up_e, K_e;

  // Constructor helpers
  void SetupTalon1();
  void SetupTalon2();

  // state machine
  void ElevatorStateMachine();
  void CheckElevatorGoal(int current_state, double goal_pos);

  void SetVoltage(double votlage_);

  // Move functions
  void Move();
  void UpdateVoltage();
  void UpdateMoveCoordinates();
  void UpdateMoveError();
  void UpdateMovingDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_);

  // helpers
  void PrintElevatorInfo();
  double GetElevatorPosition();
  double GetElevatorVelocity();
};
