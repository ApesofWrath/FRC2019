#ifndef SRC_ELEVATOR_H
#define SRC_ELEVATOR_H

#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <map>
#include <vector>
#include "ElevatorMotionProfiler.h"

// TODO:
  //elevatorconstants.h
  // Some of SetVoltage()
  // states variables in .cpp and .h file?

  // DONE Move (almost all)
  // DONE Motion profiler carry over
  //DONE Stuff in State machines
    // DONE talons initialization

class Elevator {

public:

  const int TALON_ID_1 = -1;
  const int TALON_ID_2 = -1;

  int last_elevator_state = 0;
  int elevator_state = 5;

  const int ROCKET_TOP_CARGO = 0;
  const int ROCKET_MID_CARGO = 1;
  const int ROCKET_BOTTOM_CARGO = 2;
  const int ROCKET_TOP_HATCH = 3;
  const int ROCKET_MID_HATCH = 4;
  const int BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
  const int BAY_CARGO = 6;

  const double ROCKET_TOP_CARGO_POS = 0.0;
  const double ROCKET_MID_CARGO_POS = 0.0;
  const double ROCKET_BOTTOM_CARGO_POS = 0.0;
  const double ROCKET_TOP_HATCH_POS = 0.0;
  const double ROCKET_MID_HATCH_POS = 0.0;
  const double BOTTOM_HATCH_POS = 0.0;
  const double BAY_CARGO_POS = 0.0;

  TalonSRX *talonElevator1, *talonElevator2;
  ElevatorMotionProfiler *elevator_profiler;

  std::vector<std::vector<double> > K_down_e = { {  -1.0, -1.0}, { -1.0, -1.0 } };
  std::vector<std::vector<double> > K_up_e = { {  -1.0, -1.0}, { -1.0, -1.0 } };
  const double ff_percent_e = 1.0;
  const double PULLEY_DIAMETER = -1.0;

  Elevator(ElevatorMotionProfiler *elevator_profiler_);

  void Start();
  void Stop();
  std::string GetState();
  void ManualElevator();
  bool IsAtBottom();
  bool IsAtTop();

  void PrintElevatorInfo();
  double GetElevatorPosition();
  double GetElevatorVelocity();

private:

  const double TICKS_PER_ROT_E = 4096.0; //possibly not

  std::map <int, std::string> elev_state = {
    {ROCKET_TOP_CARGO, "ROCKET TOP CARGO"},
    {ROCKET_MID_CARGO, "ROCKET MID CARGO"},
    {ROCKET_BOTTOM_CARGO, "ROCKET BOTTOM CARGO"},
    {ROCKET_TOP_HATCH, "ROCKET TOP HATCH"},
    {ROCKET_MID_HATCH, "ROCKET TOP HATCH"},
    {BOTTOM_HATCH, "HATCH BOTTOM"},
    {BAY_CARGO, "BAY CARGO"}
  };

  //Move()
  double Kv_e = 0.0;
  double offset = 0.0;
  double ff = 0.0; //feedforward
  double u_e = 0.0; //this is the output in volts to the motor
  double v_bat_e = 0.0; //this will be the voltage of the battery at every loop
  double position_offset_e = 0.0;
  double current_pos_e = 0.0;
  double current_vel_e = 0.0;
  double goal_vel_e = 0.0;
  double goal_pos_e = 0.0;
  std::vector<std::vector<double>> error_e = { {  0.0, 0.0}, { 0.0, 0.0 } };
  std::vector<std::vector<double>> K_e = { {  0.0, 0.0}, { 0.0, 0.0 } }; //just stores which gains being used

  // Constructor helpers
  void SetupTalon1();
  void SetupTalon2();

  // state machine
  void ElevatorStateMachine();
  void CheckElevatorGoal(int elevator_state, double goal_pos);

  void SetVoltage(double voltage_);

  // Move functions
  void Move();
  void UpdateVoltage();
  void UpdateMoveCoordinates();
  void UpdateMoveError();
  void UpdateMovingDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_);

};

#endif
