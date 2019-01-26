#ifndef SRC_ELEVATOR_H
#define SRC_ELEVATOR_H

#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <map>
#include <vector>
#include "ElevatorMotionProfiler.h"
#include "ElevatorConstants.h"

// TODO:
  // Some of SetVoltage()
  // states variables in .cpp and .h file?

//DONE
  // elevatorconstants.h
  // Move (almost all)
  // Motion profiler carry over
  //Stuff in State machines
  // talons initialization

class Elevator {

public:

  const int TALON_ID_1 = -1;
  const int TALON_ID_2 = -1;

  const int TOP_HALL = -1;
  const int BOT_HALL = -1;

  int last_elevator_state = 0;
  int elevator_state = 5;

  bool is_elevator_init = false;

  const int ROCKET_TOP_CARGO = 0;
  const int ROCKET_MID_CARGO = 1;
  const int ROCKET_BOTTOM_CARGO = 2;
  const int ROCKET_TOP_HATCH = 3;
  const int ROCKET_MID_HATCH = 4;
  const int BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
  const int BAY_CARGO = 6;
  const int STOP_STATE = 7;

  int zeroing_counter_e = 0;

  TalonSRX *talonElevator1, *talonElevator2;
  frc::DigitalInput *hallEffectTop, *hallEffectBottom; // http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1DigitalInput.html

  ElevatorMotionProfiler *elevator_profiler;

  std::vector<std::vector<double> > K_down_e = { {  -1.0, -1.0}, { -1.0, -1.0 } };
  std::vector<std::vector<double> > K_up_e = { {  -1.0, -1.0}, { -1.0, -1.0 } };

  Elevator(ElevatorMotionProfiler *elevator_profiler_);

  void Start();
  void Stop();
  std::string GetState();

  void ManualElevator(frc::Joystick *joyOpElev);

  void PrintElevatorInfo();
  double GetElevatorPosition();
  double GetElevatorVelocity();
  double GetVoltageElevator();

	bool IsAtBottomElevator();
	bool IsAtTopElevator();
	bool ElevatorEncodersRunning();

  bool ZeroEncs();
  void SetZeroOffset();

private:

  std::string elev_type, elev_safety;

  std::map <int, std::string> elev_state = {
    {ROCKET_TOP_CARGO, "ROCKET TOP CARGO"},
    {ROCKET_MID_CARGO, "ROCKET MID CARGO"},
    {ROCKET_BOTTOM_CARGO, "ROCKET BOTTOM CARGO"},
    {ROCKET_TOP_HATCH, "ROCKET TOP HATCH"},
    {ROCKET_MID_HATCH, "ROCKET TOP HATCH"},
    {BOTTOM_HATCH, "HATCH BOTTOM"},
    {BAY_CARGO, "BAY CARGO"},
    {STOP_STATE, "STOP"}
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
  // Set voltage safeties
  void StallSafety();
  void UpperSoftLimit();
  void LowerSoftLimit();
  void TopHallEffectSafety();
  void BottomHallEffectSafety();
  void ArmSafety();
  // setvoltage output helpers
  void ZeroElevator();
  void CapVoltage();
  void ScaleOutput();
  void InvertOutput();
	void OutputToTalon();

  // Move functions
  void Move();
  void UpdateVoltage();
  void UpdateMoveCoordinates();
  void UpdateMoveError();
  void UpdateMovingDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_);

};

#endif
