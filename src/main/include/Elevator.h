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
  // states variables in .cpp and .h file?

//DONE
  // Some of SetVoltage()
  // elevatorconstants.h
  // Move (almost all)
  // Motion profiler carry over
  //Stuff in State machines
  // talons initialization

class Elevator {

public:

  const int TALON_ID_1 = 33;//master
  const int TALON_ID_2 = 0;//not master

  const int TOP_HALL = 2;
  const int BOT_HALL = 1;

  const int INIT_STATE_H = 0;
  const int TOP_CARGO_STATE_H = 1;
  const int MID_CARGO_STATE_H = 2;
  const int BOTTOM_CARGO_STATE_H = 3;
  const int TOP_HATCH_STATE_H = 4;
  const int MID_HATCH_STATE_H = 5;
  const int BOTTOM_HATCH_STATE_H = 6; // Same for rocket and cargo bay, only need one
  const int BAY_CARGO_STATE_H = 7;
  const int STOP_STATE_H = 8;

  int last_elevator_state = INIT_STATE_H;
  int elevator_state = INIT_STATE_H;

  bool is_elevator_init = false;

  int zeroing_counter_e = 0;

  TalonSRX *talonElevator1, *talonElevator2;
  frc::DigitalInput *hallEffectTop, *hallEffectBottom; // http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1DigitalInput.html

  ElevatorMotionProfiler *elevator_profiler;

  std::vector<std::vector<double> > K_down_e = { { 27.89, 4.12}, { 1.0, 1.0 } };
  std::vector<std::vector<double> > K_up_e = { {  27.89, 4.12}, { 1.0, 1.0 } };

// remove halle effects if not there
// lower gains
//

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

  void Move();
  void ElevatorStateMachine();

  bool IsElevatorHigh();
  void IsArmBack(double arm_angle);

  bool keep_elevator_up = false;

private:

  std::string elev_type, elev_safety;

  std::map <int, std::string> elev_state = {
    {INIT_STATE_H, "INIT"},
    {TOP_CARGO_STATE_H, "ROCKET TOP CARGO"},
    {MID_CARGO_STATE_H, "ROCKET MID CARGO"},
    {BOTTOM_CARGO_STATE_H, "ROCKET BOTTOM CARGO"},
    {TOP_HATCH_STATE_H, "ROCKET TOP HATCH"},
    {MID_HATCH_STATE_H, "ROCKET TOP HATCH"},
    {BOTTOM_HATCH_STATE_H, "HATCH BOTTOM"},
    {BAY_CARGO_STATE_H, "BAY CARGO"},
    {STOP_STATE_H, "STOP"}
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

  void UpdateVoltage();
  void UpdateMoveCoordinates();
  void UpdateMoveError();
  void UpdateMovingDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_);

};

#endif
