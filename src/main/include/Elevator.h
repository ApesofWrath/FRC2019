// TODO: check states and positins for elevator

#ifndef SRC_ELEVATOR_H
#define SRC_ELEVATOR_H

#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <map>
#include <vector>
#include "MotionProfiling/ElevatorMotionProfiler.h"
#include "ElevatorConstants.h"

class Elevator {

public:

  const double METERS_TO_ENCS = (1.0 / (3.14 * PULLEY_DIAMETER)) * 4096.0 / 2.0; //enc value of x results in enc height of 2x

  const int TALON_ID_1 = 14;//master
  const int TALON_ID_2 = 3;//not master

  const int TOP_HALL = 2;
  const int BOT_HALL = 1;

  const int INIT_STATE_H = 0;
  const int TOP_CARGO_STATE_H = 1;
  const int MID_CARGO_STATE_H = 2;
  const int BOTTOM_CARGO_STATE_H = 3;
  const int TOP_HATCH_STATE_H = 4;
  const int MID_HATCH_STATE_H = 5;
  const int BOTTOM_HATCH_STATE_H = 6; // Same for rocket and cargo bay, only need one
  const int HOLD_HATCH_STATE_H = 7;
  const int STOP_STATE_H = 8;
  const int LIFTING_ARM_STATE_H = 9;

  // elevator posititons for the differnet states
  const double TOP_HATCH_POS = 1.37; //top scoring
  const double MID_HATCH_POS = 0.72; //mid scoring
  const double BOTTOM_HATCH_POS = 0.05; //bottom scoring, get hatch station state, post intake hatch, post intake cargo, post outtake hatch, post outtake cargo

  const double TOP_CARGO_POS = 1.42; //top scoring
  const double MID_CARGO_POS = 0.95; //mid rocket, bay
  const double BOTTOM_CARGO_POS = 0.295; //bottom scoring

  const double LIFTING_ARM_POS = 0.45; //get hatch ground, get cargo
  const double HOLD_HATCH_POS = 0.26; //get hatch ground, get cargo
  //

  const double ENC_TOP_CARGO_POS = TOP_CARGO_POS * METERS_TO_ENCS;
  const double ENC_MID_CARGO_POS = MID_CARGO_POS * METERS_TO_ENCS;
  const double ENC_BOTTOM_CARGO_POS = BOTTOM_CARGO_POS * METERS_TO_ENCS;
  const double ENC_TOP_HATCH_POS = TOP_HATCH_POS * METERS_TO_ENCS;
  const double ENC_MID_HATCH_POS = MID_HATCH_POS * METERS_TO_ENCS;
  const double ENC_BOTTOM_HATCH_POS = BOTTOM_HATCH_POS * METERS_TO_ENCS;
  const double ENC_HOLD_HATCH_POS = HOLD_HATCH_POS * METERS_TO_ENCS;
  const double ENC_LIFTING_ARM_POS = LIFTING_ARM_POS * METERS_TO_ENCS;

  int last_elevator_state = INIT_STATE_H;
  int elevator_state = INIT_STATE_H;

  bool is_elevator_init = false;

  int zeroing_counter_e = 0;

  TalonSRX *talonElevator1;
  VictorSPX *talonElevator2;
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
  void SetZeroOffset(); //not needed

  void Move();
  void ElevatorStateMachine();

  void Climb();

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
    {HOLD_HATCH_STATE_H, "BAY CARGO"},
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

  double target, last_target, output, error, current, ang_error, height_error, ahrs_pitch;
  double robot_length = 1.5; //m
  double Kp_c = 0.08;

  // Constructor helpers
  void SetupTalon1();
  void SetupTalon2();

  // state machine
  void CheckElevatorGoal(int elevator_state, double goal_pos);

  void SetVoltage(double voltage_);
  // Set voltage safeties
  bool StallSafety();
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
