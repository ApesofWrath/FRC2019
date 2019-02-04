/*
 * Arm.h
 *
 *  Created on: Jan 7, 2019
 *      Author: Kaya
 */

 #ifndef SRC_ARM_H_
 #define SRC_ARM_H_

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <frc/Timer.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <list>
#include <map>
#include <frc/DigitalInput.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ArmMotionProfiler.h"
#include "ArmConstants.h"

class Arm {
public:

  TalonSRX *talonArm;
  ArmMotionProfiler *arm_profiler;

  frc::DigitalInput *hallEffectArm; //for bottom

  Arm(ArmMotionProfiler *arm_profiler);

  std::string arm_safety;

  std::vector<std::vector<double> > K_down_a = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
  		{ 10.32, 0.063 } };
  // std::vector<std::vector<double> > K_down_a = { { 2.0, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
  		// { 2.0, 0.063 } }; // REMOVE AND UNCOMMENT AFTER TESTING
  std::vector<std::vector<double> > K_up_a = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
  		{ 10.32, 0.063 } };
  // std::vector<std::vector<double> > K_up_a = { { 2.0, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
  // 		{ 2.0, 0.063 } }; // REMOVE AND UNCOMMENT AFTER TESTING

  std::vector<std::vector<double> > X_a = { { 0.0 }, //state matrix filled with the states of the system //not used
  		{ 0.0 } };

  const double ff_percent_a = 0.6;

	int zeroing_counter_a = 0;

	bool is_init_arm = false; //is arm initialized

  const int INIT_STATE_H = 0;
	const int UP_STATE_H = 1; //arm state machine
  const int HIGH_CARGO_STATE_H = 2;
  const int MID_CARGO_STATE_H = 3;
  const int LOW_CARGO_STATE_H = 4;
  const int DOWN_STATE_H = 5;
  const int STOP_ARM_STATE_H = 6;
  int arm_state = INIT_STATE_H;

  const int ARM_TALON_ID = 55;
  const int HALL_EFF_ARM_ID = 0;

  std::map <int, std::string> arm_states = {
  		{INIT_STATE_H, "INIT"},
  		{UP_STATE_H, "UP"},
  		{HIGH_CARGO_STATE_H, "HIGH"},
  		{MID_CARGO_STATE_H, "MID"},
  		{LOW_CARGO_STATE_H, "LOW"},
  		{DOWN_STATE_H, "DOWN"},
      {STOP_ARM_STATE_H, "STOP"}
  	};

  void IsElevatorHigh(bool is_high); //choose arm safety
  bool elevator_high = false;

	void ManualArm(frc::Joystick *joyOpArm);
  void ArmStateMachine();
  void Rotate(); //main control loop
private:

  double arm_pos;
  double arm_vel;

  std::vector<std::vector<double> > error_a = { { 0.0 }, { 0.0 } };
  std::vector<std::vector<double> > K_a;


  double current_pos = 0.0;
  double current_vel = 0.0;
  double goal_pos = 0.0;
  double goal_vel = 0.0;
  double arm_offset = 0.0;

  int last_arm_state = 0; //cannot equal the first state or profile will not set the first time

  double u_a = 0; //this is the input in volts to the motor
  const double v_bat_a = 12.0; //needs to be separate from the max and min voltages, which may change

  bool first_time_at_bottom = false;
  bool voltage_safety = false;

  int init_counter_a = 0;
  int counter_a = 0;
  int a = 0;
  int encoder_counter = 0;
  double arm_voltage = 0.0;

  void InitializeArm();

  // Rotate Helpers
  void UpdateRotateCoordinates();
  void UpdateRotateError();
  void UpdateRotatingDirection(std::vector<std::vector<double>> K_a_);
  void UpdateVoltage();
	double GetAngularVelocity();
	double GetAngularPosition();
  void PrintArmInfo();

	bool IsAtBottomArm();
  bool EncodersRunning();

	void SetVoltageArm(double voltage_a);
  // Safeties
  void UpperSoftLimit();
  void LowerSoftLimit();
  void StallSafety();
  // Output helpers
  void CapVoltage();
  void OutputArmVoltage();

	void ZeroEnc();

  void StopArm();
  void UpdateArmProfile(int current_state, double angle);
};

#endif /* SRC_ARM_H_ */
