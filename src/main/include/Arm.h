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
  const double TICKS_PER_ROT_A = 4096.0;
  const double MAX_VOLTAGE_A = 12.0; //CANNOT EXCEED abs(10)
  const double MIN_VOLTAGE_A = -12.0;

  const double free_speed_a = 18730.0; //rpm
  const double G_a = (831.0 / 1.0); //gear ratio
  const double MAX_THEORETICAL_VELOCITY_A = ((free_speed_a/ G_a)) * (2.0 * PI)
  		/ 60.0; //rad/s
  const double Kv_a = 1 / MAX_THEORETICAL_VELOCITY_A;

  //for motion motionProfiler
  const double MAX_VELOCITY_A = 1.0; //rad/s
  const double MAX_ACCELERATION_A = 2.5; //rad/a/a
  const double TIME_STEP_A = 0.01; //sec

  const double UP_LIMIT_A = 1.58;
  const double DOWN_LIMIT_A = 0.0;

  const double UP_VOLT_LIMIT_A = 0.0;
  const double DOWN_VOLT_LIMIT_A = 0.0;

  const double ARM_OFFSET = 1.3;

  const double STALL_VEL_A = 0.05;
  const double STALL_VOLT_A = 4.0;

  const double UP_ANGLE = 1.3; //starting pos
  const double HIGH_ANGLE = 1.0;
  const double MID_ANGLE = 0.6;
  const double LOW_ANGLE = 0.3;
  const double DOWN_ANGLE = -0.1; //lowest pos

  //id's
  const int HALL_EFF_ARM_ID = 0;
  const int ARM_TALON_ID = 0;


  Arm(frc::PowerDistributionPanel *pdp,
      ArmMotionProfiler *arm_profiler);

  TalonSRX *talonArm;

  frc::DigitalInput *hallEffectArm; //for bottom

  std::string arm_safety;

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

  std::map <int, std::string> arm_states = {
  		{INIT_STATE_H, "INIT"},
  		{UP_STATE_H, "UP"},
  		{HIGH_CARGO_STATE_H, "HIGH"},
  		{MID_CARGO_STATE_H, "MID"},
  		{LOW_CARGO_STATE_H, "LOW"},
  		{DOWN_STATE_H, "DOWN"},
      {STOP_ARM_STATE_H, "STOP"}
  	};

private:

  void InitializeArm();

  void Rotate(); //main control loop
  // Rotate Helpers
  void UpdatePositions();
  void CalcOutputVoltage();
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
	void ManualArm(frc::Joystick *joyOpArm);

  void StopArm();
  void ArmStateMachine();
  void UpdateArmProfile(int current_state, double angle);


};

#endif /* SRC_ARM_H_ */
