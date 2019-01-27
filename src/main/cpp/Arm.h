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
#include <frc/DigitalInput.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ArmMotionProfiler.h"
#include "ArmConstants.h"

class Arm {
public:

  TalonSRX *talonArm;

  frc::DigitalInput *hallEffectArm; //for bottom

  std::thread ArmThread;
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

  Arm(frc::PowerDistributionPanel *pdp,
      ArmMotionProfiler *arm_profiler);

  void InitializeArm();

  void Rotate(std::vector<std::vector<double> > ref_arm);
	double GetAngularVelocity();
	double GetAngularPosition();

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

  void StartArmThread();
	void EndArmThread();
	static void ArmWrapper(Arm *arm);

};

#endif /* SRC_ARM_H_ */
