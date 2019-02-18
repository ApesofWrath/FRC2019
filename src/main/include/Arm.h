
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
  const int REST_STATE_H = 1;
	const int HATCH_STATE_H = 2; //arm state machine
  const int CARGO_STATE_H = 3;
  // EXTRA STATES
  const int DRIVING_STATE_H = 4; // TODO: rename in .cpp file from incorrect names
  const int DOWN_STATE_H = 5;
  const int EXTRA_STATE_H = 6;
  const int STOP_ARM_STATE_H = 7;

  const double REST_ANGLE = 2.1;
  const double HATCH_ANGLE = 1.57; //90 deg
  const double CARGO_ANGLE = 0.0;
  const double MID_ANGLE = 0.6;//
  const double DOWN_ANGLE = 0.1; //
  const double EXTRA_ANGLE = 0.3;//

  const double RAD_TO_ENC = (1.0 / (2.0 * 3.14)) * 4096.0;

  const double ENC_HATCH_ANGLE = 10000.0;//HATCH_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO; //hatch cargo extra
  const double ENC_CARGO_ANGLE = 8000.0;//CARGO_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO;
  const double ENC_MID_ANGLE = 12000.0;//MID_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO;
  const double ENC_DOWN_ANGLE = 10000.0;//DOWN_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO;
  const double ENC_EXTRA_ANGLE = 9000.0;//EXTRA_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO;
  const double ENC_REST_ANGLE = 13800.0;//REST_ANGLE * RAD_TO_ENC * ENC_GEAR_RATIO;

  int arm_state = INIT_STATE_H;

  const int ARM_TALON_ID = 43;
  const int HALL_EFF_ARM_ID = 0;

  std::map <int, std::string> arm_states = {
  		{INIT_STATE_H, "INIT"},
  		{HATCH_STATE_H, "UP"},
  		{CARGO_STATE_H, "HIGH"},
  		{DRIVING_STATE_H, "MID"},
  		{DOWN_STATE_H, "LOW"},
  		{EXTRA_STATE_H, "DOWN"},
      {STOP_ARM_STATE_H, "STOP"}
  	};

  void IsElevatorHigh(bool is_high); //choose arm safety
  bool elevator_high = false;

	void ManualArm(frc::Joystick *joyOpArm);
  void ArmStateMachine();
  void Rotate(); //main control loop
  void ZeroEnc();

  double GetAngularVelocity();
  double GetAngularPosition();
  void PrintArmInfo();

  bool IsAtBottomArm();

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

  bool EncodersRunning();

	void SetVoltageArm(double voltage_a);

  // Safeties
  void UpperSoftLimit();
  void LowerSoftLimit();
  void StallSafety();

  // Output helpers
  void CapVoltage();
  void OutputArmVoltage();

  void StopArm();
  void UpdateArmProfile(int current_state, double angle);
};

#endif /* SRC_ARM_H_ */
