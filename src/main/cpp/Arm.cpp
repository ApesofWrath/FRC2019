/*
 * Arm.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: DriversStation
 */

#include "Arm.h"

#define PI 3.14159265

const int INIT_STATE = 0;
const int UP_STATE = 1;
const int HIGH_STATE = 2;
const int MID_STATE = 3;
const int DOWN_STATE = 4;
const int STOP_ARM_STATE = 5;

const int HALL_EFF_INTAKE_ID = 0;

const double TICKS_PER_ROT_A = 4096.0;
const double MAX_VOLTAGE_A = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_A = -12.0;

const double free_speed_a = 18730.0; //rpm
const double G_a = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_A = ((free_speed_a / G_a)) * (2.0 * PI)
		/ 60.0; //rad/s
const double Kv_a = 1 / MAX_THEORETICAL_VELOCITY_A;

//for motion motionProfiler
const double MAX_VELOCITY_A= 1.0; //rad/s
const double MAX_ACCELERATION_A= 2.5; //rad/s/s
const double TIME_STEP_A= 0.01; //sec

const double UP_LIMIT_A= 1.58;
const double DOWN_LIMIT_A= 0.0;

const double UP_VOLT_LIMIT_A= 0.0;
const double DOWN_VOLT_LIMIT_A= 0.0;

const double ARM_OFFSET = 1.3;

const double STALL_VEL_A= 0.05;
const double STALL_VOLT_A= 4.0;

const double HIGH_ANGLE = 1.0;
const double DOWN_ANGLE = 0.1;
const double MID_ANGLE = 0.6;
const double UP_ANGLE = 1.3; //starting pos

double current_pos = 0.0;
double current_vel = 0.0;
double goal_pos = 0.0;
double goal_vel = 0.0;
double arm_offset = 0.0;

int last_arm_state = 0; //cannot equal the first state or profile will not set the first time

double u_a = 0; //this is the input in volts to the motor
const double v_bat_a = 12.0; //needs to be separate from the max and min voltages, which may change

std::vector<std::vector<double> > K_a;
std::vector<std::vector<double> > K_down_a = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_a = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };

std::vector<std::vector<double> > X_a = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_a = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_a;

ArmMotionProfiler *arm_profiler;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

int init_counter_a = 0;
int counter_a = 0;
int a = 0;
int encoder_counter = 0;

Arm::Arm(PowerDistributionPanel *pdp,
		ArmMotionProfiler *arm_profiler_){

  talonArm = new TalonSRX(0);

  talonArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	//talonArm->EnableCurrentLimit(true);
	talonArm->ConfigPeakCurrentLimit(20, 0); //for now
	talonArm->ConfigContinuousCurrentLimit(20, 0); //for now

  arm_profiler = arm_profiler_;

  arm_profiler->SetMaxAccArm(MAX_ACCELERATION_A);
	arm_profiler->SetMaxVelArm(MAX_VELOCITY_A);
	hallEffectArm = new DigitalInput(HALL_EFF_ARM_ID);

  pdp_i = pdp;

}

void Arm::InitializeArm() {

	//ZeroEnc();

	//if (!IsAtBottomArm()) { //don't see hall effect
	if (!is_init_arm) { //this has to be here for some reason
		SetVoltageArm(-1.5); //double arm_volt = (2.0 / pdp_i->GetVoltage()) * 1.0; //down//SetVoltageArm(-1.0);
		//talonArm->Set(ControlMode::PercentOutput, arm_volt);
	}

	//double up_volt = (0.2 * -1.0) / pdp_e->GetVoltage(); //to not crash down
	//talonElevator1->Set(ControlMode::PercentOutput, up_volt);
	//talonElevator2->Set(ControlMode::PercentOutput, up_volt);
}

void Arm::ManualArm(Joystick *joyOpArm) {

	//SmartDashboard::PutNumber("ARM CUR", talonArm->GetOutputCurrent());
	//double enc_arm = GetAngularPosition();
	//SmartDashboard::PutNumber("ARM ENC",
	//		talonArm->GetSensorCollection().GetQuadraturePosition());

	//SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	output *= MAX_VOLTAGE_I;

	//SmartDashboard::PutNumber("ARM OUTPUT", output);

	SetVoltageArm(output);

}

void Arm::Rotate(std::vector<std::vector<double> > ref_arm) {

	//top is position, bottom is velocity

	current_pos = GetAngularPosition();
	current_vel = GetAngularVelocity();
	goal_pos = ref_arm[0][0];
	goal_vel = ref_arm[1][0];

	SmartDashboard::PutNumber("ARM POS", current_pos);
	SmartDashboard::PutNumber("ARM VEL", current_vel);

	SmartDashboard::PutNumber("ARM REF POS", goal_pos);
	SmartDashboard::PutNumber("ARM REF VEL", goal_vel);

	error_a[0][0] = goal_pos - current_pos;
	error_a[1][0] = goal_vel - current_vel;

	SmartDashboard::PutNumber("ARM ERR POS", error_a[0][0]);
	SmartDashboard::PutNumber("ARM ERR VEL", error_a[1][0]);

	if (arm_profiler->final_goal_a < current_pos) { //must use final ref, to account for getting slightly ahead of the profiler
		K_a = K_down_a;
	} else {
		K_a = K_up_a;
	}

//	if (IsAtBottomArm() && std::abs(error_a[0][0]) > 0.4) { //shaking
//		arm_state = DOWN_STATE;
//	}

	arm_offset = ARM_OFFSET * (double) cos(current_pos); //counter gravity when needed because of slack on the arm

	u_a = (K_a[0][0] * error_a[0][0]) + (K_a[0][1] * error_a[1][0]) //u_a is in voltage, so * by v_bat_a
			+ (Kv_a * goal_vel * v_bat_a) + arm_offset;

	SmartDashboard::PutNumber("ARM CONT VOLT", u_a);

	SetVoltageArm(u_a);

}

void Arm::SetVoltageArm(double voltage_a) {

	is_at_bottom = IsAtBottomArm(); //hall effect returns 0 when at bottom. we reverse it here
	SmartDashboard::PutNumber("ARM HALL EFF", is_at_bottom); //actually means not at bottom //0 means up// 1 means dow

	SmartDashboard::PutString("ARM SAFETY", "none");

	//soft limit top
	if (GetAngularPosition() >= (UP_LIMIT_A) && voltage_a > UP_VOLT_LIMIT_A) { //at max height and still trying to move up
		voltage_a = 0.0; //shouldn't crash
		SmartDashboard::PutString("ARM SAFETY", "top soft limit");
	}

	//soft limit bottom
	if (is_at_bottom) {
		ZeroEnc(); //will not run after 2nd time (first time is in teleop init)
		if (GetAngularPosition() < (DOWN_LIMIT_A) && voltage_a < DOWN_VOLT_LIMIT_A) { //but hall effect goes off at 0.35
			voltage_a = 0.0;
			SmartDashboard::PutString("ARM SAFETY", "bot hall eff");
		}
	}

//	} else {
//		counter_i = 0;
//	}

	//voltage limit
	if (voltage_a > MAX_VOLTAGE_A) {
		voltage_a = MAX_VOLTAGE_A;
		SmartDashboard::PutString("ARM SAFETY", "clipped");
	} else if (voltage_a < MIN_VOLTAGE_A) {
		voltage_a = MIN_VOLTAGE_A;
		SmartDashboard::PutString("ARM SAFETY", "clipped");
	}

	//stall
	if (std::abs(GetAngularVelocity()) <= STALL_VEL_A && std::abs(voltage_a) > STALL_VOLT_A) { //outputting current, not moving, should be moving
		encoder_counter++;
	} else {
		encoder_counter = 0;
	}
	if (encoder_counter > 10) {
		voltage_safety = true;
		voltage_a = 0.0;
		SmartDashboard::PutString("ARM SAFETY", "stall");
	}

	SmartDashboard::PutNumber("ARM VOLTAGE", voltage_a);

	//scale
	voltage_a /= v_bat_a; //scale from -1 to 1 for the talon // max voltage is positive

	//reverse
	voltage_a *= -1.0; //set AT END

	talonArm->Set(ControlMode::PercentOutput, voltage_a);

}

double Arm::GetAngularVelocity() {

	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel =
			(talonArm->GetSensorCollection().GetQuadratureVelocity()
					/ (TICKS_PER_ROT_A)) * (2.0 * PI) * (10.0) * -1.0;
	//double ang_vel = 0.0;

	return ang_vel;

}

double Arm::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos =
			(talonArm->GetSensorCollection().GetQuadraturePosition()
					/ (TICKS_PER_ROT_A)) * (2.0 * PI) * -1.0;
	//double ang_pos = 0.0;

	double offset_pos = .35; //amount that the arm will stick up in radians

	return ang_pos + offset_pos;

}

bool Arm::IsAtBottomArm() {
	if (!hallEffectArm->Get()) {
		return true;
	} else {
		return false;
	}
}

void Arm::StopArm() {

	talonArm->Set(ControlMode::PercentOutput, 0.0);

}

void Arm::Down(){
  talonArm->Set(ControlMode::PercentOutput, -0.3);
}

void Arm::Up(){
  talonArm->Set(ControlMode::PercentOutput, 0.3);
}

void Arm::ArmStateMachine(){

  SmartDashboard::PutNumber("ARM ENC",
			talonArm->GetSensorCollection().GetQuadraturePosition())

  switch (arm_state){

    case INIT_STATE:
    SmartDashboard::PutString("ARM", "INIT");
    InitializeArm();
    if (is_init_arm) {
			arm_state = UP_STATE;
		}
		last_arm_state = INIT_STATE;
    break;

    case UP_STATE:
    SmartDashboard::PutString("ARM", "UP");
    if (last_arm_state != UP_STATE) { //first time in state
      arm_profiler->SetMaxAccArm(MAX_ACCELERATION_A); //these must be reset in each state
      arm_profiler->SetMaxVelArm(MAX_VELOCITY_A);
      arm_profiler->SetFinalGoalArm(UP_ANGLE);
			arm_profiler->SetInitPosArm(GetAngularPosition());
		}
		last_arm_state = UP_STATE;
    break;

    case HIGH_STATE:
    SmartDashboard::PutString("ARM", "HIGH");
    if (last_arm_state != HIGH_STATE) {
      arm_profiler->SetMaxAccArm(MAX_ACCELERATION_A); //these must be reset in each state
      arm_profiler->SetMaxVelArm(MAX_VELOCITY_A);
			arm_profiler->SetFinalGoalIntake(HIGH_ANGLE);
			arm_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_arm_state = HIGH_STATE;
    break;

    case MID_STATE:
    SmartDashboard::PutString("ARM", "MID");
    if (last_arm_state != MID_STATE) {
      arm_profiler->SetMaxAccArm(MAX_ACCELERATION_A); //these must be reset in each state
      arm_profiler->SetMaxVelArm(MAX_VELOCITY_A);
			arm_profiler->SetFinalGoalIntake(MID_ANGLE);
			arm_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_arm_state = MID_STATE;
    break;

    case DOWN_STATE:
    SmartDashboard::PutString("ARM", "DOWN");
    if (last_arm_state != DOWN_STATE) {
      arm_profiler->SetMaxAccArm(MAX_ACCELERATION_A); //these must be reset in each state
      arm_profiler->SetMaxVelArm(MAX_VELOCITY_A);
			arm_profiler->SetFinalGoalArm(DOWN_ANGLE);
			arm_profiler->SetInitPosArm(GetAngularPosition());
		}
		last_arm_state = DOWN_STATE;
    break;

    case STOP_ARM_STATE:
  		SmartDashboard::PutString("INTAKE ARM", "STOP");
  		StopArm();
  		last_intake_state = STOP_ARM_STATE;
  		break;
  }

  }

  bool Arm::EncodersRunning() { //will stop the controller from run //or stalled //MOVED INTO SET VOLTAGE

  //	double current_pos = GetAngularPosition(); //radians
  //	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

  	return true;
  }

  void Arm::ZeroEnc() { //called in Initialize() and in SetVoltage()

    if (zeroing_counter_a < 2) {
      talonArm->GetSensorCollection().SetQuadraturePosition(0, 0);
      zeroing_counter_a++;
    } else {
      is_init_arm = true;
    }
  }

  void Arm::ArmWrapper(Arm *arm) {

  	armTimer->Start();

  	while (true) {
  		while (frc::RobotState::IsEnabled()) {
  			std::this_thread::sleep_for(
  					std::chrono::milliseconds(ARM_SLEEP_TIME));

  			if (armTimer->HasPeriodPassed(ARM_WAIT_TIME)) {

  				std::vector<std::vector<double>> profile_arm =
  						arm_profiler->GetNextRefArm();

  				if (arm->arm_state != STOP_ARM_STATE
  						&& arm->arm_state != INIT_STATE) {
  					arm->Rotate(profile_arm);
  				}

  				armTimer->Reset();

  			}
  		}
  	}

  }

  void Arm::StartArmThread() {

  	Arm *arm_ = this;

  	ArmThread = std::thread(&Arm::ArmWrapper, arm_);
  	ArmThread.detach();

  }

  void Arm::EndArmThread() {

  	ArmThread.~thread();

  }

}
