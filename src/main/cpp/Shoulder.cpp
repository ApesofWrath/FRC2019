/*
 * Shoulder.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Kaya
 */

#include "Shoulder.h"

#define PI 3.14159265

const int INIT_STATE = 0;
const int UP_STATE = 1;
const int HIGH_STATE = 2;
const int MID_STATE = 3;
const int DOWN_STATE = 4;
const int STOP_SHOULDER_STATE = 5;

const int HALL_EFF_SHOULDER_ID = 0;

const double TICKS_PER_ROT_S = 4096.0;
const double MAX_VOLTAGE_S = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_S = -12.0;

const double free_speed_s = 18730.0; //rpm
const double G_s = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_S = ((free_speed_s/ G_s)) * (2.0 * PI)
		/ 60.0; //rad/s
const double Kv_s = 1 / MAX_THEORETICAL_VELOCITY_S;

//for motion motionProfiler
const double MAX_VELOCITY_S = 1.0; //rad/s
const double MAX_ACCELERATION_S = 2.5; //rad/s/s
const double TIME_STEP_S = 0.01; //sec

const double UP_LIMIT_S = 1.58;
const double DOWN_LIMIT_S = 0.0;

const double UP_VOLT_LIMIT_S = 0.0;
const double DOWN_VOLT_LIMIT_S = 0.0;

const double SHOULDER_OFFSET = 1.3;

const double STALL_VEL_S = 0.05;
const double STALL_VOLT_S = 4.0;

const double HIGH_ANGLE = 1.0;
const double DOWN_ANGLE = 0.1;
const double MID_ANGLE = 0.6;
const double UP_ANGLE = 1.3; //starting pos

double current_pos = 0.0;
double current_vel = 0.0;
double goal_pos = 0.0;
double goal_vel = 0.0;
double shoulder_offset = 0.0;

int last_shoulder_state = 0; //cannot equal the first state or profile will not set the first time

double u_s= 0; //this is the input in volts to the motor
const double v_bat_s = 12.0; //needs to be separate from the max and min voltages, which may change

std::vector<std::vector<double> > K_s;
std::vector<std::vector<double> > K_down_s = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_s = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };

std::vector<std::vector<double> > X_s = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_s = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_s;

ShoulderMotionProfiler *shoulder_profiler;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

int init_counter_s = 0;
int counter_s = 0;
int s = 0;
int encoder_counter = 0;

Shoulder::Shoulder(PowerDistributionPanel *pdp,
		ShoulderMotionProfiler *shoulder_profiler_){

  talonShoulder = new TalonSRX(0);

  talonShoulder->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	//talonShoulder->EnableCurrentLimit(true);
	talonShoulder->ConfigPeakCurrentLimit(20, 0); //for now
	talonShoulder->ConfigContinuousCurrentLimit(20, 0); //for now

  shoulder_profiler = shoulder_profiler_;

  shoulder_profiler->SetMaxAccShoulder(MAX_ACCELERATION_S);
	shoulder_profiler->SetMaxVelShoulder(MAX_VELOCITY_S);
	hallEffectShoulder = new DigitalInput(HALL_EFF_SHOULDER_ID);

  pdp_s= pdp;

}

void Shoulder::InitializeShoulder() {

	//ZeroEnc();

	//if (!IsAtBottomShoulder()) { //don't see hall effect
	if (!is_init_shoulder) { //this has to be here for some reason
		SetVoltageShoulder(-1.5); //double shoulder_volt = (2.0 / pdp_i->GetVoltage()) * 1.0; //down//SetVoltageShoulder(-1.0);
		//talonShoulder->Set(ControlMode::PercentOutput, shoulder_volt);
	}

	//double up_volt = (0.2 * -1.0) / pdp_e->GetVoltage(); //to not crash down
	//talonElevator1->Set(ControlMode::PercentOutput, up_volt);
	//talonElevator2->Set(ControlMode::PercentOutput, up_volt);
}

void Shoulder::ManualShoulder(Joystick *joyOpShoulder) {

	//SmartDashboard::PutNumber("SHOULDER CUR", talonShoulder->GetOutputCurrent());
	//double enc_shoulder = GetAngularPosition();
	//SmartDashboard::PutNumber("SHOULDER ENC",
	//		talonShoulder->GetSensorCollection().GetQuadraturePosition());

	//SmartDashboard::PutNumber("SHOULDER POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpShoulder->GetY() * 0.5 * 1.0;

	output *= MAX_VOLTAGE_S;

	//SmartDashboard::PutNumber("SHOULDER OUTPUT", output);

	SetVoltageShoulder(output);

}

void Shoulder::Rotate(std::vector<std::vector<double> > ref_shoulder) {

	//top is position, bottom is velocity

	current_pos = GetAngularPosition();
	current_vel = GetAngularVelocity();
	goal_pos = ref_shoulder[0][0];
	goal_vel = ref_shoulder[1][0];

	SmartDashboard::PutNumber("SHOULDER POS", current_pos);
	SmartDashboard::PutNumber("SHOULDER VEL", current_vel);

	SmartDashboard::PutNumber("SHOULDER REF POS", goal_pos);
	SmartDashboard::PutNumber("SHOULDER REF VEL", goal_vel);

	error_s[0][0] = goal_pos - current_pos;
	error_s[1][0] = goal_vel - current_vel;

	SmartDashboard::PutNumber("SHOULDER ERR POS", error_s[0][0]);
	SmartDashboard::PutNumber("SHOULDER ERR VEL", error_s[1][0]);

	if (shoulder_profiler->final_goal_s< current_pos) { //must use final ref, to account for getting slightly ahead of the profiler
		K_s = K_down_s;
	} else {
		K_s = K_up_s;
	}

//	if (IsAtBottomShoulder() && std::abs(error_a[0][0]) > 0.4) { //shaking
//		shoulder_state = DOWN_STATE;
//	}

	shoulder_offset = SHOULDER_OFFSET * (double) cos(current_pos); //counter gravity when needed because of slack on the shoulder

	u_s= (K_s[0][0] * error_s[0][0]) + (K_s[0][1] * error_s[1][0]) //u_sis in voltage, so * by v_bat_a
			+ (Kv_s* goal_vel * v_bat_s) + shoulder_offset;

	SmartDashboard::PutNumber("SHOULDER CONT VOLT", u_s);

	SetVoltageShoulder(u_s);

}

void Shoulder::SetVoltageShoulder(double voltage_s) {

	is_at_bottom = IsAtBottomShoulder(); //hall effect returns 0 when at bottom. we reverse it here
	SmartDashboard::PutNumber("SHOULDER HALL EFF", is_at_bottom); //actually means not at bottom //0 means up// 1 means dow

	SmartDashboard::PutString("SHOULDER SAFETY", "none");

	//soft limit top
	if (GetAngularPosition() >= (UP_LIMIT_S) && voltage_s> UP_VOLT_LIMIT_S) { //at max height and still trying to move up
		voltage_s= 0.0; //shouldn't crash
		SmartDashboard::PutString("SHOULDER SAFETY", "top soft limit");
	}

	//soft limit bottom
	if (is_at_bottom) {
		ZeroEnc(); //will not run after 2nd time (first time is in teleop init)
		if (GetAngularPosition() < (DOWN_LIMIT_S) && voltage_s< DOWN_VOLT_LIMIT_S) { //but hall effect goes off at 0.35
			voltage_s = 0.0;
			SmartDashboard::PutString("SHOULDER SAFETY", "bot hall eff");
		}
	}

//	} else {
//		counter_s= 0;
//	}

	//voltage limit
	if (voltage_s > MAX_VOLTAGE_S) {
		voltage_s = MAX_VOLTAGE_S;
		SmartDashboard::PutString("SHOULDER SAFETY", "clipped");
	} else if (voltage_s < MIN_VOLTAGE_S) {
		voltage_s = MIN_VOLTAGE_S;
		SmartDashboard::PutString("SHOULDER SAFETY", "clipped");
	}

	//stall
	if (std::abs(GetAngularVelocity()) <= STALL_VEL_S && std::abs(voltage_s) > STALL_VOLT_S) { //outputting current, not moving, should be moving
		encoder_counter++;
	} else {
		encoder_counter = 0;
	}
	if (encoder_counter > 10) {
		voltage_safety = true;
		voltage_s = 0.0;
		SmartDashboard::PutString("SHOULDER SAFETY", "stall");
	}

	SmartDashboard::PutNumber("SHOULDER VOLTAGE", voltage_s);

	//scale
	voltage_s /= v_bat_s; //scale from -1 to 1 for the talon // max voltage is positive

	//reverse
	voltage_s *= -1.0; //set AT END

	talonShoulder->Set(ControlMode::PercentOutput, voltage_s);

}

double Shoulder::GetAngularVelocity() {

	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel =
			(talonShoulder->GetSensorCollection().GetQuadratureVelocity()
					/ (TICKS_PER_ROT_S)) * (2.0 * PI) * (10.0) * -1.0;
	//double ang_vel = 0.0;

	return ang_vel;

}

double Shoulder::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos =
			(talonShoulder->GetSensorCollection().GetQuadraturePosition()
					/ (TICKS_PER_ROT_S)) * (2.0 * PI) * -1.0;
	//double ang_pos = 0.0;

	double offset_pos = .35; //amount that the shoulder will stick up in radians

	return ang_pos + offset_pos;

}

bool Shoulder::IsAtBottomShoulder() {
	if (!hallEffectShoulder->Get()) {
		return true;
	} else {
		return false;
	}
}

void Shoulder::StopShoulder() {

	talonShoulder->Set(ControlMode::PercentOutput, 0.0);

}

void Shoulder::ShoulderStateMachine(){

  SmartDashboard::PutNumber("SHOULDER ENC",
			talonShoulder->GetSensorCollection().GetQuadraturePosition())

  switch (shoulder_state){

    case INIT_STATE:
    SmartDashboard::PutString("SHOULDER", "INIT");
    InitializeShoulder();
    if (is_init_shoulder) {
			shoulder_state = UP_STATE;
		}
		last_shoulder_state = INIT_STATE;
    break;

    case UP_STATE:
    SmartDashboard::PutString("SHOULDER", "UP");
    if (last_shoulder_state != UP_STATE) { //first time in state
      shoulder_profiler->SetMaxAccShoulder(MAX_ACCELERATION_S); //these must be reset in each state
      shoulder_profiler->SetMaxVelShoulder(MAX_VELOCITY_S);
      shoulder_profiler->SetFinalGoalShoulder(UP_ANGLE);
			shoulder_profiler->SetInitPosShoulder(GetAngularPosition());
		}
		last_shoulder_state = UP_STATE;
    break;

    case HIGH_STATE:
    SmartDashboard::PutString("SHOULDER", "HIGH");
    if (last_shoulder_state != HIGH_STATE) {
      shoulder_profiler->SetMaxAccShoulder(MAX_ACCELERATION_S); //these must be reset in each state
      shoulder_profiler->SetMaxVelShoulder(MAX_VELOCITY_S);
			shoulder_profiler->SetFinalGoalIntake(HIGH_ANGLE);
			shoulder_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_shoulder_state = HIGH_STATE;
    break;

    case MID_STATE:
    SmartDashboard::PutString("SHOULDER", "MID");
    if (last_shoulder_state != MID_STATE) {
      shoulder_profiler->SetMaxAccShoulder(MAX_ACCELERATION_S); //these must be reset in each state
      shoulder_profiler->SetMaxVelShoulder(MAX_VELOCITY_S);
			shoulder_profiler->SetFinalGoalIntake(MID_ANGLE);
			shoulder_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_shoulder_state = MID_STATE;
    break;

    case DOWN_STATE:
    SmartDashboard::PutString("SHOULDER", "DOWN");
    if (last_shoulder_state != DOWN_STATE) {
      shoulder_profiler->SetMaxAccShoulder(MAX_ACCELERATION_S); //these must be reset in each state
      shoulder_profiler->SetMaxVelShoulder(MAX_VELOCITY_S);
			shoulder_profiler->SetFinalGoalShoulder(DOWN_ANGLE);
			shoulder_profiler->SetInitPosShoulder(GetAngularPosition());
		}
		last_shoulder_state = DOWN_STATE;
    break;

    case STOP_SHOULDER_STATE:
  		SmartDashboard::PutString("SHOULDER", "STOP");
  		StopShoulder();
  		last_shoulder_state = STOP_SHOULDER_STATE;
  		break;
  }

  }

  bool Shoulder::EncodersRunning() { //will stop the controller from run //or stalled //MOVED INTO SET VOLTAGE

  //	double current_pos = GetAngularPosition(); //radians
  //	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

  	return true;
  }

  void Shoulder::ZeroEnc() { //called in Initialize() and in SetVoltage()

    if (zeroing_counter_s< 2) {
      talonShoulder->GetSensorCollection().SetQuadraturePosition(0, 0);
      zeroing_counter_s++;
    } else {
      is_init_shoulder = true;
    }
  }

  void Shoulder::ShoulderWrapper(Shoulder *shoulder) {

  	shoulderTimer->Start();

  	while (true) {
  		while (frc::RobotState::IsEnabled()) {
  			std::this_thread::sleep_for(
  					std::chrono::milliseconds(SHOULDER_SLEEP_TIME));

  			if (shoulderTimer->HasPeriodPassed(SHOULDER_WAIT_TIME)) {

  				std::vector<std::vector<double>> profile_shoulder =
  						shoulder_profiler->GetNextRefShoulder();

  				if (shoulder->shoulder_state != STOP_SHOULDER_STATE
  						&& shoulder->shoulder_state != INIT_STATE) {
  					shoulder->Rotate(profile_shoulder);
  				}

  				shoulderTimer->Reset();

  			}
  		}
  	}

  }

  void Shoulder::StartShoulderThread() {

  	Shoulder *shoulder_ = this;

  	ShoulderThread = std::thread(&Shoulder::ShoulderWrapper, shoulder_);
  	ShoulderThread.detach();

  }

  void Shoulder::EndShoulderThread() {

  	ShoulderThread.~thread();

  }

}
