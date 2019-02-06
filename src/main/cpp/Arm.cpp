#include "Arm.h"

#define PI 3.14159265

int counter = 0;

const int INIT_STATE = 0;
const int UP_STATE = 1;
const int HIGH_CARGO_STATE = 2;
const int MID_CARGO_STATE = 3;
const int LOW_CARGO_STATE = 4;
const int DOWN_STATE = 5;
const int STOP_ARM_STATE = 6;

Arm::Arm(ArmMotionProfiler *arm_profiler_) {

  talonArm = new TalonSRX(ARM_TALON_ID);

  talonArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	//talonArm->EnableCurrentLimit(true);
	talonArm->ConfigPeakCurrentLimit(20, 0); //for now
	talonArm->ConfigContinuousCurrentLimit(20, 0); //for now

  arm_profiler = arm_profiler_;

	hallEffectArm = new frc::DigitalInput(HALL_EFF_ARM_ID);

}

void Arm::InitializeArm() {
	if (!is_init_arm) { //this has to be here for some reason
		SetVoltageArm(3.5); //	MAYBE SHOULDNT BE NEGATIVE - CHECK
	}

	ZeroEnc(); //won't start at 0 on this year's robot!
}

void Arm::ManualArm(frc::Joystick *joyOpArm) {

	frc::SmartDashboard::PutNumber("ARM CUR", talonArm->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("ARM ENC", talonArm->GetSensorCollection().GetQuadraturePosition());

	frc::SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0 * MAX_VOLTAGE_A;

	frc::SmartDashboard::PutNumber("ARM OUTPUT", output);
	// TODO, IMPORTANT: find out who wrote this
		// frc::SmartDashboard::PutString("HasRobotVoted", "True")

	SetVoltageArm(output);

}

void Arm::PrintArmInfo() {
  frc::SmartDashboard::PutBoolean("ARM HALL EFFECT", hallEffectArm->Get());
	frc::SmartDashboard::PutNumber("ARM POS", GetAngularPosition());
	frc::SmartDashboard::PutNumber("ARM VEL", GetAngularVelocity());
  frc::SmartDashboard::PutNumber("counter", counter);
	// frc::SmartDashboard::PutNumber("ARM REF POS", ref_arm[0][0]);
	// frc::SmartDashboard::PutNumber("ARM REF VEL", ref_arm[1][0]);

	frc::SmartDashboard::PutNumber("ARM ERR POS", error_a[0][0]);
	frc::SmartDashboard::PutNumber("ARM ERR VEL", error_a[1][0]);

	frc::SmartDashboard::PutNumber("ARM CONT VOLT", u_a);
}

// void Arm::RunController() {
//
//
// 	// Shaking: if (IsAtBottomArm() && std::abs(error_a[0][0]) > 0.4) {arm_state = DOWN_STATE;}
//
//
//   frc::SmartDashboard::PutNumber("GOAL VELOCITY", goal_vel);
//   frc::SmartDashboard::PutNumber("ERROR POS", error_a[0][0]);
//   frc::SmartDashboard::PutNumber("ERROR VELOCITY", error_a[0][1]);
//   frc::SmartDashboard::PutNumber("OFFSET", arm_offset);
//
// }

void Arm::UpdateRotateCoordinates() {
  //top is position, bottom is velocity
	std::vector<std::vector<double> > ref_arm = arm_profiler->GetNextRefArm();

	current_pos = GetAngularPosition();
  current_vel = GetAngularVelocity();

  goal_vel = ref_arm[1][0];
	goal_pos = ref_arm[0][0];

  frc::SmartDashboard::PutNumber("REF ARM POS", ref_arm[0][0]);
  frc::SmartDashboard::PutNumber("REF ARM VEL", ref_arm[1][0]);
}

void Arm::UpdateRotateError() {
  error_a[0][0] = goal_pos - current_pos;
	error_a[1][0] = goal_vel - current_vel;
}

void Arm::UpdateRotatingDirection(std::vector<std::vector<double>> K_a_) {
  arm_offset = ARM_OFFSET * (double) cos(current_pos);
  K_a = K_a_;
  // TODO: does ff change? if so ff = ...
}

void Arm::UpdateVoltage() {
  u_a = (K_a[0][0] * error_a[0][0]) + (K_a[0][1] * error_a[1][0]) //u_sis in voltage, so * by v_bat_a
			+ (Kv_a* goal_vel * v_bat_a) * ff_percent_a + arm_offset;
}

void Arm::Rotate() {
	// Setup
	if (arm_state != STOP_ARM_STATE_H
		&& arm_state != INIT_STATE_H) {
			UpdateRotateCoordinates();
      UpdateRotateError();

      if (arm_profiler->GetFinalGoalArm() < arm_profiler->GetInitPosArm()) { //must use final ref, to account for getting slightly ahead of the profiler
    		UpdateRotatingDirection(K_down_a);
    	} else {
    		UpdateRotatingDirection(K_up_a);
    	}

      UpdateVoltage();

    // if (arm_profiler->SetFinalGoalArm() < arm_profiler->SetInitPosArm()) {
      SetVoltageArm(u_a);
    // }

			PrintArmInfo();
		}
}

void Arm::IsElevatorHigh(bool is_high) {
	elevator_high = is_high;
}

void Arm::SetVoltageArm(double voltage_a) {
  arm_voltage = voltage_a;
  is_init_arm = true;

  arm_state = HIGH_CARGO_STATE;
	// frc::SmartDashboard::PutNumber("ARM HALL EFF", IsAtBottomArm()); // actually means not at bottom //0 means up// 1 means down

	arm_safety = "NONE";

	arm_pos = GetAngularPosition();
	arm_vel = GetAngularVelocity();

	UpperSoftLimit();
	LowerSoftLimit();
  StallSafety();
	CapVoltage();

	//zero height moves up sometimes. this will make sure the arm goes all the way down every time
	if ((arm_state == DOWN_STATE) && arm_pos <= 0.4) { // TODO: change back to 0.2 after fix zeroing encoders
		arm_voltage = 0.0;

	}
	frc::SmartDashboard::PutString("ARM SAFETY", arm_safety);
	frc::SmartDashboard::PutNumber("ARM VOLTAGE", arm_voltage);

	OutputArmVoltage();
}

void Arm::OutputArmVoltage() {
	arm_voltage /= v_bat_a; //scale from -1 to 1 for the talon // max voltage is positive

	arm_voltage *= -1.0; //set AT END //reverse direction
  counter++;
  frc::SmartDashboard::PutNumber("output counter", counter);
	talonArm->Set(ControlMode::PercentOutput, arm_voltage);
}

void Arm::UpperSoftLimit() {
  // TODO: may need to differnetiate between whether the elevator_high and the top cago state, elevator mightbe able to be high in different cases
	if (arm_state = HIGH_CARGO_STATE) {
		if ((arm_pos >= UP_LIMIT_A) && (arm_voltage > UP_VOLT_LIMIT_A) && is_init_arm) { //at max height and still trying to move up
			arm_voltage = 0.0; //shouldn't crash
			arm_safety = "top soft limit";
		}
	} else {
		if ((arm_pos >= BACK_LIMIT_A) && (arm_voltage > 0.0)
			 && is_init_arm) { //at max height and still trying to move up //no upper soft limit when initializing
		 arm_voltage = 0.0;
		 arm_safety = "top soft limit";
	 }
	}

}

void Arm::LowerSoftLimit() {
	//SOFT LIMIT BOTTOM
	if (IsAtBottomArm()) { //hall effect returns 0 when at bottom. we reverse it here
		// ZeroEnc(); //will not run after 2nd time (first time is in teleop init)
		if (arm_pos <= DOWN_LIMIT_A && arm_voltage < DOWN_VOLT_LIMIT_A) { //but hall effect goes off at 0.35
			arm_voltage = 0.0;
			arm_safety = "bot hall eff";
		}
	}
}

void Arm::StallSafety() {
	// STALL
	if (std::abs(GetAngularVelocity()) <= STALL_VEL_A && std::abs(arm_voltage) > STALL_VOLT_A) { //outputting current, not moving, should be moving
		encoder_counter++;
	} else {
		encoder_counter = 0;
	}
	if (encoder_counter > 10) {
		arm_voltage = 0.0;
		arm_safety = "stall";
	}
}

void Arm::CapVoltage() {
	// CAP VOLTAGE
	if (arm_voltage > MAX_VOLTAGE_A) {
		arm_voltage = MAX_VOLTAGE_A;
		arm_safety = "clipped";
	} else if (arm_voltage < MIN_VOLTAGE_A) {
		arm_voltage = MIN_VOLTAGE_A;
		arm_safety = "clipped";
	}
}

double Arm::GetAngularVelocity() {
	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel =
			(talonArm->GetSensorCollection().GetQuadratureVelocity()
					/ (TICKS_PER_ROT_A) * (2.0 * PI) * (10.0) * -1.0);
	//double ang_vel = 0.0;

	return ang_vel;
}

double Arm::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos =
			(talonArm->GetSensorCollection().GetQuadraturePosition()
					/ (TICKS_PER_ROT_A) * (2.0 * PI) * -1.0);

	double offset_pos = .35; //amount that the arm will stick up in radians

  return ang_pos;
	// return ang_pos + offset_pos;
}

bool Arm::IsAtBottomArm() {
	return !hallEffectArm->Get();
}

void Arm::StopArm() {
	talonArm->Set(ControlMode::PercentOutput, 0.0);
}

void Arm::ArmStateMachine() {
  switch (arm_state) {

    case INIT_STATE:
		if (is_init_arm) {
			arm_state = DOWN_STATE; //PUT BACK IN
		} else {
			InitializeArm();
		}
		last_arm_state = INIT_STATE;
    break;

    case UP_STATE:
    UpdateArmProfile(UP_STATE, UP_ANGLE);
    break;

    case HIGH_CARGO_STATE:
		UpdateArmProfile(HIGH_CARGO_STATE, HIGH_ANGLE);
    break;

    case MID_CARGO_STATE:
		UpdateArmProfile(MID_CARGO_STATE, MID_ANGLE);
    break;

    case LOW_CARGO_STATE:
		UpdateArmProfile(LOW_CARGO_STATE, LOW_ANGLE);
    break;

		case DOWN_STATE:
		UpdateArmProfile(DOWN_STATE, DOWN_ANGLE);
		break;

    case STOP_ARM_STATE:
		StopArm();
		last_arm_state = STOP_ARM_STATE;
		break;
  	}

		frc::SmartDashboard::PutNumber("ARM ENC",
				talonArm->GetSensorCollection().GetQuadraturePosition());

		frc::SmartDashboard::PutString("ARM", arm_states[arm_state]);
}

// TODO: remove current_state and just use arm_state
void Arm::UpdateArmProfile(int current_state, double angle) {

  // First time in the new state, need to update motion profile
	if (last_arm_state != arm_state) {
		arm_profiler->SetFinalGoalArm(angle);
    arm_profiler->SetInitPosArm(GetAngularPosition());
    frc::SmartDashboard::PutNumber("ANGLE", angle);
	}
	last_arm_state = current_state;

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
