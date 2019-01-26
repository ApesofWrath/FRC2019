#include "Elevator.h"

#define PI 3.14159265

const int _ROCKET_TOP_CARGO = 0;
const int _ROCKET_MID_CARGO = 1;
const int _ROCKET_BOTTOM_CARGO = 2;
const int _ROCKET_TOP_HATCH = 3;
const int _ROCKET_MID_HATCH = 4;
const int _BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
const int _BAY_CARGO = 6;

int encoder_counter_e = 0;
double elevator_voltage = 0.0;

Elevator::Elevator(ElevatorMotionProfiler *elevator_profiler_) {

  elevator_state = BOTTOM_HATCH;

  elevator_profiler = elevator_profiler_;

  SetupTalon1(); //TODO: check if functions can be in constructor
  SetupTalon2();

  talonElevator1 = new TalonSRX(TALON_ID_1);

}

void Elevator::SetupTalon1() {


  // Previous Year's code

	// talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	// talonElevator1->EnableCurrentLimit(false);
	// talonElevator1->ConfigContinuousCurrentLimit(40, 0);
	// talonElevator1->ConfigPeakCurrentLimit(80, 0);
	// talonElevator1->ConfigPeakCurrentDuration(100, 0);
  //
	// talonElevator1->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, 0);
	// talonElevator1->ConfigVelocityMeasurementWindow(5, 0); //5 samples for every talon return
	// talonElevator1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	// talonElevator1->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 0); //for getselectedsensor //getselectedsensor defaults to 10ms anyway. don't use getsensorcollection because that defaults to 160ms

}

void Elevator::SetupTalon2() {
  // additional setup for talon2, previous year's code below

  // if (TALON_ID_2 >= 0) {
		// talonElevator2 = new TalonSRX(TALON_ID_2); //0
	// 	talonElevator2->Set(ControlMode::Follower, TALON_ID_1); //re-slaved
	// 	talonElevator2->EnableCurrentLimit(false);
	// 	talonElevator2->ConfigContinuousCurrentLimit(40, 0);
	// 	talonElevator2->ConfigPeakCurrentLimit(80, 0);
	// 	talonElevator2->ConfigPeakCurrentDuration(100, 0);
	// }
}

void Elevator::ElevatorStateMachine() {
  PrintElevatorInfo();

  switch (elevator_state) {
    case _ROCKET_TOP_CARGO:
     CheckElevatorGoal(_ROCKET_TOP_CARGO, ROCKET_TOP_CARGO_POS);
    break;

    case _ROCKET_MID_CARGO:
      CheckElevatorGoal(_ROCKET_MID_CARGO, ROCKET_MID_CARGO_POS);
    break;

    case _ROCKET_BOTTOM_CARGO:
      CheckElevatorGoal(_ROCKET_TOP_CARGO, ROCKET_TOP_CARGO_POS);
    break;

    case _ROCKET_TOP_HATCH:
      CheckElevatorGoal(_ROCKET_TOP_HATCH, ROCKET_TOP_HATCH_POS);
    break;

    case _ROCKET_MID_HATCH:
      CheckElevatorGoal(_ROCKET_MID_HATCH, ROCKET_MID_HATCH_POS);
    break;

    case _BOTTOM_HATCH:
      CheckElevatorGoal(_BOTTOM_HATCH, BOTTOM_HATCH_POS);
    break;

    case _BAY_CARGO:
      CheckElevatorGoal(_BAY_CARGO, BAY_CARGO_POS);
    break;
  }
}

void Elevator::CheckElevatorGoal(int elevator_state, double goal_pos) {
  if (last_elevator_state != elevator_state) {
    elevator_profiler->SetFinalGoalElevator(goal_pos);
    elevator_profiler->SetInitPosElevator(GetElevatorPosition());
  }
  last_elevator_state = elevator_state;
}

void Elevator::ManualElevator() {
  PrintElevatorInfo();
  // Need to implement joyOpElev joystick
      // double output = (joyOpElev->GetY()) * 0.5 * 12.0; //multiply by voltage because setvoltageelevator takes voltage
    	// SetVoltage(output);
}

void Elevator::Stop() {
  talonElevator1->Set(ControlMode::PercentOutput, 0.0);
}

double Elevator::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int elev_pos = talonElevator1->GetSelectedSensorPosition(0);

	double elevator_pos = ((elev_pos - position_offset_e) / TICKS_PER_ROT_E) //position offset to zero
	* (PI * PULLEY_DIAMETER) * -1.0;

	return elevator_pos;

}

double Elevator::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel =
	(talonElevator1->GetSelectedSensorVelocity(0)
	/ (TICKS_PER_ROT_E)) * (PULLEY_DIAMETER * PI) * (10.0)
	* -1.0;
	return elevator_vel;

}

// Change the height based on the
void Elevator::Move() {

  UpdateMoveCoordinates();
  UpdateMoveError();

  v_bat_e = 12.0;

  // Need to go down
  if (elevator_profiler->GetFinalGoalElevator() < elevator_profiler->GetInitPosElevator()) { //can't be the next goal in case we get ahead of the profiler
  	UpdateMovingDirection(1.0, 0.55, K_down_e);
  } else {
  	UpdateMovingDirection(1.0, ff_percent_e, K_up_e);
  }

  UpdateVoltage();
  SetVoltage(u_e);
}

void Elevator::UpdateVoltage() {
  u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]);
	u_e += ff + offset;
}

void Elevator::UpdateMoveCoordinates() {
  std::vector<std::vector<double> > ref_elevator = elevator_profiler->GetNextRefElevator();

	current_pos_e = GetElevatorPosition();//GetElevatorPosition(); //TAKE THIS BACK OUT
	current_vel_e = GetElevatorVelocity();//GetElevatorVelocity();


	goal_pos_e = ref_elevator[0][0];
	goal_vel_e = ref_elevator[1][0];
}

void Elevator::UpdateMoveError() {
  error_e[0][0] = goal_pos_e - current_pos_e;
	error_e[1][0] = goal_vel_e - current_vel_e;
}

void Elevator::UpdateMovingDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_) {
  K_e = K_e_;
	offset = offset_; //dampen
	ff = (Kv_e * goal_vel_e * v_bat_e) * percent;
}

// Calculate and output to talons
void Elevator::SetVoltage(double voltage_) {
  elevator_voltage = voltage_;
  // SAEFTY CHECKS
  elev_safety = "NONE"; // Default value

  // Safeties set output voltage = 0;
  StallSafety();
  UpperSoftLimit();
  LowerSoftLimit();
  TopHallEffectSafety();
	BottomHallEffectSafety();
  ArmSafety();

  frc::SmartDashboard::PutString("ELEVATOR SAFETY", elev_safety);

  // Output Voltage
  ZeroElevator();
  CapVoltage();
  ScaleOutput();
  InvertOutput();
	OutputToTalon();

  frc::SmartDashboard::PutNumber("ELEV VOLT: ", elevator_voltage);
}

std::string Elevator::GetState() {
  return elev_state[elevator_state];
}

void Elevator::PrintElevatorInfo() {
  frc::SmartDashboard::PutString("ELEV: ", GetState());

  frc::SmartDashboard::PutNumber("ELEV CUR 1", talonElevator1->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("ELEV CUR 2", talonElevator2->GetOutputCurrent());

  frc::SmartDashboard::PutNumber("ElEV ENC", talonElevator1->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

  frc::SmartDashboard::PutBoolean("TOP HALL", IsAtTopElevator());
  frc::SmartDashboard::PutBoolean("BOT HALL", IsAtBottomElevator());
}

// converts the voltage into a percent for output
void Elevator::ScaleOutput() {
     elevator_voltage /= 12.0;
}

// changes the direction of the motor output
void Elevator::InvertOutput() {
     elevator_voltage *= -1.0; //reverse at END
}


void Elevator::OutputToTalon() {
     talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);
}

void Elevator::CapVoltage() {
     if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}
}

void Elevator::ZeroElevator() {
  if (!is_elevator_init) { //changed this to just zero on start up (as it always be at the bottom at the start of the match)
		if (ZeroEncs()) { //successfully zeroed one time
			is_elevator_init = true;
		}
	}
}

void Elevator::StallSafety() {
	if (std::abs(GetElevatorVelocity()) <= 0.05
        && std::abs(elevator_voltage) > 3.0) { //this has to be here at the end
		encoder_counter_e++;
	} else {
		encoder_counter_e = 0;
	}
	if (encoder_counter_e > STALLS_TILL_STOP) { //bypass the initial high voltage to accelerate from 0
		elevator_voltage = 0.0;
		elev_safety = "stall";
	}
}

void Elevator::ArmSafety() {
  //Last Year's code
  	// if (keep_elevator_up) {
  	// 	elevator_voltage = 1.0;
  	// 	elev_safety = "arm safety";
  	// }
}

void Elevator::BottomHallEffectSafety() {
	if (IsAtBottomElevator() && elevator_voltage > 0.2) { //elevator_voltage is actually reverse
		elev_safety = "bot hall eff";
		elevator_voltage = 0.0;
	}
}

void Elevator::TopHallEffectSafety() {
	if (IsAtTopElevator() && elevator_voltage < -0.2) { //elevator_voltage is actually reverse
		elev_safety = "top hall eff";
		elevator_voltage = 0.0;
	}
}

void Elevator::LowerSoftLimit() {
     if (GetElevatorPosition() <= (-0.05) && elevator_voltage < 0.0) {  //lower soft limit
		elevator_voltage = 0.0;
		elev_safety = "lower soft";
	}
}

void Elevator::UpperSoftLimit() {
     //TODO: may need to change order
	if (GetElevatorPosition() >= (0.92) && elevator_voltage > 0.0) { //upper soft limit //TODO: separate carr, mds top height
		elevator_voltage = 0.0;
		elev_safety = "upper soft";
	}
}




bool Elevator::IsAtBottomElevator() {
	if (!hallEffectBottom->Get()) {
		return true;
	} else {
		return false;
	}
}

bool Elevator::IsAtTopElevator() {
	if (!hallEffectTop->Get()) {
		return true;
	} else {
		return false;
	}
}

// something that goes in the ElevatorState?
double Elevator::GetVoltageElevator() { //not voltage sent to the motor. the voltage the controller sends to SetVoltage()
	return u_e;
}

void Elevator::SetZeroOffset() {

	position_offset_e =
	talonElevator1->GetSelectedSensorPosition(0);
}

bool Elevator::ZeroEncs() {

	if (zeroing_counter_e < 1) {
		//Great Robotic Actuation and Controls Execution ()
		SetZeroOffset();
		zeroing_counter_e++;
		return true;
	} else {
		return false;
	}

}
