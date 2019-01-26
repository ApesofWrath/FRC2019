#include "Elevator.h"

#define PI 3.14159265

const int _ROCKET_TOP_CARGO = 0;
const int _ROCKET_MID_CARGO = 1;
const int _ROCKET_BOTTOM_CARGO = 2;
const int _ROCKET_TOP_HATCH = 3;
const int _ROCKET_MID_HATCH = 4;
const int _BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
const int _BAY_CARGO = 6;

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

	///	SmartDashboard::PutNumber("Actual Vel", current_vel_e);
	//	SmartDashboard::PutNumber("Actual Pos", current_pos_e);

	goal_pos_e = ref_elevator[0][0];
	goal_vel_e = ref_elevator[1][0];

	//	SmartDashboard::PutNumber("Goal Vel", goal_vel_e);
	//	SmartDashboard::PutNumber("Goal Pos", goal_pos_e);
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

  // SAEFTY CHECKS

  voltage_ /= 12.0;
  voltage_ *= -1.0;

  frc::SmartDashboard::PutNumber("ELEV VOLT: ", voltage_);
  talonElevator1->Set(ControlMode:: PercentOutput, voltage_);  // Talon 2 is follower to talon1

}

std::string Elevator::GetState() {
  return elev_state[elevator_state];
}

void Elevator::PrintElevatorInfo() {
  frc::SmartDashboard::PutString("ELEV: ", elev_state[elevator_state]);

  // Other prints here
}
