#include "../include/Elevator.h"
// Look into Heirarchical state machine

// Elevator vs. Arm vs. intake?
  // Where does controlling the arm go?

#define PI 3.14159265

// Moved to .h file, may have to add copies to this file
    // int last_elevator_state;
    // int current_state;
    const int ROCKET_TOP_CARGO = 0;
    const int ROCKET_MID_CARGO = 1;
    const int ROCKET_BOTTOM_CARGO = 2;
    const int ROCKET_TOP_HATCH = 3;
    const int ROCKET_MID_HATCH = 4;
    const int BOTTOM_HATCH = 5; // Same for rocket and cargo bay, only need one
    const int BAY_CARGO = 6;
// ¿¿DIFFERNET STATES FOR LOADING STATION?? (HPS_STATE)


// Setup talon id's here or remove these and add local varaibles in constructor and add id param for setup functions
const int TALON_ID_1 = -1;
const int TALON_ID_2 = -1;

double Kv_e;

double offset = 0.0;
double ff = 0.0; //feedforward
double u_e = 0.0; //this is the output in volts to the motor
double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

double position_offset_e = 0.0;

double current_pos_e = 0.0;
double current_vel_e = 0.0;

// Move error
std::vector<std::vector<double>> error_e;

Elevator::Elevator(ElevatorMotionProfiler *elevator_profiler_, double rocket_top_cargo_pos_, double rocket_mid_cargo_pos_, double rocket_bottom_cargo_pos_, double rocket_top_hatch_pos_, double rocket_mid_hatch_pos_, double bottom_hatch_pos_, double bay_cargo_pos_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_, double ff_percent_e_, double PULLEY_DIAMETER_) {

  current_state = BOTTOM_HATCH;

  elevator_profiler = elevator_profiler_;

  // Move initializations
	error_e = { { 0.0 }, { 0.0 } };
  K_down_e = K_down_e_;
  K_up_e = K_up_e_;
  ff_percent_e = ff_percent_e_;

  PULLEY_DIAMETER = PULLEY_DIAMETER_;

  SetupTalon1();
  SetupTalon2();

  rocket_top_cargo_pos = rocket_top_cargo_pos_;
  rocket_mid_cargo_pos = rocket_mid_cargo_pos_;
  rocket_bottom_cargo_pos = rocket_bottom_cargo_pos_;
  rocket_top_hatch_pos = rocket_top_hatch_pos_;
  rocket_mid_hatch_pos = rocket_mid_hatch_pos_;
  bottom_hatch_pos = bottom_hatch_pos_;
  bay_cargo_pos = bay_cargo_pos_;
}

void Elevator::SetupTalon1() {
  talonElevator1 = new TalonSRX(TALON_ID_1);

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

  switch (current_state) {
    case ROCKET_TOP_CARGO:
     CheckElevatorGoal(ROCKET_TOP_CARGO, rocket_top_cargo_pos);
    break;

    case ROCKET_MID_CARGO:
      CheckElevatorGoal(ROCKET_MID_CARGO, rocket_mid_cargo_pos);
    break;

    case ROCKET_BOTTOM_CARGO:
      CheckElevatorGoal(ROCKET_TOP_CARGO, rocket_top_cargo_pos);
    break;

    case ROCKET_TOP_HATCH:
      CheckElevatorGoal(ROCKET_TOP_HATCH, rocket_top_hatch_pos);
    break;

    case ROCKET_MID_HATCH:
      CheckElevatorGoal(ROCKET_MID_HATCH, rocket_mid_hatch_pos);
    break;

    case BOTTOM_HATCH:
      CheckElevatorGoal(BOTTOM_HATCH, bottom_hatch_pos);
    break;

    case BAY_CARGO:
      CheckElevatorGoal(BAY_CARGO, bay_cargo_pos);
    break;
  }
}

void Elevator::CheckElevatorGoal(int current_state, double goal_pos) {
  if (last_elevator_state != current_state) {
    elevator_profiler->SetFinalGoalElevator(goal_pos);
    elevator_profiler->SetInitPosElevator(GetElevatorPosition());
  }
  last_elevator_state = current_state;
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
void Elevator::SetVoltage(double votlage_) {

}

std::string Elevator::GetState() {
  return elev_state[current_state];
}

void Elevator::PrintElevatorInfo() {
  frc::SmartDashboard::PutString("ElevatorState: ", elev_state[current_state]);

  // Other prints here
}
