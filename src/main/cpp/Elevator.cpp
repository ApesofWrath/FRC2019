#include "Elevator.h"

//#define PI 3.14159265

const int INIT_STATE = 0;
const int TOP_CARGO_STATE = 1;
const int MID_CARGO_STATE = 2;
const int BOTTOM_CARGO_STATE = 3;
const int TOP_HATCH_STATE = 4;
const int MID_HATCH_STATE = 5;
const int BOTTOM_HATCH_STATE = 6; // Same for rocket and cargo bay, only need one
const int HOLD_HATCH_STATE = 7;
const int STOP_STATE = 8;
const int LIFTING_ARM_STATE = 9;
const int STATION_HATCH_STATE = 10;

double elevator_voltage = 0.0;

int encoder_counter_e = 0;

Elevator::Elevator(ElevatorMotionProfiler *elevator_profiler_) {

  elevator_state = BOTTOM_HATCH_STATE;

  elevator_profiler = elevator_profiler_;

  talonElevator1 = new TalonSRX(TALON_ID_1);
  talonElevator2 = new VictorSPX(TALON_ID_2);

  talonElevator2->Follow(*talonElevator1);
  //
  // talonElevator1->ConfigForwardSoftLimitThreshold(16000);
  // talonElevator2->ConfigForwardSoftLimitThreshold(16000);
  //
  // talonElevator1->ConfigReverseSoftLimitThreshold(100);
  // talonElevator2->ConfigReverseSoftLimitThreshold(100);

  talonElevator1->ConfigFactoryDefault();
  talonElevator2->ConfigFactoryDefault();
  talonElevator1->ConfigVoltageCompSaturation(12.0);
  talonElevator2->ConfigVoltageCompSaturation(12.0);
  talonElevator1->EnableVoltageCompensation(true);
  talonElevator2->EnableVoltageCompensation(true);
  /* Configure Sensor Source for Pirmary PID */
  talonElevator1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

    talonElevator1->SetInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    talonElevator1->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    talonElevator1->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    talonElevator1->ConfigNominalOutputForward(0, 10);
    talonElevator1->ConfigNominalOutputReverse(0, 10);
    talonElevator1->ConfigPeakOutputForward(1, 10);
    talonElevator1->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    talonElevator1->SelectProfileSlot(0, 0);
    talonElevator1->Config_kF(0, Kf_e, 10);
    talonElevator1->Config_kP(0, Kp_e, 10);
    talonElevator1->Config_kI(0, Ki_e, 10);
    talonElevator1->Config_kD(0, Kd_e, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    talonElevator1->ConfigMotionCruiseVelocity(ENC_CRUISE_VEL_E, 10);//3120
    talonElevator1->ConfigMotionAcceleration(ENC_CRUISE_ACC_E, 10);


    hallEffectTop = new frc::DigitalInput(TOP_HALL);
    hallEffectBottom = new frc::DigitalInput(BOT_HALL);
  }

  void Elevator::ElevatorStateMachine() {
    //PrintElevatorInfo();
  //  frc::SmartDashboard::PutString("ELEV ", GetState());

//if (!StallSafety()) {
    switch (elevator_state) {
      case INIT_STATE:
        frc::SmartDashboard::PutString("ELEV ", "init");
      if (std::abs(talonElevator1->GetSelectedSensorPosition(0)) < 10) {
        elevator_state = BOTTOM_HATCH_STATE;
      } else {
        talonElevator1->SetSelectedSensorPosition(0, 0, 100);
      }
      last_elevator_state = INIT_STATE;
      break;

      case TOP_CARGO_STATE:
        frc::SmartDashboard::PutString("ELEV ", "top cargo");
      //CheckElevatorGoal(TOP_CARGO_STATE, TOP_CARGO_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_TOP_CARGO_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case MID_CARGO_STATE:
        frc::SmartDashboard::PutString("ELEV ", "mid cargo");
      //CheckElevatorGoal(MID_CARGO_STATE, MID_CARGO_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_MID_CARGO_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case BOTTOM_CARGO_STATE:
        frc::SmartDashboard::PutString("ELEV ", "bot cargo");
      //CheckElevatorGoal(BOTTOM_CARGO_STATE, BOTTOM_CARGO_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_BOTTOM_CARGO_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case TOP_HATCH_STATE:
        frc::SmartDashboard::PutString("ELEV ", "top hatch");
      //CheckElevatorGoal(TOP_HATCH_STATE, TOP_HATCH_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_TOP_HATCH_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case MID_HATCH_STATE:
        frc::SmartDashboard::PutString("ELEV ", "mid hatch");
      //CheckElevatorGoal(MID_HATCH_STATE, MID_HATCH_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_MID_HATCH_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case BOTTOM_HATCH_STATE: //lowest  pos
        frc::SmartDashboard::PutString("ELEV ", "bot hatch");
      //CheckElevatorGoal(BOTTOM_HATCH_STATE, BOTTOM_HATCH_POS);
      if (talonElevator1->GetActiveTrajectoryPosition() < 200) { //std::abs(talonElevator1->GetSelectedSensorPosition() - ENC_BOTTOM_HATCH_POS) < 200
        Stop();
      } else {
        talonElevator1->Set(ControlMode::MotionMagic, ENC_BOTTOM_HATCH_POS, DemandType_ArbitraryFeedForward, 0.07);
      }
      break;

      case HOLD_HATCH_STATE:
        frc::SmartDashboard::PutString("ELEV ", "hold hatch");
      //CheckElevatorGoal(HOLD_HATCH_STATE, HOLD_HATCH_POS);
      talonElevator1->Set(ControlMode::MotionMagic, ENC_HOLD_HATCH_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case STOP_STATE:
        frc::SmartDashboard::PutString("ELEV ", "stop");
      Stop();
      last_elevator_state = STOP_STATE;
      break;

      case LIFTING_ARM_STATE:
        frc::SmartDashboard::PutString("ELEV ", "lift arm");
      talonElevator1->Set(ControlMode::MotionMagic, ENC_LIFTING_ARM_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;

      case STATION_HATCH_STATE:
        frc::SmartDashboard::PutString("ELEV ", "statino hatch state");
      talonElevator1->Set(ControlMode::MotionMagic, ENC_STATION_HATCH_POS, DemandType_ArbitraryFeedForward, 0.07);
      break;
    }
  //} else {
//    Stop();
  //  frc::SmartDashboard::PutString("ELEV STALL", "stalled");
  //}
  }

  void Elevator::CheckElevatorGoal(int elevator_state, double goal_pos) {
    if (last_elevator_state != elevator_state) {
      elevator_profiler->SetFinalGoalElevator(goal_pos);
      elevator_profiler->SetInitPosElevator(GetElevatorPosition());
    }
    last_elevator_state = elevator_state;
  }

  void Elevator::ManualElevator(frc::Joystick *joyOpElev) {
    PrintElevatorInfo();
    double y_pos = joyOpElev->GetY();
    double output = (joyOpElev->GetY()) * 0.5 * 12.0; //multiply by voltage because setvoltageelevator takes voltage
    frc::SmartDashboard::PutNumber("output", output);
    frc::SmartDashboard::PutNumber("joystick y", y_pos);
    SetVoltage(output);
  }

  void Elevator::Stop() {
    talonElevator1->Set(ControlMode::PercentOutput, 0.0);
  }

  double Elevator::GetElevatorPosition() {

    return (((talonElevator1->GetSelectedSensorPosition(0)) / TICKS_PER_ROT_E)
    * (3.1415 * PULLEY_DIAMETER) * 2.0); //*2 for cascading elev

  }

  bool Elevator::IsElevatorHigh() {

    return (GetElevatorPosition() > EL_SAFETY_HEIGHT);

  }

  void Elevator::IsArmBack(double arm_angle) {

    keep_elevator_up = (arm_angle > ARM_HEIGHT_SAFETY_HEIGHT) && (elevator_voltage < 0.0);

  }

  double Elevator::GetElevatorVelocity() {
    //native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
    //RPS then muliply by circumference for m/s
  return ((talonElevator1->GetSelectedSensorVelocity(0)
    / (TICKS_PER_ROT_E)) * (PULLEY_DIAMETER * 3.14159) * (10.0));

  }

  void Elevator::Move() {
    if (elevator_state != STOP_STATE) {
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

  frc::SmartDashboard::PutNumber("goal_pos_e", goal_pos_e);
  frc::SmartDashboard::PutNumber("goal_vel_e", goal_vel_e);
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
  //ZeroElevator(); just for init ing
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
  frc::SmartDashboard::PutNumber("ELEV CUR 2", talonElevator1->GetOutputCurrent());

  frc::SmartDashboard::PutNumber("ElEV ENC", talonElevator1->GetSelectedSensorPosition(0));

  frc::SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());
  frc::SmartDashboard::PutNumber("ELEV vel", GetElevatorVelocity());

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

bool Elevator::StallSafety() {
  if (((std::abs(GetElevatorVelocity()) <= 0.1) && (talonElevator1->GetActiveTrajectoryVelocity() > 0.08))) { // std::abs(GetElevatorPosition() - ) <= 0.1
    encoder_counter_e++;
  } else {
    encoder_counter_e = 0;
  }
  if (encoder_counter_e > 3) {
    return true;
    frc::SmartDashboard::PutString("ARM SAFETY", "stall");
  }
  return false;
}

void Elevator::ArmSafety() {

  if (keep_elevator_up) {
    elevator_voltage = 1.0;
    elev_safety = "arm safety";
  }

}

void Elevator::BottomHallEffectSafety() {
  // is called before inverting the volage
  if (IsAtBottomElevator() && elevator_voltage < -0.1) { //elevator_voltage is actually reverse
    elev_safety = "bot hall eff";
    elevator_voltage = 0.0;
  }
}

void Elevator::TopHallEffectSafety() {
  if (IsAtTopElevator() && elevator_voltage > 0.1) { //elevator_voltage is actually reverse
    elev_safety = "top hall eff";
    elevator_voltage = 0.0;
  }
}

void Elevator::LowerSoftLimit() {
  if (GetElevatorPosition() <= (-0.05) && elevator_voltage < 0.0) {
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
  return !hallEffectBottom->Get();
}

bool Elevator::IsAtTopElevator() {
  return !hallEffectTop->Get();
}

// something that goes in the ElevatorState?
double Elevator::GetVoltageElevator() { //not voltage sent to the motor. the voltage the controller sends to SetVoltage()
  return u_e;
}

void Elevator::SetZeroOffset() {
  position_offset_e =
  talonElevator1->GetSelectedSensorPosition(0);
  frc::SmartDashboard::PutNumber("position_offset_e", position_offset_e);
}

bool Elevator::ZeroEncs() {
  if (zeroing_counter_e < 1) {
    // Great Robotic Actuation and Controls Execution ()
    talonElevator1->SetSelectedSensorPosition(0, 0, 10);
    zeroing_counter_e++;
    return true;
  } else {
    return false;
  }

}
