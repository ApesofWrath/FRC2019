#include "Arm.h"

int counter = 0;
int encoder_counter_i = 0;
int stall_count = 0;

const int INIT_STATE = 0;
const int REST_STATE = 1;
const int HATCH_STATE = 2;
const int CARGO_STATE = 3;
const int HIGH_CARGO_STATE = 4;
const int GET_HATCH_GROUND_STATE = 5;
const int REINIT_STATE = 6;
const int STOP_ARM_STATE = 7;

Arm::Arm(ArmMotionProfiler *arm_profiler_) {

  talonArm = new TalonSRX(ARM_TALON_ID);

  talonArm->ConfigFactoryDefault();

  talonArm->ConfigVoltageCompSaturation(12.0);
  talonArm->EnableVoltageCompensation(true);

  //  talonArm->ConfigForwardSoftLimitThreshold(16000);
  //  talonArm->ConfigReverseSoftLimitThreshold(18300); or something

  talonArm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, TIMEOUT_MS);//configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    talonArm->SetSensorPhase(true);
    talonArm->SetInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    talonArm->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, FRAME_PERIOD, TIMEOUT_MS);
    talonArm->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, FRAME_PERIOD, TIMEOUT_MS);

    /* Set the peak and nominal outputs */
    talonArm->ConfigNominalOutputForward(0, TIMEOUT_MS);
    talonArm->ConfigNominalOutputReverse(0, TIMEOUT_MS);
    talonArm->ConfigPeakOutputForward(1, TIMEOUT_MS);
    talonArm->ConfigPeakOutputReverse(-1, TIMEOUT_MS);

    /* Set Motion Magic gains in slot0 - see documentation */
    talonArm->SelectProfileSlot(0, 0);
    talonArm->Config_kF(0, Kf, TIMEOUT_MS); //1023/ max speed
    talonArm->Config_kP(0, Kp, TIMEOUT_MS);
    talonArm->Config_kI(0, Ki, TIMEOUT_MS); //middle number is the gain
    talonArm->Config_kD(0, Kd, TIMEOUT_MS);

    talonArm->Config_IntegralZone(0, 1000, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    talonArm->ConfigMotionCruiseVelocity(ENC_CRUISE_VEL, TIMEOUT_MS);
    talonArm->ConfigMotionAcceleration(ENC_CRUISE_ACC, TIMEOUT_MS);

    arm_profiler = arm_profiler_;

    hallEffectArm = new frc::DigitalInput(HALL_EFF_ARM_ID);

  }

//call this over and over with current angle until desired angle reached
  double Arm::Interpolate(double angle) {

   //  std::vector<double> angles = {2.8, 2.6, 2.4};
   //  std::vector<double> outputs = {};
   //
   //  int size = angles.size();
   //
   //  int i = 0;
   //  for (int j = 0; j < angles.size() - 2; j++) { //find right bound
   //    if (angle >= angles[j]) {
   //      i++;
   //    } else {
   //      break;
   //    }
   //  }
   //
   // double xL = angles[i];
   // double yL = outputs[i];
   // double xR = angles[i-1];
   // double yR = outputs[i-1];
   //
   // double dydx = ( yR - yL ) / ( xR - xL );
   //
   // return yL + dydx * ( angle - xL );
return 0;
  }

  void Arm::InitializeArm() {
    if (!is_init_arm) { //this has to be here for some reason
      SetVoltageArm(3.5); //	MAYBE SHOULDNT BE NEGATIVE - CHECK
    }

    ZeroEnc(); //won't start at 0 on this year's robot!
  }

  void Arm::ManualArm(frc::Joystick *joyOpArm) {

//    frc::SmartDashboard::PutNumber("ARM CUR", talonArm->GetOutputCurrent());
    //  frc::SmartDashboard::PutNumber("ARM ENC", talonArm->GetSensorCollection().GetQuadraturePosition());

//    frc::SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

    double output = joyOpArm->GetY() * 0.5 * 1.0 * MAX_VOLTAGE_A;

  //  frc::SmartDashboard::PutNumber("ARM OUTPUT", output);
    // TODO, IMPORTANT: find out who wrote this
    // frc::SmartDashboard::PutString("HasRobotVoted", "True")

    SetVoltageArm(output);

  }

  void Arm::PrintArmInfo() {
    frc::SmartDashboard::PutBoolean("ARM HALL EFFECT", hallEffectArm->Get());
    frc::SmartDashboard::PutNumber("ARM POS", GetAngularPosition());
    frc::SmartDashboard::PutNumber("ARM VEL", GetAngularVelocity());
  //  frc::SmartDashboard::PutNumber("counter", counter);
    // frc::SmartDashboard::PutNumber("ARM REF POS", ref_arm[0][0]);
    // frc::SmartDashboard::PutNumber("ARM REF VEL", ref_arm[1][0]);

    frc::SmartDashboard::PutNumber("ARM ERR POS", error_a[0][0]);
    frc::SmartDashboard::PutNumber("ARM ERR VEL", error_a[1][0]);

    frc::SmartDashboard::PutNumber("ARM CONT VOLT", u_a);
  }

  // void Arm::RunController() {
  //
  //
  // 	// Shaking: if (IsAtBottomArm() && std::abs(error_a[0][0]) > 0.4) {arm_state = GET_HATCH_GROUND_STATE;}
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

      arm_state = CARGO_STATE;
      // frc::SmartDashboard::PutNumber("ARM HALL EFF", IsAtBottomArm()); // actually means not at bottom //0 means up// 1 means down

      arm_safety = "NONE";

      arm_pos = GetAngularPosition();
      arm_vel = GetAngularVelocity();

      UpperSoftLimit();
      LowerSoftLimit();
      StallSafety();
      CapVoltage();

      //zero height moves up sometimes. this will make sure the arm goes all the way down every time
      if ((arm_state == GET_HATCH_GROUND_STATE) && arm_pos <= 0.4) { // TODO: change back to 0.2 after fix zeroing encoders
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

      if (!elevator_high) {
        if ((arm_pos >= UP_LIMIT_A) && (arm_voltage > UP_VOLT_LIMIT_A) && is_init_arm) { //at max height and still trying to move up
          arm_voltage = 0.0; //shouldn't crash
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

    bool Arm::StallSafety() {

      frc::SmartDashboard::PutNumber("ARM STALLS", stall_count);
      frc::SmartDashboard::PutNumber("ARM error", talonArm->GetSelectedSensorPosition(0) - talonArm->GetActiveTrajectoryPosition());
      // STALL
      if ((std::abs(GetAngularVelocity()) < 0.1) && (std::abs(talonArm->GetOutputCurrent())) > 6.0 && std::abs(talonArm->GetSelectedSensorPosition(0) - talonArm->GetActiveTrajectoryPosition()) > 0.2 * RAD_TO_ENC) { //GetActiveTrajectoryVelocity won't work because targ vel is 0 at end of profile
        encoder_counter_i++;
      } else {
        encoder_counter_i = 0;
      }
      if (encoder_counter_i > 8) {
        stall_count++;
        frc::SmartDashboard::PutString("ARM SAFETY", "stall");
        return true;
      } else {
      return false;
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

    //returns rad/s
    double Arm::GetAngularVelocity() {
      //Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
      //multiply by 2pi to get into radians per second (2pi radians are in one revolution)
      return (talonArm->GetSelectedSensorVelocity(0)
      / (TICKS_PER_ROT_A * ENC_GEAR_RATIO) * (2.0 *  3.14159265) * (10.0));

    }

    //returns rad
    double Arm::GetAngularPosition() {

      return (talonArm->GetSelectedSensorPosition(0)
      / (TICKS_PER_ROT_A * ENC_GEAR_RATIO) * (2.0 *  3.14159265));

    }

    bool Arm::IsAtBottomArm() {
      return !hallEffectArm->Get();
    }

    void Arm::StopArm() {
      talonArm->Set(ControlMode::PercentOutput, 0.0);
    }

    void Arm::ArmStateMachine() {

    //  frc::SmartDashboard::PutNumber("arm vel", GetAngularVelocity());
      frc::SmartDashboard::PutNumber("arm current",talonArm->GetOutputCurrent());
  //    (GetAngularVelocity()) <= 0.1) && (std::abs(talonArm->GetOutputCurrent())) > 2.0)

       if (!StallSafety()) {

      //   frc::SmartDashboard::PutString("ARM SAFETY", "none");

      switch (arm_state) {

        case INIT_STATE:
        frc::SmartDashboard::PutString("ARM", "init");
        if (std::abs(talonArm->GetSelectedSensorPosition(0) - ENC_START_ANGLE) < 10) {
          arm_state = REST_STATE;
        } else {
          talonArm->SetSelectedSensorPosition(ENC_START_ANGLE, 0, 100);
        }
        last_arm_state = INIT_STATE;
        break;

        case REST_STATE:
        frc::SmartDashboard::PutString("ARM", "rest");
        if (talonArm->GetSelectedSensorPosition(0) > (ENC_REST_ANGLE - 200)) { //std::abs(talonArm->GetSelectedSensorPosition() - ENC_REST_ANGLE) < 200
          StopArm();
        } else {
          talonArm->Set(ControlMode::MotionMagic, ENC_REST_ANGLE, DemandType_ArbitraryFeedForward, -0.089 * (GetAngularPosition()) * (GetAngularPosition()) + 0.0655681 * (GetAngularPosition()) + 0.293378);
        }
        break;

        case HATCH_STATE:
        frc::SmartDashboard::PutString("ARM", "hatch");
        //UpdateArmProfile(HATCH_STATE, HATCH_ANGLE);
        talonArm->Set(ControlMode::MotionMagic, ENC_HATCH_ANGLE, DemandType_ArbitraryFeedForward, -0.089 * (GetAngularPosition()) * (GetAngularPosition()) + 0.0655681 * (GetAngularPosition()) + 0.293378);
        break;

        case CARGO_STATE:
        frc::SmartDashboard::PutString("ARM", "cargo");
        //UpdateArmProfile(CARGO_STATE, CARGO_ANGLE);
        talonArm->Set(ControlMode::MotionMagic, ENC_CARGO_ANGLE, DemandType_ArbitraryFeedForward,-0.089 * (GetAngularPosition()) * (GetAngularPosition()) + 0.0655681 * (GetAngularPosition()) + 0.293378);
        break;

        case HIGH_CARGO_STATE:
        frc::SmartDashboard::PutString("ARM", "high cargo");
        //UpdateArmProfile(HIGH_CARGO_STATE_H, HIGH_CARGO_ANGLE);
        talonArm->Set(ControlMode::MotionMagic, ENC_HIGH_CARGO_ANGLE, DemandType_ArbitraryFeedForward, -0.089 * (GetAngularPosition()) * (GetAngularPosition()) + 0.0655681 * (GetAngularPosition()) + 0.293378);
        break;

        case GET_HATCH_GROUND_STATE:
        frc::SmartDashboard::PutNumber("arb ff", cos(GetAngularPosition() - arb_ff_offset) * arb_ff_percent);
        frc::SmartDashboard::PutString("ARM", "get hatch ground");
        //UpdateArmProfile(GET_HATCH_GROUND_STATE_H, GET_HATCH_GROUND_ANGLE);
        talonArm->Set(ControlMode::MotionMagic, ENC_GET_HATCH_GROUND_ANGLE, DemandType_ArbitraryFeedForward,-0.089 * (GetAngularPosition()) * (GetAngularPosition()) + 0.0655681 * (GetAngularPosition()) + 0.293378);
        //
        break;

        case REINIT_STATE:
        frc::SmartDashboard::PutString("ARM", "reinit");
        //UpdateArmProfile(EXTRA_STATE_H, EXTRA_ANGLE);
        // if (GetAngularPosition() + 0.2 < EXTRA_ANGLE) {
        //talonArm->Set(ControlMode::PercentOutput, Interpolate(GetAngularPosition() + 0.2));
      //}
        talonArm->Set(ControlMode::PercentOutput, 0.7);
        break;

        case STOP_ARM_STATE:
        frc::SmartDashboard::PutString("ARM", "stop");
        StopArm();
        last_arm_state = STOP_ARM_STATE;
        break;
      }
      } else {
       StopArm();
      }
      // frc::SmartDashboard::PutNumber("ARM ENC",
      // 		talonArm->GetSensorCollection().GetQuadraturePosition());
      //
      // frc::SmartDashboard::PutString("ARM", arm_states[arm_state]);
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

    //NOT REALLY ZEROING
    void Arm::ZeroEnc() { //called in Initialize() and in SetVoltage()
      if (zeroing_counter_a < 2) {
        /* Zero the sensor */
        talonArm->SetSelectedSensorPosition(ENC_REST_ANGLE, 0, 10);
        //  talonArm->GetSensorCollection().SetQuadraturePosition(0, 0);
        zeroing_counter_a++;
      } else {
        is_init_arm = true;
      }
    }
