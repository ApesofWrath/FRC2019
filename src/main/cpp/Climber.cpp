#include "Climber.h"

const int INIT_STATE = 0;
const int STOP_STATE = 1;
const int UP_STATE = 2;
const int DOWN_STATE = 3;

int climber_state = INIT_STATE;

Climber::Climber(AHRS *ahrs_) {
  talonClimb1 = new TalonSRX(-0);
  talonClimb2 = new TalonSRX(-0);
  ahrs_climber = ahrs_;

  talonClimb2->Follow(*talonClimb1);
  //
  // talonClimb1->ConfigForwardSoftLimitThreshold(16000);
  // talonClimb2->ConfigForwardSoftLimitThreshold(16000);
  //
  // talonClimb1->ConfigReverseSoftLimitThreshold(100);
  // talonClimb2->ConfigReverseSoftLimitThreshold(100);

  talonClimb1->ConfigFactoryDefault();
  talonClimb2->ConfigFactoryDefault();
  talonClimb1->ConfigVoltageCompSaturation(12.0);
  talonClimb2->ConfigVoltageCompSaturation(12.0);
  talonClimb1->EnableVoltageCompensation(true);
  talonClimb2->EnableVoltageCompensation(true);
  /* Configure Sensor Source for Pirmary PID */
  talonClimb1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

  //  talonClimb1->SetInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    talonClimb1->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    talonClimb1->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    talonClimb1->ConfigNominalOutputForward(0, 10);
    talonClimb1->ConfigNominalOutputReverse(0, 10);
    talonClimb1->ConfigPeakOutputForward(1, 10);
    talonClimb1->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    talonClimb1->SelectProfileSlot(0, 0);
    talonClimb1->Config_kF(0, Kf_c, 10);
    talonClimb1->Config_kP(0, Kp_c, 10);
    talonClimb1->Config_kI(0, Ki_c, 10);
    talonClimb1->Config_kD(0, Kd_c, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    talonClimb1->ConfigMotionCruiseVelocity(ENC_CRUISE_VEL_C, 10);//3120
    talonClimb1->ConfigMotionAcceleration(ENC_CRUISE_ACC_C, 10);

}

void Climber::Up() {
  talonClimb1->Set(ControlMode::MotionMagic, 0, DemandType_ArbitraryFeedForward, 0.0);
}

void Climber::Down() {
  talonClimb1->Set(ControlMode::MotionMagic, 0, DemandType_ArbitraryFeedForward, 0.0);
}

void Climber::Stop() {
  talonClimb1->Set(ControlMode::PercentOutput, 0.0);
}

void Climber::ClimberStateMachine() {

  switch (climber_state) {
    case INIT_STATE:
      SmartDashboard::PutString("Climber", "INIT");
      if (std::abs(talonClimb1->GetSelectedSensorPosition(0)) < 10) {
        climber_state = STOP_STATE;
      } else {
        talonClimb1->SetSelectedSensorPosition(0, 0, 100);
      }
      climber_state = STOP_STATE;
      break;
    case STOP_STATE:
      SmartDashboard::PutString("Climber", "STOP");
      Stop();
      break;
    case UP_STATE:
      SmartDashboard::PutString("Climber", "UP");
      Up();
      break;
    case DOWN_STATE:
      SmartDashboard::PutString("Climber", "DOWN");
      Down();
      break;
  }
}

double Climber::GetClimberPosition() {

  return (((talonClimb1->GetSelectedSensorPosition(0)) / 4096.0)
  * (3.1415 * 0.0381) * 2.0); //*2 for cascading elev

}
