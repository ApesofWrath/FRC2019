#include "Climber.h"

const int INIT_STATE = 0;
const int STOP_STATE = 1;
const int UP_STATE = 2;
const int DOWN_STATE = 3;
int climber_state = INIT_STATE;

Climber::Climber(Elevator *elevator_) {

  elevator_climber = elevator_;

  talonClimb1 = new TalonSRX(-0);
  talonClimb2 = new TalonSRX(-0);

  talonClimb1->ConfigFactoryDefault();
  talonClimb2->ConfigFactoryDefault();
  talonClimb1->ConfigVoltageCompSaturation(12.0);
  talonClimb2->ConfigVoltageCompSaturation(12.0);
  talonClimb1->EnableVoltageCompensation(true);
  talonClimb2->EnableVoltageCompensation(true);

    /* Set the peak and nominal outputs */
  talonClimb1->ConfigNominalOutputForward(0, 10);
  talonClimb1->ConfigNominalOutputReverse(0, 10);
  talonClimb1->ConfigPeakOutputForward(1, 10);
  talonClimb1->ConfigPeakOutputReverse(-1, 10);

  talonClimb2->ConfigNominalOutputForward(0, 10);
  talonClimb2->ConfigNominalOutputReverse(0, 10);
  talonClimb2->ConfigPeakOutputForward(1, 10);
  talonClimb2->ConfigPeakOutputReverse(-1, 10);

}

void Climber::Up() { //bring robot up

  target = elevator_climber->INIT_CLIMB_HEIGHT - elevator_climber->GetElevatorPosition(); //should be positive
  left_error = target - GetClimberPosition();
  right_error = left_error;

  ang_error = ahrs_pitch * 3.14159 / 180.0;
  height_error = sin(ang_error) * robot_width;
  left_error += height_error / 2.0; //or opposite +-
  right_error -= height_error / 2.0;

  right_output = Kp_r_c * right_error + Kf_r_c * target;
  left_output = Kp_l_c * left_error + Kf_l_c * target;

  talonClimb1->Set(ControlMode::PercentOutput, left_output);
  talonClimb2->Set(ControlMode::PercentOutput, right_output);
}

void Climber::Down() { //bring robot down
  talonClimb1->Set(ControlMode::PercentOutput, 0.0);
}

void Climber::Stop() {
  talonClimb1->Set(ControlMode::PercentOutput, 0.0);
}

void Climber::GetPitch(double pitch) {

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
