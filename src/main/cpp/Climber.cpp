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

  //feedforward
  target_left = elevator_climber->INIT_CLIMB_HEIGHT - elevator_climber->GetElevatorPosition(); //should be positive
  target_right = target_left;

  //forward/backward controller
  forward_angle_error = ahrs_pitch * 3.14159 / 180.0; //same for both legs

  current_left_height = sin(forward_angle_error) * robot_width + elevator_climber->GetElevatorPosition();
  current_right_height = current_left_height; //for purposes of forward angle

  forward_left_error = target_left - current_left_height;
  forward_right_error = target_right - current_right_height;

  forward_left_output = forward_left_error * Kp_l_c_f;
  forward_right_output = forward_right_error * Kp_r_c_f;

  //side/side controller
  side_angle_error = ahrs_roll * 3.14159 / 180.0;
  side_left_error += sin(side_angle_error) * robot_width / 2.0; //or opposite +-
  side_right_error -= sin(side_angle_error) * robot_width / 2.0;

  side_left_output = left_error * Kp_l_c_s;
  side_right_output = right_error * Kp_r_c_s;

  //combine, + ff
  left_output = forward_left_output + side_left_output + Kf_l_c * target_left;
  right_output = forward_right_output + side_right_output + Kf_r_c * target_right;

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
