#include "Climber.h"


const int INIT_STATE = 0;
const int STOP_STATE = 1;
const int UP_STATE = 2;
const int DOWN_STATE = 3;

const float output = 0.3;

int climber_state = INIT_STATE;


Climber::Climber() {
  talonClimb = new TalonSRX(42);
  elevator_motion_profiler = new ElevatorMotionProfiler(0.5, 2.0, 0.02);
  elevator = new Elevator(elevator_motion_profiler);
  ahrs = new AHRS(SerialPort::kUSB);
}

void Climber::Up() {
  talonClimb->Set(ControlMode::PercentOutput, output);
}

void Climber::Down() {
  talonClimb->Set(ControlMode::PercentOutput, -output);
}

void Climber::Stop() {
  talonClimb->Set(ControlMode::PercentOutput, 0.0);
}

void Climber::ClimberStateMachine(){
  switch (climber_state) {
    case INIT_STATE:
      SmartDashboard::PutString("Climber State", "INIT");
      climber_state = STOP_STATE;
      break;
    case STOP_STATE:
      SmartDashboard::PutString("Climber State", "STOP");
      if (climber_state) {
        Stop();
      }
      break;
    case UP_STATE:
      SmartDashboard::PutString("Climber State", "UP");
      if (climber_state) {
        Up();
      }
      break;
    case DOWN_STATE:
      SmartDashboard::PutString("Climber State", "DOWN");
      if (climber_state) {
        elevator->ff = ff;
        Down();
      }
      break;
  }
}
