#include "BallIntake.h"

const int INIT_STATE = 0;
const int STOP_STATE = 1;
const int IN_BALL_STATE = 2;
const int OUT_BALL_STATE = 3;

const int STOP_SPEED = 0.2; //placeholder #s put in correct ones
const int IN_SPEED = 0.95;
const int STOP_SPEED = 0.95;

int intake_state = STOP_STATE;

BallIntake::BallIntake(){
  talonIntake1 = new TalonSRX();//put talon ids in
  talonIntake2 = new TalonSRX();
}

void BallIntake::Init(){
  //init stuff here
}

void BallIntake::Stop(){
talonIntake1->Set(ControlMode::PercentOutput, 0.0 - STOP_SPEED);
talonIntake2->Set(ControlMode::PercentOutput, 0.0 + STOP_SPEED);
}

void BallIntake::In(){
  talonIntake1->Set(ControlMode::PercentOutput, -IN_SPEED);
  talonIntake2->Set(ControlMode::PercentOutput, IN_SPEED);
}

void BallIntake::Out(){
  talonIntake1->Set(ControlMode::PercentOutput, OUT_SPEED);
  talonIntake2->Set(ControlMode::PercentOutput, -OUT_SPEED);
}


void BallIntake::IntakeStateMachine(){

  switch (intake_state) {

    case INIT_STATE:
      Init();
      SmartDashboard::PutString("State", "BALL_INIT_STATE")
      last_intake_state = INIT_STATE;
      break;

    case STOP_STATE:
      Stop();
      SmartDashboard::PutString("State", "BALL_STOP_STATE");
      last_intake_state = STOP_STATE;
      break;

    case IN_BALL_STATE:
      In();
      SmartDashboard::PutString("State", "BALL_IN_STATE");
      last_intake_state = IN_BALL_STATE;
      break;

    case OUT_BALL_STATE:
      Out();
      SmartDashboard::PutString("State", "BALL_OUT_STATE");
      last_intake_state = OUT_BALL_STATE;
      break;

  }
}
