#include "TeleopStateMachine.h"

const int INIT_STATE = 0;
const int PLACING_HATCH_STATE = 1;
const int PLACING_BALL_SLOW_STATE = 2;
const int PLACING_BALL_MED_STATE = 3;
const int PLACING_BALL_FAST_STATE = 4;
const int TAKING_HATCH_STATE = 5;
const int TAKING_BALL_STATE = 6;
const int TAKING_GROUND_BALL_STATE = 7;
const int TAKING_GROUND_HATCH_STATE = 8;
int state = INIT_STATE

void TeleopStateMachine::Statemachine(){

  switch (state) {

    case INIT_STATE:
      // Initialize Stuff
      SmartDashboard::PutString("State", "INIT");
      break;

    case PLACING_HATCH_STATE:
      // Set stuff needed to place hatch
      SmartDashboard::PutString("State", "PLACING_HATCH");
      break;

    case PLACING_BALL_SLOW_STATE:
      // Set stuff needed to place ball slowly
      SmartDashboard::PutString("State", "PLACING_BALL_SLOW");
      break;

    case PLACING_BALL_MED_STATE:
      // Set stuff needed to place ball at medium speed
      SmartDashboard::PutString("State", "PLACING_BALL_MED");
      break;

    case PLACING_BALL_FAST_STATE:
      // Set stuff needed to place ball quickly
      SmartDashboard::PutString("State", "PLACING_BALL_FAST");
      break;

    case TAKING_HATCH_STATE:
      // Set stuff needed to take hatch
      SmartDashboard::PutString("State", "TAKING_HATCH");
      break;

    case TAKING_BALL_STATE:
      // Set stuff needed to take ball
      SmartDashboard::PutString("State", "TAKEING_BALL");
      break;

    case TAKING_GROUND_HATCH_STATE:
      // Set stuff needed to take hatch from ground
      SmartDashboard::PutString("State", "TAKING_HATCH_GROUND");
      break;

    case TAKING_GROUND_BALL_STATE:
      // Set stuff needed to take ball from ground
      SmartDashboard::PutString("State", "TAKING_BALL_GROUND");
      break;
  }

}
