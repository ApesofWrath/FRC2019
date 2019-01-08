/*
 * Arm.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: DriversStation
 */

#include "Arm.h"
#include <ctre/Phoenix.h> //double included
#include <WPILib.h>

#define PI 3.14159265

const int INIT_STATE = 0;
const int UP_STATE = 1;
const int HIGH_STATE = 2;
const int MID_STATE = 3;
const int DOWN_STATE = 4;

int last_arm_state = 0; //cannot equal the first state or profile will not set the first time

ArmMotionProfiler *arm_profiler;

Arm::Arm(){

  talonArm = new TalonSRX(0);

}

void Arm::Down(){
  talonArm->Set(ControlMode::PercentOutput, -0.3);
}

void Arm::Up(){
  talonArm->Set(ControlMode::PercentOutput, 0.3);
}

void Arm::ArmStateMachine(){

  switch (arm_state){

    case INIT_STATE:
    SmartDashboard::PutString("ARM", "INIT");
    if (is_init_arm) {
			arm_state = UP_STATE;
		} else {
			InitializeArm();
		}
		last_arm_state = INIT_STATE;
    break;

    case UP_STATE:
    SmartDashboard::PutString("ARM", "UP");
    if (last_arm_state != UP_STATE) { //first time in state
			arm_profiler->SetFinalGoalArm(UP_ANGLE);
			arm_profiler->SetInitPosArm(GetAngularPosition());
		}
		last_arm_state = UP_STATE;
    break;

    case HIGH_STATE:
    SmartDashboard::PutString("ARM", "HIGH");
    if (last_arm_state != HIGH_STATE) {
			arm_profiler->SetFinalGoalIntake(HIGH_ANGLE);
			arm_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_arm_state = HIGH_STATE;
    break;

    case MID_STATE:
    SmartDashboard::PutString("ARM", "MID");
    if (last_arm_state != MID_STATE) {
			arm_profiler->SetFinalGoalIntake(MID_ANGLE);
			arm_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_arm_state = MID_STATE;
    break;

    case DOWN_STATE:
    SmartDashboard::PutString("ARM", "DOWN");
    if (last_arm_state != DOWN_STATE) {
			arm_profiler->SetFinalGoalArm(DOWN_ANGLE);
			arm_profiler->SetInitPosArm(GetAngularPosition());
		}
		last_arm_state = DOWN_STATE;
    break;
  }
}
