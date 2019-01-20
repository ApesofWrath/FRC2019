/*
 * Arm.cpp
 *
 *  Created on: Jan 11, 2019
 *      Author: Kaya
 */

 #include "Arm.h"

 const int INIT_STATE = 0;
 const int WAIT_FOR_BUTTON_STATE = 1;
 const int LOW_HATCH_STATE = 2;
 const int MID_HATCH_STATE = 3;
 const int HIGH_HATCH_STATE = 4;
 const int LOW_CARGO_STATE = 5;
 const int MID_CARGO_STATE = 6;
 const int HIGH_CARGO_STATE = 7;

 int last_arm_state = 0;

 bool state_shoulder = false;
 bool state_wrist = false;

 Shoulder *shoulder;
 Wrist *wrist;

 Arm::Arm(Shoulder *shoulder_, Wrist *wrist_){

   shoulder = shoulder_;
   wrist = wrist_;
 }

 void Arm::ArmStateMachine(bool wait_for_button, bool low_hatch, bool mid_hatch,
      bool high_hatch, bool low_cargo, bool mid_cargo, bool high_cargo){

   switch (arm_state) {

     case INIT_STATE:
     SmartDashboard::PutString("STATE", "INIT");

     break;

     case WAIT_FOR_BUTTON_STATE:
     SmartDashboard::PutString("STATE", "WAIT FOR BUTTON");

     break;

     case LOW_HATCH_STATE:
     SmartDashboard::PutString("STATE", "LOW HATCH");

     break;

     case MID_HATCH_STATE:
     SmartDashboard::PutString("STATE", "MID HATCH");

     break;

     case HIGH_HATCH_STATE:
     SmartDashboard::PutString("STATE", "HIGH HATCH");

     break;

     case LOW_CARGO_STATE:
     SmartDashboard::PutString("STATE", "LOW CARGO");

     break;

     case MID_CARGO_STATE:
     SmartDashboard::PutString("STATE", "MID_CARGO");

     break;

     case HIGH_CARGO_STATE:
     SmartDashboard::PutString("STATE", "HIGH_CARGO");

     break;


   }

 }
