/*
 * Arm.h
 *
 *  Created on: Jan 11, 2019
 *      Author: Kaya
 */

class Arm {
public:

  const int INIT_STATE_H = 0;
  const int WAIT_FOR_BUTTON_STATE = 1;
  const int LOW_HATCH_STATE_H = 2;
  const int MID_HATCH_STATE_H = 3;
  const int HIGH_HATCH_STATE_H = 4;
  const int LOW_CARGO_STATE_H = 5;
  const int MID_CARGO_STATE_H = 6;
  const int HIGH_CARGO_STATE_H = 7;

  int arm_state = INIT_STATE_H;

Arm(Shoulder *shoulder_, Wrist *wrist_);

void ArmStateMachine(bool wait_for_button, bool low_hatch, bool mid_hatch,
     bool high_hatch, bool low_cargo, bool mid_cargo, bool high_cargo);


}
