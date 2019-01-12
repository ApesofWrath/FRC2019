/*
 * Arm.h
 *
 *  Created on: Jan 11, 2019
 *      Author: Kaya
 */

class Arm {
public:

  const int INIT_STATE_H = 0;
  const int LOW_HATCH_STATE_H = 1;
  const int MID_HATCH_STATE_H = 2;
  const int HIGH_HATCH_STATE_H = 3;
  const int LOW_CARGO_STATE_H = 4;
  const int MID_CARGO_STATE_H = 5;
  const int HIGH_CARGO_STATE_H = 6;

Arm();

void ArmStateMachine();


}
