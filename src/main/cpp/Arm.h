/*
 * Arm.h
 *
 *  Created on: Jan 7, 2019
 *      Author: DriversStation
 */

 #ifndef SRC_ARM_H_
 #define SRC_ARM_H_

 #include <WPILib.h>
 #include "ctre/Phoenix.h"

class Arm {
public:

  const int INIT_STATE_H = 0;
	const int UP_STATE_H = 1; //arm state machine
  const int HIGH_STATE_H = 2;
	const int MID_STATE_H = 3;
	const int DOWN_STATE_H = 4;
  int arm_state = INIT_STATE_H;

  TalonSRX *talonArm;

  void Down();
  void Up();
  void ArmStateMachine();

private:

};

#endif /* SRC_ARM_H_ */
