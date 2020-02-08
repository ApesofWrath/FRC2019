#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>

class Arm {
public:

  TalonSRX *talonArm;

  const int REST_STATE_H = 1;
  const int UP_STATE_H = 2;
  const int DOWN_STATE_H = 3;
  int intake_arm_state = REST_STATE;

    Arm();

    void Arm::Up();
    void Arm::Down();
    void Arm::Rest();

    void Arm::IntakeArmStateMachine();

    void Arm::UpperSoftLimit();
    void Arm::LowerSoftLimit();


};

#endif
