#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>

class Intake {
public:

  TalonSRX *talonIntake1, *talonIntake2;

  const int TALON_ID_TOP = 46;
  const int TALON_ID_BOT = 26;

  const int STOP_STATE_H = 0;
  const int IN_STATE_H = 1;
  const int OUT_STATE_H = 2;
  int top_intake_state = STOP_STATE_H;
  int bottom_intake_state = STOP_STATE_H;

  const double STOP_BOTTOM_SPEED = 0.2; //placeholder #s put in correct ones
  const double IN_BOTTOM_SPEED = -0.95;
  const double OUT_BOTTOM_SPEED = 0.95;

  const double STOP_TOP_SPEED = 0.2; //placeholder #s put in correct ones
  const double IN_TOP_SPEED = -0.95;
  const double OUT_TOP_SPEED = 0.95;

  Intake();

  void StopTop();
  void InTop();
  void OutTop();

  void StopBottom();
  void InBottom();
  void OutBottom();

  void IntakeTopStateMachine();
  void IntakeBottomStateMachine();

  bool HaveBall();
  bool ReleasedBall();

};

#endif
