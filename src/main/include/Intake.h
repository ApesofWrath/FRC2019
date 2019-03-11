#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>

class Intake {
public:

  TalonSRX *talonIntake1, *talonIntake2;

  const int TALON_ID_TOP = 26;
  const int TALON_ID_BOT = 46;

  const int STOP_STATE_H = 0;
  const int HOLD_STATE_H = 1;
  const int IN_STATE_H = 2;
  const int OUT_STATE_H = 3;
  const int OUT_SLOW_STATE_H = 4;
  const int IN_SLOW_STATE_H = 5;
  const int OUT_FAST_STATE_H = 6;
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
  void HoldTop();
  void Slow();
  void InTop();
  void OutTop(bool slow);
  void OutFastTop(bool fast);

  void StopBottom();
  void HoldBottom();
  void InBottom(bool slow);
  void OutBottom(bool slow);
  void OutFastBottom(bool fast);

  void IntakeTopStateMachine();
  void IntakeBottomStateMachine();

  bool HaveBall();
  bool ReleasedBall();

};

#endif
