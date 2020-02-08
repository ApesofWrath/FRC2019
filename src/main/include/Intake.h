#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>

class Intake {
public:

  TalonSRX *talonIntake;

  const int STOP_STATE_H = 0;
  const int IN_STATE_H = 1;
  const int OUT_STATE_H = 2;
  int intake_state = STOP_STATE;

    Intake();

    void Intake::Stop();
    void Intake::In();
    void Intake::Out();

    void Intake::IntakeStateMachine();



};

#endif
